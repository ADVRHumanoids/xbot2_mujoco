"""IsaacLab-like seeded terrain generation for MuJoCo."""

from __future__ import annotations
import argparse
import copy, hashlib, json, pickle, warnings
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
import numpy as np
from PIL import Image
import yaml


@dataclass
class SubTerrainBaseCfg:
    """Common settings for one terrain type."""

    proportion: float = 1.0


@dataclass
class MeshPyramidStairsTerrainCfg(SubTerrainBaseCfg):
    """Pyramid stairs made from MuJoCo box geoms."""

    step_height_range: tuple[float, float] = (0.05, 0.2)
    step_width: float = 0.3
    platform_width: float = 1.0
    border_width: float = 0.0
    holes: bool = False


@dataclass
class MeshInvertedPyramidStairsTerrainCfg(MeshPyramidStairsTerrainCfg):
    pass


@dataclass
class MeshRandomGridTerrainCfg(SubTerrainBaseCfg):
    """Random-height box grid terrain."""

    grid_width: float = 0.45
    grid_height_range: tuple[float, float] = (0.05, 0.2)
    platform_width: float = 1.0
    holes: bool = False


@dataclass
class HfRandomUniformTerrainCfg(SubTerrainBaseCfg):
    """Heightfield terrain sampled from uniform random noise."""

    noise_range: tuple[float, float] = (-0.05, 0.05)
    noise_step: float = 0.005
    border_width: float = 0.0
    downsampled_scale: float | None = None


@dataclass
class HfPyramidSlopedTerrainCfg(SubTerrainBaseCfg):
    """Heightfield terrain with a sloped pyramid profile."""

    slope_range: tuple[float, float] = (0.0, 0.4)
    platform_width: float = 1.0
    border_width: float = 0.0


@dataclass
class HfInvertedPyramidSlopedTerrainCfg(HfPyramidSlopedTerrainCfg):
    pass


@dataclass
class TerrainGeneratorCfg:
    """Configuration for a tiled, seeded terrain world.

    ``size`` is the width and depth of each tile. ``sub_terrains`` maps names
    to terrain configurations; ``proportion`` controls type selection. The
    grid contains ``num_rows`` by ``num_cols`` tiles. Heightfields use the
    specified horizontal and vertical quantization scales. ``border_width``
    and ``border_height`` define the outer wall. ``curriculum`` lays out
    terrain types by column and difficulty by row; otherwise types and
    difficulties are sampled randomly within ``difficulty_range``.
    ``spawn_height`` is added to each generated spawn Z coordinate. Caching is
    controlled by ``use_cache`` and ``cache_dir``; ``seed`` makes generation
    reproducible.
    """

    size: tuple[float, float]
    sub_terrains: dict[str, SubTerrainBaseCfg]
    border_width: float = 0.0
    border_height: float = 1.0
    num_rows: int = 1
    num_cols: int = 1
    horizontal_scale: float = 0.1
    vertical_scale: float = 0.005
    slope_threshold: float | None = 0.75
    use_cache: bool = False
    cache_dir: str = "/tmp/xbot2_mujoco/terrains"
    curriculum: bool = False
    difficulty_range: tuple[float, float] = (0.0, 1.0)
    spawn_height: float = 1.0
    seed: int | None = None


@dataclass
class _Patch:
    name: str
    difficulty: float
    origin_z: float
    boxes: list = field(default_factory=list)
    heightfield: np.ndarray | None = None


class TerrainGenerator:
    def __init__(self, cfg):
        self.cfg = copy.deepcopy(cfg)
        self._validate()
        if cfg.use_cache and cfg.seed is None:
            warnings.warn(
                "use_cache=True without an explicit seed is not reproducible",
                RuntimeWarning,
                stacklevel=2,
            )
        if (
            any(
                isinstance(
                    x, (HfRandomUniformTerrainCfg, HfPyramidSlopedTerrainCfg)
                )
                for x in cfg.sub_terrains.values()
            )
            and cfg.slope_threshold is not None
        ):
            warnings.warn(
                "slope_threshold is not portable to native MuJoCo "
                "heightfields and is ignored",
                RuntimeWarning,
                stacklevel=2,
            )
        # Keep terrain generation independent from NumPy global RNG state.
        seed = (
            cfg.seed
            if cfg.seed is not None
            else int(np.random.get_state()[1][0])
        )
        self._seed = seed
        self._rng = np.random.default_rng(seed)
        shape = (cfg.num_rows, cfg.num_cols)
        self.terrain_origins = np.zeros((*shape, 3))
        self.terrain_types = np.empty(shape, object)
        self.difficulties = np.zeros(shape)
        self.xml_tree = None
        if not self._load():
            self._generate()
            self._store()

    def _validate(self):
        c = self.cfg
        if len(c.size) != 2 or min(c.size) <= 0:
            raise ValueError("size must contain two positive dimensions")
        if min(c.num_rows, c.num_cols) <= 0:
            raise ValueError("num_rows and num_cols must be positive")
        if min(c.horizontal_scale, c.vertical_scale) <= 0:
            raise ValueError("scales must be positive")
        if not c.sub_terrains:
            raise ValueError("sub_terrains must not be empty")
        if (
            any(x.proportion < 0 for x in c.sub_terrains.values())
            or sum(x.proportion for x in c.sub_terrains.values()) <= 0
        ):
            raise ValueError("proportions require a positive sum")
        if not 0 <= c.difficulty_range[0] <= c.difficulty_range[1] <= 1:
            raise ValueError("difficulty_range must lie in [0, 1]")
        for n, x in c.sub_terrains.items():
            if isinstance(x, MeshPyramidStairsTerrainCfg) and (
                x.step_width <= 0
                or x.platform_width + 2 * x.step_width
                > min(c.size) - 2 * x.border_width
            ):
                raise ValueError(f"{n}: stairs do not fit")
            if isinstance(x, MeshRandomGridTerrainCfg) and (
                x.grid_width <= 0 or not np.isclose(c.size[0], c.size[1])
            ):
                raise ValueError(
                    f"{n}: random grid requires positive width and square size"
                )
            if isinstance(x, HfRandomUniformTerrainCfg) and (
                x.noise_step <= 0
                or (x.downsampled_scale or c.horizontal_scale)
                < c.horizontal_scale
            ):
                raise ValueError(f"{n}: invalid noise sampling")

    def _key(self):
        d = asdict(self.cfg)
        d["types"] = {
            k: type(v).__name__ for k, v in self.cfg.sub_terrains.items()
        }
        d["seed"] = self._seed
        return hashlib.sha256(
            json.dumps(d, sort_keys=True).encode()
        ).hexdigest()

    def _path(self):
        return Path(self.cfg.cache_dir) / self._key() / "terrain.pkl"

    def _load(self):
        if not self.cfg.use_cache or not self._path().is_file():
            return False
        with self._path().open("rb") as f:
            (
                self._patches,
                self.terrain_origins,
                self.terrain_types,
                self.difficulties,
            ) = pickle.load(f)
        return True

    def _store(self):
        if self.cfg.use_cache:
            self._path().parent.mkdir(parents=True, exist_ok=True)
            with self._path().open("wb") as f:
                pickle.dump(
                    (
                        self._patches,
                        self.terrain_origins,
                        self.terrain_types,
                        self.difficulties,
                    ),
                    f,
                )

    def _center(self, r, c, z=0):
        sx, sy = self.cfg.size
        return np.array(
            (
                (r + 0.5 - self.cfg.num_rows / 2) * sx,
                (c + 0.5 - self.cfg.num_cols / 2) * sy,
                z,
            )
        )

    def _generate(self):
        names = list(self.cfg.sub_terrains)
        cs = list(self.cfg.sub_terrains.values())
        p = np.array([x.proportion for x in cs])
        p = p / p.sum()
        # IsaacLab assigns curriculum terrain types by column proportions.
        ci = np.searchsorted(
            np.cumsum(p),
            np.arange(self.cfg.num_cols) / self.cfg.num_cols + 0.001,
        )
        lo, hi = self.cfg.difficulty_range
        self._patches = [
            [None] * self.cfg.num_cols for _ in range(self.cfg.num_rows)
        ]
        for c in range(self.cfg.num_cols):
            for r in range(self.cfg.num_rows):
                if self.cfg.curriculum:
                    i = int(ci[c])
                    d = (
                        lo
                        + (hi - lo)
                        * (r + self._rng.uniform())
                        / self.cfg.num_rows
                    )
                else:
                    i = int(self._rng.choice(len(cs), p=p))
                    d = self._rng.uniform(lo, hi)
                q = self._patch(names[i], cs[i], d)
                self._patches[r][c] = q
                self.terrain_types[r, c] = names[i]
                self.difficulties[r, c] = d
                self.terrain_origins[r, c] = self._center(r, c, q.origin_z)

    @staticmethod
    # Four boxes form a non-overlapping rectangular ring.  A single mesh would
    # be convexified by MuJoCo and would fill the stairs or depression.
    def _ring(x, y, w, b, t):
        h = max(abs(t - b) / 2, 0.001)
        z = (b + t) / 2
        iy = max(y - 2 * w, 0)
        return [
            ((0, y / 2 - w / 2, z), (x / 2, w / 2, h)),
            ((0, -y / 2 + w / 2, z), (x / 2, w / 2, h)),
            ((x / 2 - w / 2, 0, z), (w / 2, iy / 2, h)),
            ((-x / 2 + w / 2, 0, z), (w / 2, iy / 2, h)),
        ]

    def _patch(self, n, c, d):
        lerp = lambda a: a[0] + d * (a[1] - a[0])
        if isinstance(c, MeshPyramidStairsTerrainCfg):
            inv = isinstance(c, MeshInvertedPyramidStairsTerrainCfg)
            sx, sy = self.cfg.size
            ux, uy = sx - 2 * c.border_width, sy - 2 * c.border_width
            k = int(
                min(
                    (ux - c.platform_width) / (2 * c.step_width),
                    (uy - c.platform_width) / (2 * c.step_width),
                )
            )
            h = lerp(c.step_height_range)
            floor = -(k + 1) * h if inv else 0
            boxes = []
            for i in range(k):
                boxes += self._ring(
                    ux - 2 * i * c.step_width,
                    uy - 2 * i * c.step_width,
                    c.step_width,
                    floor,
                    -i * h if inv else (i + 1) * h,
                )
            top = -k * h if inv else (k + 1) * h
            boxes += [
                (
                    (0, 0, (floor + top) / 2),
                    (
                        (ux - 2 * k * c.step_width) / 2,
                        (uy - 2 * k * c.step_width) / 2,
                        max(abs(top - floor) / 2, 0.001),
                    ),
                )
            ]
            if c.border_width and not c.holes:
                boxes += self._ring(sx, sy, c.border_width, min(floor, 0), 0)
            if c.holes:
                boxes = [
                    b
                    for b in boxes
                    if abs(b[0][0]) < c.platform_width / 2
                    or abs(b[0][1]) < c.platform_width / 2
                ]
            return _Patch(n, d, top, boxes)
        if isinstance(c, MeshRandomGridTerrainCfg):
            s = self.cfg.size[0]
            k = max(1, int(s / c.grid_width))
            w = s / k
            amp = lerp(c.grid_height_range)
            floor = -amp - 0.01
            boxes = []
            for i in range(k):
                for j in range(k):
                    x, y = -s / 2 + (i + 0.5) * w, -s / 2 + (j + 0.5) * w
                    if c.holes and not (
                        abs(x) <= c.platform_width / 2
                        or abs(y) <= c.platform_width / 2
                    ):
                        continue
                    top = (
                        0
                        if abs(x) <= c.platform_width / 2
                        and abs(y) <= c.platform_width / 2
                        else self._rng.uniform(-amp, amp)
                    )
                    boxes.append(
                        (
                            (x, y, (floor + top) / 2),
                            (w / 2, w / 2, (top - floor) / 2),
                        )
                    )
            return _Patch(n, d, 0, boxes)
        nx, ny = (
            round(x / self.cfg.horizontal_scale) + 1 for x in self.cfg.size
        )
        x = np.linspace(-self.cfg.size[0] / 2, self.cfg.size[0] / 2, nx)
        y = np.linspace(-self.cfg.size[1] / 2, self.cfg.size[1] / 2, ny)
        if isinstance(c, HfRandomUniformTerrainCfg):
            vals = np.arange(
                c.noise_range[0],
                c.noise_range[1] + c.noise_step / 2,
                c.noise_step,
            )
            h = self._rng.choice(vals, (nx, ny))
        else:
            dx = np.maximum(abs(x)[:, None] - c.platform_width / 2, 0)
            dy = np.maximum(abs(y)[None, :] - c.platform_width / 2, 0)
            usable_x = self.cfg.size[0] / 2 - c.border_width
            usable_y = self.cfg.size[1] / 2 - c.border_width
            h = lerp(c.slope_range) * np.minimum(
                np.maximum(usable_x - c.platform_width / 2 - dx, 0),
                np.maximum(usable_y - c.platform_width / 2 - dy, 0),
            )
            if isinstance(c, HfInvertedPyramidSlopedTerrainCfg):
                h = -h
        h = np.round(h / self.cfg.vertical_scale) * self.cfg.vertical_scale
        if c.border_width:
            h[
                (abs(x)[:, None] > self.cfg.size[0] / 2 - c.border_width)
                | (abs(y)[None, :] > self.cfg.size[1] / 2 - c.border_width)
            ] = 0
        return _Patch(n, d, float(h[nx // 2, ny // 2]), heightfield=h)

    @staticmethod
    def _v(*x):
        return " ".join(f"{float(a):.12g}" for a in x)

    def build_xml(self, asset_dir, relative_assets=False):
        ad = Path(asset_dir).resolve()
        ad.mkdir(parents=True, exist_ok=True)
        root = ET.Element("mujoco", model="generated_terrain")
        de = ET.SubElement(
            ET.SubElement(root, "default"), "default", {"class": "terrain"}
        )
        ET.SubElement(
            de, "geom", friction=".7 .005 .001", condim="3", solref=".004 1.2"
        )
        asset = ET.SubElement(root, "asset")
        ET.SubElement(
            asset, "material", name="terrain_grid", rgba=".25 .35 .25 1"
        )
        world = ET.SubElement(root, "worldbody")
        for r, row in enumerate(self._patches):
            for c, q in enumerate(row):
                cx, cy, _ = self._center(r, c)
                pre = f"terrain_r{r}_c{c}_{q.name}"
                if q.heightfield is not None:
                    mn, mx = float(q.heightfield.min()), float(
                        q.heightfield.max()
                    )
                    zr = max(mx - mn, self.cfg.vertical_scale)
                    pix = np.rint(
                        np.clip((q.heightfield - mn) / zr, 0, 1) * 65535
                    ).astype(np.uint16)
                    filename = f"{pre}.png"
                    fn = ad / filename
                    # MuJoCo image rows map to y, while arrays here are (x, y).
                    Image.fromarray(pix.T, mode="I;16").save(
                        fn, format="PNG", compress_level=9
                    )
                    hn = pre + "_hfield"
                    ET.SubElement(
                        asset,
                        "hfield",
                        name=hn,
                        file=(
                            f"{ad.name}/{filename}"
                            if relative_assets
                            else str(fn.resolve())
                        ),
                        size=self._v(
                            self.cfg.size[0] / 2,
                            self.cfg.size[1] / 2,
                            zr,
                            max(self.cfg.vertical_scale, 0.001),
                        ),
                    )
                    ET.SubElement(
                        world,
                        "geom",
                        name=pre,
                        type="hfield",
                        hfield=hn,
                        pos=self._v(cx, cy, mn),
                        material="terrain_grid",
                        **{"class": "terrain"},
                    )
                else:
                    for i, (p, s) in enumerate(q.boxes):
                        if min(s) > 0:
                            ET.SubElement(
                                world,
                                "geom",
                                name=f"{pre}_box_{i}",
                                type="box",
                                pos=self._v(cx + p[0], cy + p[1], p[2]),
                                size=self._v(*s),
                                material="terrain_grid",
                                **{"class": "terrain"},
                            )
        # Surround the complete grid with four static box geoms.
        if self.cfg.border_width:
            ix, iy = (
                self.cfg.num_rows * self.cfg.size[0],
                self.cfg.num_cols * self.cfg.size[1],
            )
            w = self.cfg.border_width
            b, t = (
                (-self.cfg.border_height, 0)
                if self.cfg.border_height > 0
                else (0, abs(self.cfg.border_height))
            )
            for i, (p, s) in enumerate(
                self._ring(ix + 2 * w, iy + 2 * w, w, b, t)
            ):
                ET.SubElement(
                    world,
                    "geom",
                    name=f"terrain_outer_border_{i}",
                    type="box",
                    pos=self._v(*p),
                    size=self._v(*s),
                    material="terrain_grid",
                    **{"class": "terrain"},
                )
        self.xml_tree = ET.ElementTree(root)
        ET.indent(self.xml_tree, space="  ")
        return self.xml_tree

    def to_xml_string(self, asset_dir):
        return ET.tostring(
            self.build_xml(asset_dir).getroot(), encoding="unicode"
        )

    def save(self, mjcf_path):
        p = Path(mjcf_path).resolve()
        p.parent.mkdir(parents=True, exist_ok=True)
        self.build_xml(
            p.parent / f"{p.stem}_assets", relative_assets=True
        ).write(
            p, encoding="unicode", xml_declaration=False
        )
        (p.parent / "spawn_locations.yaml").write_text(
            yaml.safe_dump(
                {
                    "spawn_locations": (
                        self.terrain_origins
                        + np.array((0.0, 0.0, self.cfg.spawn_height))
                    ).reshape(-1, 3).tolist()
                },
                sort_keys=False,
            )
        )
        return p


ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    seed=0,  # Deterministic terrain generation.
    size=(8.0, 8.0),  # Width and depth of each terrain tile in meters.
    border_width=20.0,  # Width of the outer collision border in meters.
    border_height=1.0,  # Height of the outer collision border in meters.
    num_rows=4,  # Number of terrain rows.
    num_cols=6,  # Number of terrain columns.
    horizontal_scale=0.1,  # Heightfield sample spacing in meters.
    vertical_scale=0.005,  # Heightfield quantization step in meters.
    slope_threshold=0.75,  # Retained for compatibility; ignored by MuJoCo heightfields.
    use_cache=False,  # Whether to cache generated terrain patches.
    cache_dir="/tmp/xbot2_mujoco/terrains",  # Directory used for terrain caches.
    curriculum=True,  # Arrange terrain types by columns and difficulty by rows.
    difficulty_range=(0.0, 1.0),  # Difficulty interval used for terrain sampling.
    spawn_height=1.0,  # Height added above each terrain origin for robot spawning.
    sub_terrains={
        "pyramid_stairs": MeshPyramidStairsTerrainCfg(
            proportion=0.2,  # Share of randomly selected terrain types.
            step_height_range=(0.05, 0.1),  # Minimum and maximum step height.
            step_width=0.3,  # Width of each stair step in meters.
            platform_width=3.0,  # Flat center platform width in meters.
            border_width=1.0,  # Inset from the tile edge in meters.
        ),
        "pyramid_stairs_inv": MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.2,  # Share of randomly selected terrain types.
            step_height_range=(0.05, 0.1),  # Minimum and maximum step height.
            step_width=0.3,  # Width of each stair step in meters.
            platform_width=3.0,  # Flat center platform width in meters.
            border_width=1.0,  # Inset from the tile edge in meters.
        ),
        "boxes": MeshRandomGridTerrainCfg(
            proportion=0.2,  # Share of randomly selected terrain types.
            grid_width=0.45,  # Width of each random box cell in meters.
            grid_height_range=(0.05, 0.1),  # Minimum and maximum box height.
            platform_width=2.0,  # Flat center platform width in meters.
        ),
        "random_rough": HfRandomUniformTerrainCfg(
            proportion=0.2,  # Share of randomly selected terrain types.
            noise_range=(0.02, 0.2),  # Minimum and maximum noise height.
            noise_step=0.02,  # Quantization step for sampled noise values.
            border_width=0.25,  # Flat inset border width in meters.
        ),
        "hf_pyramid_slope": HfPyramidSlopedTerrainCfg(
            proportion=0.1,  # Share of randomly selected terrain types.
            slope_range=(0, 0.4),  # Minimum and maximum pyramid slope.
            platform_width=2.0,  # Flat center platform width in meters.
            border_width=0.25,  # Flat inset border width in meters.
        ),
        "hf_pyramid_slope_inv": HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.1,  # Share of randomly selected terrain types.
            slope_range=(0, 0.4),  # Minimum and maximum inverted pyramid slope.
            platform_width=2.0,  # Flat center platform width in meters.
            border_width=0.25,  # Flat inset border width in meters.
        ),
    },
)


def main(argv: list[str] | None = None) -> int:
    """Generate the default rough terrain from the command line."""
    parser = argparse.ArgumentParser(
        description="Save ROUGH_TERRAINS_CFG as a MuJoCo world."
    )
    parser.add_argument(
        "output_dir",
        type=Path,
        help="Directory for the MJCF and its heightfield assets.",
    )
    parser.add_argument(
        "--filename",
        default="rough_terrain.xml",
        help="MJCF filename inside OUTPUT_DIR (default: %(default)s).",
    )
    args = parser.parse_args(argv)

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = TerrainGenerator(ROUGH_TERRAINS_CFG).save(
        output_dir / args.filename
    )
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
