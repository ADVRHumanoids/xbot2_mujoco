from pathlib import Path
import mujoco, numpy as np, pytest, yaml
from xbot2_py_mujoco.mjcf_tools import MjcfGenerator
from xbot2_py_mujoco.utils import terrain_generator as tg


def cfg(
    seed=42,
    curriculum=True,
    use_cache=False,
    cache_dir="/tmp/terrain_test_cache",
):
    terrains = {
        "stairs": tg.MeshPyramidStairsTerrainCfg(1, (0.05, 0.1), 0.2, 0.6, 0.1),
        "stairs_inv": tg.MeshInvertedPyramidStairsTerrainCfg(
            1, (0.05, 0.1), 0.2, 0.6, 0.1
        ),
        "boxes": tg.MeshRandomGridTerrainCfg(1, 0.4, (0.05, 0.2), 0.4),
        "rough": tg.HfRandomUniformTerrainCfg(1, (-0.05, 0.05), 0.01, 0.2),
        "slope": tg.HfPyramidSlopedTerrainCfg(1, (0.1, 0.3), 0.6, 0.2),
        "slope_inv": tg.HfInvertedPyramidSlopedTerrainCfg(
            1, (0.1, 0.3), 0.6, 0.2
        ),
    }
    return tg.TerrainGeneratorCfg(
        seed=seed,
        size=(2.0, 2.0),
        num_rows=2,
        num_cols=6,
        border_width=0.2,
        horizontal_scale=0.2,
        vertical_scale=0.01,
        slope_threshold=None,
        curriculum=curriculum,
        use_cache=use_cache,
        cache_dir=cache_dir,
        sub_terrains=terrains,
    )


def test_reproducible_seed_and_assets(tmp_path):
    a, b = tg.TerrainGenerator(cfg(curriculum=False)), tg.TerrainGenerator(
        cfg(curriculum=False)
    )
    assert np.array_equal(a.terrain_types, b.terrain_types)
    assert np.array_equal(a.difficulties, b.difficulties)
    xa, xb = a.to_xml_string(tmp_path / "a"), b.to_xml_string(tmp_path / "b")
    assert xa.replace(str((tmp_path / "a").resolve()), "A") == xb.replace(
        str((tmp_path / "b").resolve()), "A"
    )
    assert [p.read_bytes() for p in sorted((tmp_path / "a").glob("*.png"))] == [
        p.read_bytes() for p in sorted((tmp_path / "b").glob("*.png"))
    ]
    assert not np.array_equal(
        a.difficulties, tg.TerrainGenerator(cfg(7, False)).difficulties
    )


def test_curriculum_layout_origins_and_mujoco_load(tmp_path):
    g = tg.TerrainGenerator(cfg())
    assert g.terrain_types[0].tolist() == list(cfg().sub_terrains)
    assert np.all(g.difficulties[0] < 0.5) and np.all(g.difficulties[1] >= 0.5)
    assert np.isclose(g.terrain_origins[:, :, :2].mean(axis=(0, 1)), 0).all()
    m = mujoco.MjModel.from_xml_path(str(g.save(tmp_path / "world.xml")))
    assert m.nhfield == 6 and m.ngeom > 20
    # Heightfields, box tiles and outer borders all inherit the terrain group.
    panels = m.geom_group == 3
    assert np.count_nonzero(panels) > 12
    assert np.all(m.geom_group[~panels] == 2)
    assert np.all(m.geom_contype[panels] == 0)
    assert np.all(m.geom_conaffinity[panels] == 0)


def test_saved_world_is_relocatable(tmp_path):
    output_dir = tmp_path / "generated"
    world = tg.TerrainGenerator(
        tg.TerrainGeneratorCfg(
            seed=1,
            size=(2, 2),
            slope_threshold=None,
            num_cols=2,
            curriculum=True,
            sub_terrains={
                "rough": tg.HfRandomUniformTerrainCfg(),
                "stairs": tg.MeshPyramidStairsTerrainCfg(),
            },
        )
    ).save(output_dir / "terrain.xml")

    relocated = tmp_path / "relocated"
    output_dir.rename(relocated)
    model = mujoco.MjModel.from_xml_path(str(relocated / world.name))

    assert model.nhfield == 1
    assert np.any(model.geom_group == 3)


@pytest.mark.parametrize("terrain_class", [
    tg.MeshPyramidStairsTerrainCfg, tg.MeshInvertedPyramidStairsTerrainCfg,
    tg.MeshOpenPyramidStairsTerrainCfg, tg.MeshOpenInvertedPyramidStairsTerrainCfg,
])
def test_step_labels_are_centered_on_vertical_faces_and_visual_only(tmp_path, terrain_class):
    generator = tg.TerrainGenerator(tg.TerrainGeneratorCfg(
        seed=1, size=(2, 2), sub_terrains={"stairs": terrain_class(
            step_height_range=(0.2, 0.2), step_width=0.2, platform_width=0.6,
            border_width=0.1,
        )},
    ))
    world = generator.save(tmp_path / "world.xml")
    model = mujoco.MjModel.from_xml_path(str(world))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    labels = np.flatnonzero(model.geom_group == 3)
    # Four faces per riser; a solid inverted landing has no exposed sides.
    assert len(labels) == (12 if terrain_class is tg.MeshInvertedPyramidStairsTerrainCfg else 16)
    assert np.all(model.geom_contype[labels] == 0)
    assert np.all(model.geom_conaffinity[labels] == 0)
    assert np.all(model.geom_group[np.setdiff1d(np.arange(model.ngeom), labels)] == 2)
    texture = model.texture("terrain_r0_c0_stairs_step_height")
    assert texture.width[0] == 1024 and texture.height[0] == 384
    assert (tmp_path / "world_assets" / "terrain_r0_c0_stairs_step_height.png").is_file()
    groups = np.array([0, 0, 1, 0, 1, 1], dtype=np.uint8)
    for label_id in labels:
        rotation = data.geom_xmat[label_id].reshape(3, 3)
        normal = rotation[:, 2]
        assert np.allclose(rotation[:, 1], [0, 0, 1])  # Text is upright.
        assert normal[2] == pytest.approx(0)
        origin = model.geom_pos[label_id] + 0.05 * normal
        hit = np.array([-1], dtype=np.int32)
        mujoco.mj_ray(model, data, origin, -normal, None, 1, -1, hit)
        assert hit[0] == label_id
        distance = mujoco.mj_ray(model, data, origin, -normal, groups, 1, -1, hit)
        assert distance == pytest.approx(0.0515)
        assert model.geom_group[hit[0]] == 2
        top = model.geom_pos[hit[0], 2] + model.geom_size[hit[0], 2]
        face_height = (0.02 if isinstance(generator.cfg.sub_terrains["stairs"], tg.MeshOpenPyramidStairsTerrainCfg)
                       else 0.2)
        assert model.geom_pos[label_id, 2] == pytest.approx(top - face_height / 2)


def test_merge_with_mjcf_generator(tmp_path):
    g = tg.TerrainGenerator(
        tg.TerrainGeneratorCfg(
            seed=1,
            size=(2, 2),
            horizontal_scale=0.2,
            slope_threshold=None,
            num_cols=2,
            curriculum=True,
            sub_terrains={
                "rough": tg.HfRandomUniformTerrainCfg(),
                "stairs": tg.MeshPyramidStairsTerrainCfg(),
            },
        )
    )
    world = g.save(tmp_path / "terrain.xml")
    resources = Path(__file__).parent / "resources"
    merger = MjcfGenerator(
        "terrain_merge",
        (resources / "nice_robot.urdf").read_text(),
        output_dir=str(tmp_path / "robot"),
    )
    merger.merge_xml(world)
    model = mujoco.MjModel.from_xml_string(merger.generate_mjcf_string())
    assert model.nhfield == 1
    assert (
        mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "terrain_r0_c0_rough"
        )
        >= 0
    )
    assert model.geom("terrain_r0_c0_rough").group[0] == 2
    assert np.any(model.geom_group == 3)


def test_cache_and_warnings(tmp_path):
    c = cfg(use_cache=True, cache_dir=str(tmp_path))
    a = tg.TerrainGenerator(c)
    b = tg.TerrainGenerator(c)
    assert np.array_equal(a.terrain_origins, b.terrain_origins)
    assert next(tmp_path.rglob("terrain.pkl")).is_file()
    with pytest.warns(RuntimeWarning, match="slope_threshold"):
        tg.TerrainGenerator(
            tg.TerrainGeneratorCfg(
                size=(2, 2), sub_terrains={"h": tg.HfRandomUniformTerrainCfg()}
            )
        )
    with pytest.warns(RuntimeWarning, match="explicit seed"):
        tg.TerrainGenerator(
            tg.TerrainGeneratorCfg(
                size=(2, 2),
                slope_threshold=None,
                use_cache=True,
                cache_dir=str(tmp_path / "u"),
                sub_terrains={
                    "s": tg.MeshPyramidStairsTerrainCfg(platform_width=0.5)
                },
            )
        )


def test_validation():
    with pytest.raises(ValueError, match="positive sum"):
        tg.TerrainGenerator(
            tg.TerrainGeneratorCfg(
                size=(2, 2),
                sub_terrains={"x": tg.MeshRandomGridTerrainCfg(proportion=0)},
            )
        )


def test_inverted_stairs_have_uniform_step_heights():
    config = tg.TerrainGeneratorCfg(
        size=(2.0, 2.0),
        sub_terrains={
            "stairs": tg.MeshInvertedPyramidStairsTerrainCfg(
                step_height_range=(0.1, 0.1),
                step_width=0.2,
                platform_width=0.6,
            )
        },
    )
    generator = tg.TerrainGenerator(config)
    patch = generator._patches[0][0]
    k = int((2.0 - 0.6) / (2 * 0.2))
    expected_levels = np.arange(0, -k - 1, -1) * 0.1
    ring_levels = [patch.boxes[index][0][2] + patch.boxes[index][1][2] for index in range(0, 4 * k, 4)]
    center = patch.boxes[4 * k]
    ring_levels.append(center[0][2] + center[1][2])

    assert np.allclose(ring_levels, expected_levels)


def test_sloped_heightfield_reaches_zero_at_border():
    config = tg.TerrainGeneratorCfg(
        size=(2.0, 2.0),
        horizontal_scale=0.1,
        vertical_scale=0.01,
        slope_threshold=None,
        sub_terrains={
            "slope": tg.HfPyramidSlopedTerrainCfg(
                slope_range=(0.4, 0.4),
                platform_width=0.6,
                border_width=0.2,
            )
        },
    )
    heightfield = tg.TerrainGenerator(config)._patches[0][0].heightfield
    profile = heightfield[:, heightfield.shape[1] // 2]

    assert profile[-3] == 0
    assert profile[-1] == 0


@pytest.mark.parametrize("inverted", [False, True])
@pytest.mark.parametrize("border_width", [0.0, 0.1])
@pytest.mark.parametrize("overlap", [0.0, 0.05])
def test_open_stairs_geometry_and_collision(tmp_path, inverted, border_width, overlap):
    terrain_class = (
        tg.MeshOpenInvertedPyramidStairsTerrainCfg
        if inverted else tg.MeshOpenPyramidStairsTerrainCfg
    )
    config = tg.TerrainGeneratorCfg(
        seed=3,
        size=(2.0, 2.4),
        difficulty_range=(0.5, 0.5),
        sub_terrains={"open_stairs": terrain_class(
            step_height_range=(0.08, 0.12),
            step_width=0.2,
            platform_width=0.6,
            border_width=border_width,
            tread_thickness=0.02,
            tread_overlap=overlap,
        )},
    )
    generator = tg.TerrainGenerator(config)
    patch = generator._patches[0][0]
    # Three rings fit even when the available width rounds just below 3.
    assert len(patch.boxes) == 12 + (4 if border_width else 0) + 2
    direction = -1 if inverted else 1
    ring_offset = 4 if border_width else 0
    for k in range(3):
        ring = patch.boxes[ring_offset + 4 * k:ring_offset + 4 * (k + 1)]
        outer = border_width + k * 0.2
        inner = outer + 0.2 + (overlap if inverted else 0)
        if not inverted:
            outer = max(0, outer - overlap)
        assert all(p[2] + s[2] == pytest.approx(direction * (k + 1) * 0.1)
                   for p, s in ring)
        assert ring[0][1][1] * 2 == pytest.approx(inner - outer)
        # The plates partition the rectangular ring without filling its opening.
        area = sum(4 * s[0] * s[1] for _, s in ring)
        assert area == pytest.approx(
            (2.0 - 2 * outer) * (2.4 - 2 * outer)
            - (2.0 - 2 * inner) * (2.4 - 2 * inner)
        )
    landing, floor = patch.boxes[-2:]
    assert landing[0][:2] == pytest.approx((0, 0))
    landing_inset = border_width + 0.6 - (overlap if not inverted else 0)
    assert landing[1][:2] == pytest.approx((1 - landing_inset, 1.2 - landing_inset))
    assert patch.origin_z == pytest.approx(direction * 0.4)
    assert floor[0][2] + floor[1][2] == pytest.approx(
        min(direction * 0.4, 0 if border_width else 0.1) - 0.5
    )
    assert all(s[2] * 2 == pytest.approx(0.02) for _, s in patch.boxes)
    assert all(min(s) > 0 for _, s in patch.boxes)

    world = generator.save(tmp_path / "open_stairs.xml")
    model = mujoco.MjModel.from_xml_path(str(world))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.ngeom == len(patch.boxes) + 16
    assert np.count_nonzero(model.geom_group == 2) == len(patch.boxes)
    geom_id = np.array([-1], dtype=np.int32)
    groups = np.array([0, 0, 1, 0, 1, 1], dtype=np.uint8)
    # A horizontal ray passes through the first riser's vertical clearance.
    assert mujoco.mj_ray(model, data, np.array([-2., 0., direction * 0.05]),
                         np.array([1., 0., 0.]), groups, 1, -1, geom_id) == -1
    # A vertical ray at the origin lands on the central walking surface.
    distance = mujoco.mj_ray(model, data, np.array([0., 0., 1.]),
                             np.array([0., 0., -1.]), groups, 1, -1, geom_id)
    assert distance == pytest.approx(1 - patch.origin_z)
    spawns = yaml.safe_load((tmp_path / "spawn_locations.yaml").read_text())
    assert spawns["spawn_locations"][0] == pytest.approx(
        (0, 0, patch.origin_z + config.spawn_height)
    )


@pytest.mark.parametrize("kwargs, message", [
    ({"step_height_range": (0, 0.1)}, "step_height_range"),
    ({"step_height_range": (0.2, 0.1)}, "step_height_range"),
    ({"step_height_range": (0.1, float("nan"))}, "step_height_range"),
    ({"step_width": 0}, "step_width"),
    ({"platform_width": float("inf")}, "platform_width"),
    ({"border_width": -0.1}, "border_width"),
    ({"tread_thickness": 0}, "tread_thickness"),
    ({"tread_thickness": 0.05}, "minimum step height"),
    ({"tread_overlap": -0.1}, "tread_overlap"),
    ({"tread_overlap": 0.3}, "tread_overlap"),
    ({"tread_overlap": float("nan")}, "tread_overlap"),
    ({"platform_width": 2.0}, "do not fit"),
    ({"inverted": True, "platform_width": 0.1, "tread_overlap": 0.25}, "positive opening"),
])
def test_open_stairs_validation(kwargs, message):
    with pytest.raises(ValueError, match=message):
        tg.TerrainGenerator(tg.TerrainGeneratorCfg(
            size=(2, 2),
            sub_terrains={"stairs": tg.MeshOpenPyramidStairsTerrainCfg(**kwargs)},
        ))


def test_open_stairs_cache_and_curriculum(tmp_path):
    config = tg.TerrainGeneratorCfg(
        seed=42,
        size=(2, 2),
        num_rows=2,
        num_cols=2,
        curriculum=True,
        use_cache=True,
        cache_dir=str(tmp_path / "cache"),
        sub_terrains={
            "up": tg.MeshOpenPyramidStairsTerrainCfg(platform_width=0.6),
            "down": tg.MeshOpenInvertedPyramidStairsTerrainCfg(platform_width=0.6),
        },
    )
    first, cached = tg.TerrainGenerator(config), tg.TerrainGenerator(config)
    assert first.terrain_types.tolist() == [["up", "down"], ["up", "down"]]
    assert first.terrain_origins[1, 0, 2] > first.terrain_origins[0, 0, 2] > 0
    assert first.terrain_origins[1, 1, 2] < first.terrain_origins[0, 1, 2] < 0
    assert first.to_xml_string(tmp_path / "assets") == cached.to_xml_string(tmp_path / "assets")


def test_cli_saves_default_world(tmp_path, monkeypatch, capsys):
    small = tg.TerrainGeneratorCfg(
        seed=1,
        size=(2, 2),
        slope_threshold=None,
        sub_terrains={"rough": tg.HfRandomUniformTerrainCfg()},
    )
    monkeypatch.setattr(tg, "ROUGH_TERRAINS_CFG", small)

    assert tg.main([str(tmp_path)]) == 0

    output = tmp_path / "rough_terrain.xml"
    assert output.is_file()
    assert str(output) in capsys.readouterr().out
    assert mujoco.MjModel.from_xml_path(str(output)).nhfield == 1
    spawn_file = tmp_path / "spawn_locations.yaml"
    assert spawn_file.is_file()
    spawns = yaml.safe_load(spawn_file.read_text())["spawn_locations"]
    assert len(spawns) == 1
    terrain_origin = tg.TerrainGenerator(small).terrain_origins.reshape(-1, 3)[0]
    assert spawns[0][2] == pytest.approx(terrain_origin[2] + small.spawn_height)
