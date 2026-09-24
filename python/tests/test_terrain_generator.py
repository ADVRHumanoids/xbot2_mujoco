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


def test_saved_world_is_relocatable(tmp_path):
    output_dir = tmp_path / "generated"
    world = tg.TerrainGenerator(
        tg.TerrainGeneratorCfg(
            seed=1,
            size=(2, 2),
            slope_threshold=None,
            sub_terrains={"rough": tg.HfRandomUniformTerrainCfg()},
        )
    ).save(output_dir / "terrain.xml")

    relocated = tmp_path / "relocated"
    output_dir.rename(relocated)
    model = mujoco.MjModel.from_xml_path(str(relocated / world.name))

    assert model.nhfield == 1


def test_merge_with_mjcf_generator(tmp_path):
    g = tg.TerrainGenerator(
        tg.TerrainGeneratorCfg(
            seed=1,
            size=(2, 2),
            horizontal_scale=0.2,
            slope_threshold=None,
            sub_terrains={"rough": tg.HfRandomUniformTerrainCfg()},
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
