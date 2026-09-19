from pathlib import Path

import pytest

from sdf_reference_converter import generate, prepare_mesh_asset, resolve_model_uri


def test_separates_collision_visual_and_render_only_model(tmp_path: Path) -> None:
    resources = tmp_path / "resources"
    (resources / "terrain").mkdir(parents=True)
    (resources / "terrain" / "world.dae").write_text(
        "<COLLADA xmlns='http://www.collada.org/2005/11/COLLADASchema'/>", encoding="utf-8")
    (resources / "water").mkdir()
    (resources / "water" / "water.dae").write_text(
        "<COLLADA xmlns='http://www.collada.org/2005/11/COLLADASchema'/>", encoding="utf-8")
    world = tmp_path / "world.sdf"
    world.write_text("""<sdf version='1.9'><world name='test'>
      <model name='terrain'><static>true</static><pose>1 2 3 0 0 1.57079632679</pose><link name='l'>
        <collision name='c'><geometry><mesh><uri>model://terrain/world.dae</uri><scale>2 3 4</scale></mesh></geometry></collision>
        <visual name='v'><geometry><mesh><uri>model://terrain/world.dae</uri><scale>2 3 4</scale></mesh></geometry></visual>
      </link></model>
      <model name='water'><link name='l'><visual name='v'><geometry><mesh><uri>model://water/water.dae</uri></mesh></geometry></visual></link></model>
    </world></sdf>""", encoding="utf-8")
    output = tmp_path / "generated"
    result = generate(world, [resources], output, "Assets/Generated/Test", "test-v1",
                      {"project": "example/test", "version": "abc"})

    assert len(result["objects"]) == 2
    terrain, water = result["objects"]
    assert terrain["unityPose"]["position"] == [1.0, 3.0, 2.0]
    assert terrain["collisions"][0]["unityScale"] == [2.0, 4.0, 3.0]
    assert terrain["collisions"][0]["asset"]["sourceSha256"] == \
        terrain["visuals"][0]["asset"]["sourceSha256"]
    assert water["semanticRole"] == "render-only-environment"
    assert not water["collisions"]
    assert (output / "manifest.json").is_file()


def test_rejects_unresolved_or_nonlocal_uri(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        resolve_model_uri("https://fuel.gazebosim.org/model", [tmp_path])
    with pytest.raises(FileNotFoundError):
        resolve_model_uri("model://missing/model.dae", [tmp_path])


def test_converts_ascii_stl_to_deterministic_obj(tmp_path: Path) -> None:
    source = tmp_path / "source.stl"
    source.write_text("""solid test
facet normal 0 0 1
 outer loop
  vertex 0 0 0
  vertex 1 0 0
  vertex 0 1 0
 endloop
endfacet
endsolid test
""", encoding="ascii")
    output = tmp_path / "generated"
    record = prepare_mesh_asset(source, "model/source.stl", output,
                                "Assets/Generated/Test", {})
    converted = output / "source/model/source.obj"
    assert record["conversion"] == "deterministic-stl-to-obj"
    assert record["unityAssetPath"].endswith("model/source.obj")
    assert converted.read_text(encoding="ascii").endswith("f 1 2 3\n")
    assert len(record["derivedSha256"]) == 64
