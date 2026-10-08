#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""The pictures a simulation takes of its scene (see 'snapshot_raster.py').

The renderer is tested on its own, with nothing but numpy -- the same tests
the MuJoCo plugin runs over the same file -- and then what this plugin adds to
it: reading what a world looks like out of its SDFormat, and placing it where
Gazebo's pose messages say it was. Gazebo itself is not needed for either; the
messages are the ones a real 'gz topic -e' printed.
"""

import os
import struct
import sys
import zlib

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import snapshot_raster  # noqa: E402

SNAPSHOT = {
    "format": "png",
    "files": {"before": "snapshot-before.png", "after": "snapshot-after.png"},
    "width": 64,
    "height": 48,
    "viewport_origin": [100.0, -100.0, 100.0],
    "viewport_up": [0.0, 0.0, 1.0],
}


def read_png(path):
    """The pixels of an RGB PNG this module wrote, as (height, width, 3)."""
    with open(path, "rb") as f:
        data = f.read()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    width, height = struct.unpack(">II", data[16:24])
    start = data.index(b"IDAT") + 4
    length = struct.unpack(">I", data[start - 8 : start - 4])[0]
    raw = zlib.decompress(data[start : start + length])
    rows = np.frombuffer(raw, dtype=np.uint8).reshape(height, 1 + width * 3)
    assert not rows[:, 0].any(), "every row is written unfiltered"
    return rows[:, 1:].reshape(height, width, 3)


def cube(center, half=1.0, color=(0.8, 0.2, 0.2), ident=1):
    return {"triangles": snapshot_raster.box([half] * 3) + np.asarray(center), "color": color, "id": ident}


BACKGROUND = np.array([int(c * 255 + 0.5) for c in snapshot_raster.BACKGROUND], dtype=np.uint8)


def is_background(pixel):
    return (np.asarray(pixel) == BACKGROUND).all()


def drawn_rows(image):
    """The rows of a picture anything is drawn on."""
    return np.where((image != BACKGROUND).any(axis=2).any(axis=1))[0]


def test_the_front_view_has_x_to_the_right_and_z_up():
    """The same convention 'pc render --view front' draws in."""
    camera = snapshot_raster.Camera((0, -100, 0), (0, 0, 1), 10, 10)
    right, up, toward = camera.axes
    assert np.allclose(right, (1, 0, 0))
    assert np.allclose(up, (0, 0, 1))
    assert np.allclose(toward, (0, -1, 0))


def test_a_solid_is_drawn_in_the_middle_of_the_picture_and_nothing_is_drawn_around_it():
    camera = snapshot_raster.Camera((0, -100, 0), (0, 0, 1), 40, 40)
    scene = {"solids": [cube((5, 5, 5))]}
    camera.frame(scene["solids"][0]["triangles"])

    image = snapshot_raster.draw(camera, scene)

    assert image.shape == (40, 40, 3)
    assert not is_background(image[20, 20])
    assert is_background(image[1, 1])


def test_the_nearer_face_hides_the_farther_one():
    camera = snapshot_raster.Camera((0, -100, 0), (0, 0, 1), 40, 40)
    near = cube((0, -5, 0), color=(0.0, 0.0, 1.0), ident=1)
    far = cube((0, 5, 0), color=(1.0, 0.0, 0.0), ident=2)
    camera.frame(np.concatenate([near["triangles"], far["triangles"]]))

    image = snapshot_raster.draw(camera, {"solids": [far, near]})

    red, green, blue = image[20, 20]
    assert blue > red


def test_a_picture_is_written_as_a_png_that_reads_back_as_itself(tmp_path):
    image = (np.arange(4 * 3 * 3).reshape(4, 3, 3) * 7 % 256).astype(np.uint8)
    path = tmp_path / "x.png"

    snapshot_raster.write_png(str(path), image)

    assert (read_png(str(path)) == image).all()


def test_both_pictures_are_framed_alike_so_what_moved_is_seen_to_have_moved(tmp_path):
    """A block that fell is lower in the frame, not re-centred."""
    before = {"solids": [cube((0, 0, 10))]}
    after = {"solids": [cube((0, 0, 0))]}
    snapshot = dict(SNAPSHOT, viewport_origin=[0, -100, 0])

    written = snapshot_raster.take(str(tmp_path), snapshot, {"before": before, "after": after})

    assert written == SNAPSHOT["files"]
    rows_before = drawn_rows(read_png(str(tmp_path / "snapshot-before.png")))
    rows_after = drawn_rows(read_png(str(tmp_path / "snapshot-after.png")))
    # Picture rows grow downwards: the block in the second is further down.
    assert rows_after.mean() > rows_before.mean()


def test_the_floor_is_drawn_under_the_scene_and_does_not_decide_the_framing(tmp_path):
    plane = {"origin": np.zeros(3), "axes": np.eye(3), "color": (0.8, 0.8, 0.8)}
    scene = {"solids": [cube((0, 0, 1))], "planes": [plane]}

    snapshot_raster.take(str(tmp_path), dict(SNAPSHOT, files={"after": "a.png"}), {"after": scene})
    image = read_png(str(tmp_path / "a.png"))

    # The corners are floor rather than background: it reaches the edges.
    assert not is_background(image[0, 0])
    assert not is_background(image[-1, -1])


def test_only_png_is_drawn(tmp_path):
    with pytest.raises(ValueError, match="PNG"):
        snapshot_raster.take(str(tmp_path), dict(SNAPSHOT, format="jpeg"), {"before": {"solids": []}})


def test_every_primitive_is_a_closed_set_of_triangles():
    for triangles in (
        snapshot_raster.box((1, 2, 3)),
        snapshot_raster.ellipsoid(1, 2, 3),
        snapshot_raster.cylinder(1, 2),
        snapshot_raster.capsule(1, 2),
    ):
        assert triangles.ndim == 3 and triangles.shape[1:] == (3, 3)
        assert np.isfinite(triangles).all()


def test_an_stl_is_read_whether_it_is_binary_or_text(tmp_path):
    here = os.path.join(os.path.dirname(__file__), "tests", "data", "cube.stl")
    triangles = snapshot_raster.read_stl(here)
    assert triangles.shape[1:] == (3, 3) and len(triangles) >= 12

    text = tmp_path / "t.stl"
    text.write_text(
        "solid t\n facet normal 0 0 1\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n"
        "   vertex 0 1 0\n  endloop\n endfacet\nendsolid t\n",
        encoding="utf-8",
    )
    assert np.allclose(snapshot_raster.read_stl(str(text)), [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])


#
# A world, placed where Gazebo said it was
#

try:
    import partcad as pc
except ImportError:  # pragma: no cover - CI installs it
    pc = None

if pc is not None:
    sys.path.append(os.path.join(os.path.dirname(pc.__file__), "wrappers"))

WORLD = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="w">
    <model name="ground_plane"><static>true</static>
      <link name="link"><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane>
      </geometry></visual></link></model>
    <model name="bottom"><pose>0 0 0.01 0 0 0</pose>
      <link name="bottom_link"><visual name="visual"><geometry><box><size>0.02 0.02 0.02</size></box></geometry>
      <material><diffuse>0.2 0.4 0.8 1</diffuse></material></visual></link></model>
    <model name="top"><pose>0.018 0 0.03 0 0 0</pose>
      <link name="top_link"><visual name="visual"><geometry>
        <mesh><uri>cube.stl</uri><scale>0.001 0.001 0.001</scale></mesh></geometry></visual></link></model>
  </world>
</sdf>
"""


def pose(name, x, y, z, w=1.0, qx=0.0, qy=0.0, qz=0.0):
    return (
        'pose {\n  name: "%s"\n  position {\n    x: %r\n    y: %r\n    z: %r\n  }\n  orientation {\n    x: %r\n    y: %r\n    z: %r\n    w: %r\n  }\n}\n'
        % (name, x, y, z, qx, qy, qz, w)
    )


# What 'gz topic -e -t /world/w/pose/info' printed for a world like this one:
# every model relative to the world, every link relative to its model, every
# visual relative to its link -- and the visuals all called the same.
BEFORE = (
    "header {\n  stamp {\n    nsec: 1000000\n  }\n}\n"
    + pose("ground_plane", 0, 0, 0)
    + pose("bottom", 0, 0, 0.01)
    + pose("top", 0.018, 0, 0.03)
    + pose("link", 0, 0, 0)
    + pose("bottom_link", 0, 0, 0)
    + pose("top_link", 0, 0, 0)
    + pose("visual", 0, 0, 0)
    + pose("visual", 0, 0, 0)
)
AFTER = BEFORE.replace(pose("top", 0.018, 0, 0.03), pose("top", 0.0298, 0, 0.01, 0.7071068, 0, 0.7071068, 0))


@pytest.fixture
def world(tmp_path):
    if pc is None:
        pytest.skip("partcad is not installed: 'gazebo_common' reads poses with its 'urdf_common'")
    import shutil

    shutil.copy(os.path.join(os.path.dirname(__file__), "tests", "data", "cube.stl"), tmp_path / "cube.stl")
    (tmp_path / "world.sdf").write_text(WORLD, encoding="utf-8")
    return tmp_path


def test_a_name_two_entities_share_answers_for_neither():
    import simulate_gazebo

    poses = simulate_gazebo.unique_poses(simulate_gazebo.parse_message(BEFORE))

    assert "visual" not in poses
    assert poses["top"][1] == pytest.approx((18.0, 0.0, 30.0))


def test_the_world_is_read_as_the_tree_its_poses_are_stated_along(world):
    import simulate_gazebo

    warnings = []
    layout = simulate_gazebo.world_layout(str(world / "world.sdf"), warnings)

    assert [model["name"] for model in layout] == ["ground_plane", "bottom", "top"]
    assert "plane" in layout[0]["links"][0]["visuals"][0]
    assert layout[1]["links"][0]["visuals"][0]["color"] == [0.2, 0.4, 0.8]
    # The mesh is millimetres, scaled by the 0.001 that says so: what is read
    # back is millimetres again, like everything else here.
    mesh = layout[2]["links"][0]["visuals"][0]["triangles"]
    assert np.ptp(mesh.reshape(-1, 3), axis=0).max() > 1.0
    assert warnings == []


def test_each_picture_places_the_world_where_that_reading_says_it_was(world):
    import simulate_gazebo

    layout = simulate_gazebo.world_layout(str(world / "world.sdf"), [])
    before = simulate_gazebo.arrange(layout, simulate_gazebo.unique_poses(simulate_gazebo.parse_message(BEFORE)))
    after = simulate_gazebo.arrange(layout, simulate_gazebo.unique_poses(simulate_gazebo.parse_message(AFTER)))

    def lowest(scene, index):
        return scene["solids"][index]["triangles"][..., 2].min()

    assert len(before["planes"]) == 1 and len(before["solids"]) == 2
    # The bottom block did not move; the top one ended up twenty millimetres
    # lower, where the reading says it went.
    assert lowest(after, 0) == pytest.approx(lowest(before, 0))
    assert lowest(after, 1) < lowest(before, 1) - 10.0


def test_a_run_draws_both_pictures_and_says_so(world):
    import simulate_gazebo

    warnings = []
    drawn = simulate_gazebo.take_snapshots(
        str(world),
        SNAPSHOT,
        str(world / "world.sdf"),
        simulate_gazebo.parse_message(BEFORE),
        simulate_gazebo.parse_message(AFTER),
        warnings,
    )

    assert drawn == SNAPSHOT["files"]
    assert warnings == []
    assert (read_png(str(world / "snapshot-before.png")) != read_png(str(world / "snapshot-after.png"))).any()


def test_a_picture_that_cannot_be_drawn_is_a_warning(world):
    import simulate_gazebo

    warnings = []
    drawn = simulate_gazebo.take_snapshots(
        str(world), SNAPSHOT, str(world / "missing.sdf"), {"pose": []}, {"pose": []}, warnings
    )

    assert drawn == {}
    assert any("snapshots could not be drawn" in warning for warning in warnings)
