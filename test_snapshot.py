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
import time
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


def test_the_floor_is_under_a_scene_far_from_the_origin_too(tmp_path):
    """The floor is centred where the line of sight through the middle of the
    picture meets it, not where that line crosses the plane through the origin
    -- which, for a scene a long way from the origin, is off the picture."""
    plane = {"origin": np.zeros(3), "axes": np.eye(3), "color": (0.8, 0.8, 0.8)}
    scene = {"solids": [cube((1000, 0, 1))], "planes": [plane]}

    snapshot_raster.take(str(tmp_path), dict(SNAPSHOT, files={"after": "a.png"}), {"after": scene})
    image = read_png(str(tmp_path / "a.png"))

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


def reference(camera, triangles, supersample=snapshot_raster.SUPERSAMPLE):
    """Which triangle is nearest at every pixel, worked out one pixel at a time: -1 for none.

    The plainest way there is of filling triangles, to hold the batched one to.
    """
    triangles = np.asarray(triangles, dtype=float)
    projected = camera.project(triangles)
    xs, ys = camera.to_pixels(projected, supersample)
    zs = projected[..., 2]
    height, width = camera.height * supersample, camera.width * supersample
    px, py = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    nearest = np.full((height, width), -np.inf)
    which = np.full((height, width), -1)
    for index, (x, y, z) in enumerate(zip(xs, ys, zs)):
        area = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0])
        w0 = ((x[1] - px) * (y[2] - py) - (x[2] - px) * (y[1] - py)) / area
        w1 = ((x[2] - px) * (y[0] - py) - (x[0] - px) * (y[2] - py)) / area
        w2 = 1.0 - w0 - w1
        depth = w0 * z[0] + w1 * z[1] + w2 * z[2]
        nearer = (w0 >= 0) & (w1 >= 0) & (w2 >= 0) & (depth > nearest)
        nearest[nearer] = depth[nearer]
        which[nearer] = index
    return which


def test_triangles_of_every_size_are_filled_as_one_at_a_time_would_fill_them():
    """The batched fill, the large-triangle fill, and the two together, against the plainest one."""
    rng = np.random.default_rng(7)
    camera = snapshot_raster.Camera((0, 0, 100), (0, 1, 0), 120, 90)
    camera.frame(np.array([[-60.0, -45.0, 0.0], [60.0, 45.0, 0.0]]))
    centres = rng.uniform(-60, 60, (300, 1, 3))
    # Mostly a few pixels across, some long and thin, a few larger than a batch.
    sizes = np.concatenate([rng.uniform(0.5, 4, 240), rng.uniform(10, 40, 50), rng.uniform(40, 90, 10)])
    triangles = centres + rng.normal(0, 1, (300, 3, 3)) * sizes[:, None, None]
    triangles[200:250, :, 1] *= 0.05

    one_at_a_time = snapshot_raster.Picture(camera)
    for index, triangle in enumerate(triangles):
        # One call each, as a thing of its own, so that the ids say which won.
        one_at_a_time.add(triangle[None], np.zeros((1, 3)), index)
    batched = snapshot_raster.Picture(camera)
    for start in range(0, len(triangles), 100):
        # A hundred in one call, which is what batches them; each one's index
        # is its colour, which says which won -- exactly, over a power of two.
        indices = np.arange(start, start + 100)
        colors = np.stack([indices / 512.0, np.zeros(100), np.zeros(100)], axis=1)
        batched.add(triangles[start : start + 100], colors, 1)

    expected = reference(camera, triangles)
    background = snapshot_raster._BACKGROUND_ID
    alone = np.where(one_at_a_time.ids == background, -1, one_at_a_time.ids)
    together = np.where(batched.ids == background, -1, np.round(batched.color[..., 0] * 512).astype(int))
    # A pixel centre exactly on an edge two triangles share can go either way.
    assert (alone != expected).mean() < 1e-3
    assert (together != expected).mean() < 1e-3


def test_a_closed_surface_is_turned_outward_and_an_open_one_is_left_as_it_is():
    for triangles in (
        snapshot_raster.box((1, 2, 3)),
        snapshot_raster.ellipsoid(1, 2, 3),
        snapshot_raster.cylinder(1, 2),
        snapshot_raster.capsule(1, 2),
        snapshot_raster.capsule(1, 2, rings=7),
    ):
        for given in (triangles, triangles[:, ::-1]):
            outward, closed = snapshot_raster.outward(given)
            assert closed
            volume = np.einsum("ij,ij->i", outward[:, 0], np.cross(outward[:, 1], outward[:, 2])).sum()
            assert volume > 0

    open_box = snapshot_raster.box((1, 1, 1))[:-1]
    assert snapshot_raster.outward(open_box)[1] is False
    mixed = snapshot_raster.box((1, 1, 1)).copy()
    mixed[0] = mixed[0][::-1]
    assert snapshot_raster.outward(mixed)[1] is False


def test_the_back_of_a_closed_surface_is_not_drawn_and_nothing_shows_it():
    camera = snapshot_raster.Camera((100, -100, 100), (0, 0, 1), 64, 48)
    triangles, closed = snapshot_raster.outward(snapshot_raster.ellipsoid(10, 6, 4))
    assert closed
    camera.frame(triangles)
    solid = {"triangles": triangles, "color": (0.8, 0.4, 0.2), "id": 1}

    every = snapshot_raster.draw(camera, {"solids": [solid]})
    culled = snapshot_raster.draw(camera, {"solids": [dict(solid, closed=True)]})

    assert (every == culled).all()


def test_every_frame_of_a_video_is_the_picture_drawn_afresh():
    """What 'Frames' draws again of a frame, and copies of the floor, adds up to the whole picture."""
    plane = {"origin": np.zeros(3), "axes": np.eye(3), "color": (0.8, 0.8, 0.8)}
    scenes = [
        {"planes": [plane], "solids": [cube((step * 3.0, 0, 10 - step)), cube((0, 5, 1), ident=2)]} for step in range(8)
    ]
    camera = snapshot_raster.Camera((100, -100, 100), (0, 0, 1), 64, 48)
    camera.frame(np.concatenate([snapshot_raster._solid_points(scene) for scene in scenes]))
    frames = snapshot_raster.Frames(camera)

    for scene in scenes:
        assert (frames.draw(scene) == snapshot_raster.draw(camera, scene)).all()


def test_a_video_has_a_frame_for_every_frame_time_of_the_run():
    assert snapshot_raster.frame_times(10.0, 25) == [index / 25.0 for index in range(251)]
    assert len(snapshot_raster.frame_times(0.1, 25)) == 3
    assert snapshot_raster.frame_times(0.0, 25) == [0.0]
    assert snapshot_raster.frame_times(1.0, 0) == []


VIDEO = {
    "format": "webm",
    "file": "video.webm",
    "fps": 25,
    "width": 64,
    "height": 48,
    "viewport_origin": [100.0, -100.0, 100.0],
    "viewport_up": [0.0, 0.0, 1.0],
}


def falling_cube(index):
    plane = {"origin": np.zeros(3), "axes": np.eye(3), "color": (0.8, 0.8, 0.8)}
    return {"planes": [plane], "solids": [cube((0, 0, 10 - index))]}


def test_a_video_is_recorded_as_webm(tmp_path):
    pytest.importorskip("imageio_ffmpeg")

    written = snapshot_raster.record(str(tmp_path), VIDEO, 10, falling_cube)

    assert written == "video.webm"
    with open(tmp_path / "video.webm", "rb") as f:
        # The EBML header every Matroska file, and so every WebM, starts with.
        assert f.read(4) == b"\x1a\x45\xdf\xa3"


def test_only_webm_is_recorded(tmp_path):
    with pytest.raises(ValueError, match="webm"):
        snapshot_raster.record(str(tmp_path), dict(VIDEO, format="mp4"), 10, falling_cube)


def test_a_video_that_would_not_be_finished_in_time_is_given_up_and_leaves_no_file(tmp_path):
    pytest.importorskip("imageio_ffmpeg")

    with pytest.raises(snapshot_raster.OutOfTime):
        snapshot_raster.record(str(tmp_path), VIDEO, 10, falling_cube, deadline=time.monotonic() - 1)

    assert not (tmp_path / "video.webm").exists()


def test_a_video_ffmpeg_cannot_write_says_what_ffmpeg_said(tmp_path, monkeypatch):
    pytest.importorskip("imageio_ffmpeg")
    monkeypatch.setitem(snapshot_raster.VIDEO_FORMATS, "webm", {"codec": "no-such-codec", "parameters": []})

    with pytest.raises(RuntimeError, match="FFmpeg"):
        snapshot_raster.record(str(tmp_path), VIDEO, 10, falling_cube)

    assert not (tmp_path / "video.webm").exists()


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


def message_at(seconds, top):
    """A pose message of the world above, at 'seconds' of simulated time, with the top block at 'top' (x, z)."""
    whole = int(seconds)
    stamp = "header {\n  stamp {\n    sec: %d\n    nsec: %d\n  }\n}\n" % (whole, round((seconds - whole) * 1e9))
    return stamp + BEFORE.split("}\n}\n", 1)[1].replace(pose("top", 0.018, 0, 0.03), pose("top", top[0], 0, top[1]))


def test_the_film_keeps_a_frame_for_every_frame_time_from_the_message_of_its_instant():
    import simulate_gazebo

    film = simulate_gazebo.Film(VIDEO, 0.2)
    # Sixty messages a second, and one gap of a tenth of a second in them.
    for moment in [0.0, 1 / 60, 2 / 60, 3 / 60, 0.15, 0.2]:
        message = simulate_gazebo.parse_message(message_at(moment, (0.018 + moment, 0.03)))
        film.watch(moment, message)

    assert len(film.poses) == len(snapshot_raster.frame_times(0.2, 25)) == 6
    xs = [poses["top"][1][0] for poses in film.poses]
    # Each frame is the first message at or past its instant (0, 0.04, ...),
    # and the message after the gap is every frame the gap held.
    assert xs == pytest.approx([18.0, 68.0, 168.0, 168.0, 218.0, 218.0])


def test_a_run_records_its_video_out_of_the_world_and_the_messages(world):
    pytest.importorskip("imageio_ffmpeg")
    import simulate_gazebo

    film = simulate_gazebo.Film(VIDEO, 0.4)
    for step in range(25):
        moment = step / 60
        message = simulate_gazebo.parse_message(message_at(moment, (0.018 + moment / 10, 0.03 - moment / 20)))
        film.watch(moment, message)
    warnings = []

    recorded = film.record(str(world), str(world / "world.sdf"), None, warnings)

    assert recorded == "video.webm", warnings
    assert warnings == []
    assert (world / "video.webm").stat().st_size > 0


def test_a_video_that_cannot_be_recorded_is_a_warning(world):
    import simulate_gazebo

    film = simulate_gazebo.Film(VIDEO, 0.1)
    film.watch(0.0, simulate_gazebo.parse_message(BEFORE))
    warnings = []

    assert film.record(str(world), str(world / "missing.sdf"), None, warnings) is None
    assert any("video could not be recorded" in warning for warning in warnings)
