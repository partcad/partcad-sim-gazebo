#!/usr/bin/env bash
#
# This package's tests, run in the image the simulation runs in -- the only
# place their real-Gazebo half runs: every test that starts a Gazebo is skipped
# where there is none, and a CI runner has none.
#
# One script, run by the `gazebo` job of `test.yml` against the published
# image, and by `image.yml` against the image a change to `container/` has just
# built, so that the image and the code are each tested against the other's
# current state with one recipe. Run it by hand the same way, from the root of
# a checkout:
#
#     container/test.sh ghcr.io/partcad/partcad-sim-gazebo:latest
#     container/test.sh ghcr.io/partcad/partcad-sim-gazebo:latest test_joints.py -k pendulum
#
# The environment is the one the `pytest` job of `test.yml` installs, with
# PartCAD from its `devel` branch for the reason that file's header gives. The
# checkout is mounted read-only, so that nothing a run writes outlives it.
#
# It fails if any test was skipped for want of a Gazebo: an image whose Gazebo
# this plugin cannot find would otherwise skip the very tests it is run for, and
# pass.
#
# Usage: container/test.sh <image> [pytest arguments]
set -euo pipefail
image="$1"
shift
docker run --rm -v "$PWD:/w:ro" -w /w -e PYTHONDONTWRITEBYTECODE=1 --entrypoint bash "$image" -c '
  set -euo pipefail
  python3 -m venv /tmp/v
  /tmp/v/bin/pip install -q --upgrade pip
  /tmp/v/bin/pip install -q pytest pyyaml numpy "imageio-ffmpeg>=0.5,<1" \
    "partcad @ git+https://github.com/partcad/partcad@devel"
  # A stack of every thread of a test still running after five minutes, which
  # none of these comes near: a run that hangs says where.
  status=0
  /tmp/v/bin/python -m pytest -v -rs -p no:cacheprovider -o faulthandler_timeout=300 "$@" \
    | tee /tmp/pytest.log || status=$?
  if grep -q "no Gazebo here" /tmp/pytest.log; then
    echo "::error::tests that need a Gazebo were skipped in $0: this image has none this plugin can find"
    exit 1
  fi
  exit "$status"
' "$image" "$@"
