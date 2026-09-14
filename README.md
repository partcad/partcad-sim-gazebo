# partcad-sim-gazebo

A [PartCAD](https://partcad.org/) package that is **everything PartCAD knows
about [Gazebo](https://gazebosim.org/)**: reading a world, writing one, opening
one, and running one.

```yaml
dependencies:
  sim-gazebo:
    type: git
    url: https://github.com/partcad/partcad-sim-gazebo.git

scenes:
  warehouse:
    # A world somebody else wrote, used directly as a PartCAD scene.
    type: sim-gazebo:world
    path: warehouse.world

assemblies:
  stack:
    type: assy
    simulate:
      stands:
        simulation: sim-gazebo:gazebo
        offset: [[0, 0, 10], [0, 0, 1], 0]
        validation: |
          max(
              abs(after["bodies"][name]["pos"][2] - before["bodies"][name]["pos"][2])
              for name in before["bodies"]
          ) < 2.0
```

```console
$ pc sim -a stack
INFO: //your/package:stack: the simulation 'stands' validated
```

## Four entry points, one format

Gazebo describes a simulation world in **SDFormat**, which PartCAD calls `world`
after the files it lives in. This package declares all four of the things a
PartCAD package can teach PartCAD, and all four are about that one format:

| Section | What it does | How it is asked for |
| --- | --- | --- |
| `import:` | read a `.world` file as a PartCAD scene | `type: sim-gazebo:world` |
| `export:` | write a PartCAD scene out as one | `pc export -S -t sim-gazebo:world` |
| `simulation:` | run one and say where everything ended up | `simulation: sim-gazebo:gazebo` |
| `open:` | open one in the Gazebo GUI | `pc open --with gazebo` |

They are declared together because they are one piece of knowledge. A reader and
a writer of the same format disagree the moment they are maintained apart, and
the simulator is what decides what the file has to say in the first place.

So a world somebody else wrote is a scene you can place parts in, and a scene
you built out of parts is a world Gazebo can run — and `pc convert scene` moves
a package between the two:

```shell
pc convert scene -t assy warehouse              # take it over, as parts of your own
pc convert scene -t sim-gazebo:world bench      # and write it back out
```

## What a run reports

PartCAD exports the scene as a world with every model free to move, hands this
package the file, and this package runs the server until the *simulated* clock
reaches `duration`. It then reports where every model was at the start and at
the end:

```json
{
  "before": {"time": 0.0,  "bodies": {"top": {"pos": [0, 0, 30], "quat": [1, 0, 0, 0]}}},
  "after":  {"time": 10.0, "bodies": {"top": {"pos": [29.7, 0, 9.8], "quat": [...]}}},
  "simulator": "gazebo", "world": "stack", "duration": 10.0, "units": "mm"
}
```

Positions are in **millimetres**, PartCAD's unit everywhere, not the metres
SDFormat works in: a `validation:` expression is written by whoever wrote the
part, against the numbers that part is drawn in.

PartCAD reads nothing inside `before` and `after`. It hands them to the
`validation:` expression the package wrote and reports what that says — every
judgement in that sentence belongs to the package being simulated.

The clock is read out of the messages rather than off the wall, which matters:
`gz sim -r` runs at a real-time factor of about one, and "about" is not something
a validation should depend on. Wall-clock time is only the `timeout` that stops a
run which is not progressing at all.

## Where Gazebo comes from

Not from pip — there is no wheel that carries a Gazebo. That is the one way this
differs from
[partcad-sim-mujoco](https://github.com/partcad/partcad-sim-mujoco), where
`pip install mujoco` is the whole of it.

So it is found the same two ways `pc open --with gazebo` finds it:

* the `gz` (or `ign`, or `gazebo`) on this machine, if there is one;
* otherwise the official image, which `dockerImage` in `partcad.yaml` names and
  PartCAD's `docker` sandbox is built from.

`PC_GZ` names an installation `PATH` does not know about. A machine with neither
is told which of the two to arrange, rather than shown a traceback.

Reading and writing a world are not like that at all. They are XML and
arithmetic, no Gazebo is involved, and they run in an ordinary PartCAD sandbox
alongside OpenCASCADE — which is there for the geometry at the ends of it: the
primitives a world names instead of a mesh, and triangulating what the writer
writes.

## Parameters

Set any of these as fields of the simulation (per package), or in the `params:`
of one `simulate:` entry (per simulation).

| Parameter | Default | What it is |
| --- | --- | --- |
| `duration` | `10.0` | Seconds of simulated time to run for. |
| `timestep` | Gazebo's | Integration step, in seconds. |
| `gravity` | `[0, 0, -9.81]` | m/s², in the scene's frame. |
| `timeout` | `300.0` | Wall-clock seconds to wait for the server before giving up. |
| `samples` | `0` | Report the state at this many evenly spaced instants too, as `samples`. |

The reader and the writer take parameters of their own; `partcad.yaml` lists
each one beside what it does.

## Friction is a fact about the material

Whether a stack of blocks stands up is not a property of its geometry. A part
that declares a material whose `mu` is set gets that coefficient written into
the world — SDFormat spells it `<surface><friction><ode><mu>` — and the
simulation answers for the material the part is actually made of. A part that
states neither gets the simulator's own default, which is a number nobody chose.

PartCAD writes each body's own coefficient and says nothing about how the two
sides of a contact combine: that is Gazebo's model rather than the part's.

## Tests

```shell
pytest
```

`test_simulate_gazebo.py` needs nothing but the standard library and says why an
end-to-end run is not among what it checks. `test_world.py` needs `partcad`
installed: the reader and the writer are written against the sandbox contract
that lives there — `ocp_serialize`, `urdf_common` and `primitive_shapes` — and
the tests import them the same way a sandbox does.

## Where this came from

The reader, the writer and the `open:` entry shipped inside the `partcad` wheel
until 0.8.80. They should not, for the reason the simulator never did: Gazebo is
somebody's program with a release cycle of its own, and SDFormat is Gazebo's.
Nothing about either belongs in a CAD tool's wheel.

`urdf` is the counter-example and stays in PartCAD's `//builtin/import`: a URDF
describes a robot rather than any one engine's world, and ROS, Gazebo, MuJoCo,
PyBullet and Isaac all read it.

Once this package is listed in the [public
index](https://github.com/partcad/partcad-index) it will also be reachable as
`//pub/feature/simulate/gazebo` — which is the name it gives itself — without
the `dependencies:` entry above.

## License

Apache License 2.0. See [LICENSE](./LICENSE).
