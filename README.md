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

> [!IMPORTANT]
> Naming this package's reader by its full path — `type: sim-gazebo:world` — needs
> a PartCAD carrying [partcad/partcad#643](https://github.com/partcad/partcad/pull/643).
> Earlier releases look the package up from the root while the package declaring
> the object is still loading, find nothing, and record the object as broken. The
> `simulation:` entry works on any release.

## Four entry points, one format

Gazebo describes a simulation world in **SDFormat**, which PartCAD calls `world`
after the files it lives in. This package declares all four of the things a
PartCAD package can teach PartCAD, and all four are about that one format:

| Section | What it does | How it is asked for |
| --- | --- | --- |
| `import:` | read a `.world` file as a PartCAD scene | `type: sim-gazebo:world` |
| `export:` | write a PartCAD scene out as one | `pc export -S -t sim-gazebo:world` |
| `simulation:` | run one and say where everything ended up | `simulation: sim-gazebo:gazebo` |
| `open:` | open one in the Gazebo GUI | `pc ide open --with gazebo` |

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

PartCAD exports the scene as a world with every free body free to move and the
joints of every mechanism in it (see [Mechanisms](#mechanisms)), hands this
package the file, and this package runs the server until the *simulated* clock
reaches `duration`. It then reports where every model was at the start and at
the end:

```json
{
  "before": {"time": 0.0,  "bodies": {"top": {"pos": [0, 0, 30], "quat": [1, 0, 0, 0]}}, "joints": {}},
  "after":  {"time": 10.0, "bodies": {"top": {"pos": [29.7, 0, 9.8], "quat": [...]}}, "joints": {}},
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

Every run has a Gazebo transport **partition** of its own, and the server and
everything that reads from it are started in it. Gazebo names its topics after
the world, so two runs of one simulation at once — `pc test` running a
package's simulations side by side, say — are two servers on the same topics,
and in one partition each run would hear both: nothing fails, and a validation
is handed poses that were not its own. A `GZ_PARTITION` you have set (or
`IGN_PARTITION`, for the generation that reads that one) is kept as the prefix,
`<yours>:partcad-<random>`, rather than used as it is, which would put every run
back into one.

### Joints

A world whose models have joints in them — a hinge, a slide — also says where
each of them is, as `joints` beside `bodies`, in the vocabulary
[partcad-sim-mujoco](https://github.com/partcad/partcad-sim-mujoco) reports
them in: the terms an interface's `motion:` is written in, **degrees** for a
turn and **millimetres** for a move.

```json
"joints": {
  "swing": {"type": "continuous", "pos": 93.2, "vel": 358.6},
  "drop":  {"type": "prismatic", "pos": -896.4, "vel": -4188.9}
}
```

| `type` | in SDFormat | `pos` | `vel` |
| --- | --- | --- | --- |
| `revolute` | a `revolute` with a `<limit>` | degrees | deg/s |
| `continuous` | a `revolute` without one | degrees | deg/s |
| `prismatic` | a `prismatic` | mm | mm/s |
| `screw` | a `screw`, by its turn | degrees | deg/s |

* **`pos`** is the joint's own coordinate: zero where the world placed the
  child link. A pendulum written out horizontal reads 0 there and 90, one way
  or the other, hanging down.
* A turn nothing limits is **`continuous`**, whatever the world calls it — the
  line the MuJoCo plugin draws too, so a validation reads the same type out of
  either engine. Write one as a `revolute` with no `<limit>`: Harmonic's DART
  physics builds a `continuous` joint as a *fixed* one, and says so only in the
  server's own log.
* **No `effort`**, which the MuJoCo plugin reports. Gazebo's joint state
  publisher has a field for it and leaves it empty — it stays empty while a
  force is being applied to the joint — and a zero nobody measured would be
  read as a measurement.
* **No ball joints.** Gazebo publishes two of a ball joint's three coordinates,
  which is not an orientation.
* Not listed, because nothing there moves: a `fixed` joint, and every joint of
  a `<static>` model. Not read: joints of a model a world brings in with
  `<include>`, which are in another file — PartCAD's exporter writes everything
  into the one.
* Every joint that could have moved and is not reported is **named in
  `warnings`** with the reason: a ball joint, a `universal`, `revolute2` or
  `gearbox` one (PartCAD has no motion of those types), and one Gazebo ran with
  no freedom to move. A reading never quietly leaves out something that moved.
* A joint is reported under its own name, and as **`<model>::<joint>`** where
  another model has a joint of the same name — both of them, so that neither is
  the one the plain name means. A nested model is `outer::inner`.
* A world with no joints to read — a scene of parts — reports
  `"joints": {}`, so a validation can walk `after["joints"]` without asking
  first whether it is there.

```yaml
validation: after["joints"]["swing"]["pos"] > 80    # it swung down
```

Gazebo publishes a model's joint states only if the model carries its
`JointStatePublisher` system, and a world has no reason to. So when there is a
joint to read, the server runs a copy of the world with that system added to
each model that has one, on a topic of the run's own — written beside the
original, because a world names its meshes relative to where it is, and removed
when the run ends. A world with nothing to read runs exactly as it was handed
over. The states arrive every step and each reading takes the one of its own
simulated instant, so a reading's joints and its bodies are the same moment.

## Mechanisms

The world exporter writes the bodies PartCAD's body rule finds (`kinematics` in
PartCAD, the rule the MuJoCo plugin and PartCAD's URDF exporter write too), not
one body per node of geometry:

* every element of a scene is a **`<model>`**, and an assembly placed in one is
  one model, its root body the model's first link, `<name>_link`;
* everything inside an assembly is attached: parts welded together are **one
  `<link>`**, a `<visual>` and a `<collision>` each, and the link's
  `<inertial>` is their mass, centre of mass and inertia combined; a connection
  that keeps a degree of freedom is a **`<joint>`** to the link it moves
  against;
* an element marked `fixed: true` is welded to the world by a fixed joint to
  `world`.

Each joint is written as SDFormat's own type wherever SDFormat has one:

| What the joint's free steps are | SDFormat |
| --- | --- |
| one turn, limited | `revolute`, `<limit>` in radians |
| one turn, unlimited | `revolute` with no `<limit>` — **never `continuous`**, which Gazebo's DART builds as fixed |
| one move | `prismatic`, `<limit>` in metres |
| turn and move on one axis, coupled by a thread | `screw`, with its pitch |
| two turns through a point | `universal` (`<axis>`, `<axis2>`) |
| three turns through a point | `ball` |
| turn and move on one axis, uncoupled (cylindrical) | `revolute` + `prismatic` through a dummy link |
| two moves and the turn about their normal (planar) | two `prismatic` + `revolute` through dummy links |
| floating, inside a tree | three `prismatic` + `ball` through dummy links |
| anything else | one `revolute` or `prismatic` per freedom, in order, through dummy links |

* **Names.** A joint written as one SDFormat joint keeps PartCAD's name —
  `<child>-<target>`, or the connection's `joint:` — so
  `after["joints"]["shoulder_pan"]` reads it. One written as several is
  `<name>.<parameter>` for each (`nut-bolt.turnZ`, `float-tank.ball`), the name
  the MuJoCo plugin gives the MuJoCo joint of the same freedom. No two things in
  a world share a name, which is why a model's first link is not called after
  the model: Gazebo reports every pose by the bare name, and a link sharing its
  model's name was reported in the model's place.
* **Dummy links** weigh a gram, with an inertia of 10⁻⁶ kg·m², have no geometry,
  and are named after the joint (`<name>_link1`, ...). The export says which
  joints needed them.
* **Frames.** Each link is written where its joints put it at **zero**, in its
  model's frame; a joint's `<pose>` is the frame it turns or slides in, relative
  to its child link, and its `<axis><xyz expressed_in="...">` is in its parent
  link. So Gazebo's zero is PartCAD's zero, a limit is the declared one, and a
  joint reading is PartCAD's value.
* **Starting values are not written.** SDFormat 1.9 has no starting position
  (`<axis><initial_position>` went with 1.8, and Gazebo never read it), so a
  joint that `toParams` starts away from zero starts at zero here, and the
  export names each one. The MuJoCo plugin does start it there.
* **Physics.** `damping`, `friction`, `springStiffness` and `springReference`
  are `<axis><dynamics>`; `maxEffort` and `maxVelocity` are `<limit><effort>`
  and `<velocity>`. All in SI per radian or per metre; `springReference`, a
  position of the joint, converts from degrees or millimetres. They are written
  on every axis but one another axis moves (a screw's move). A ball has no
  axis, so its physics and the bounds of its turns are reported, not written;
  so are soft limits and anything else a joint states.
* **Screws.** `<thread_pitch>`, in radians a metre and positive for a
  left-handed thread, as Harmonic reads a 1.9 file; `<screw_thread_pitch>`, in
  metres a turn, from `version: "1.10"` on.
* **Mimics** are `<mimic>` from `version: "1.11"` on, and reported before it.
  Gazebo Harmonic reads one and does not enforce it: its DART has no mimic
  constraint, so a mimicking joint moves freely there either way.
* **`static`** (the export's default, for a file to look at) welds every body
  nothing holds: a model with nothing moving in it is `<static>`, and the root of
  a mechanism gets a fixed joint to `world`, so a pendulum exported on its own
  still swings. A run asks for `static: false`. **`flatten` is gone**, and
  reported when it is passed.
* **`self_collide`** (on for a run) lets the links of one model collide.
  Gazebo's DART never collides the two links of one joint, but the two ends of a
  chain through dummy links are not that, and with it on those can.

## Snapshots

PartCAD asks every run for two pictures of the world — one before anything
moved, one when the time is up — and says where to take them from: the scene's
own `render: png:` viewpoint (`viewport_origin`, `viewport_up`), the same one
`pc render` uses, or the corner a rendered part is drawn from when nothing says
otherwise. This package draws them, writes them into the run directory and
reports them beside the result:

```json
{"snapshots": {"before": "snapshot-before.png", "after": "snapshot-after.png"}}
```

They are what the PartCAD IDE's **Validation → Simulation** tab shows side by
side, and `pc sim` says where they were written.

They are the world file Gazebo ran, each visual placed where Gazebo's pose
messages say its model and its link were at the first reading and at the last,
rasterized on the CPU with numpy (`snapshot_raster.py`, the same file the MuJoCo
plugin draws with). Not Gazebo's own renderer: that needs a GPU or EGL, which a
sandbox does not have. Gazebo publishes every pose relative to its parent, and
names repeat — every visual of a hand-written world may be called `visual` — so
a pose is taken from the messages only where its name is unique, and from the
world file otherwise.

A picture that cannot be drawn is a warning, never a failed run: the verdict
does not depend on it.

## Where Gazebo comes from

Not from pip — there is no wheel that carries a Gazebo. That is the one way this
differs from
[partcad-sim-mujoco](https://github.com/partcad/partcad-sim-mujoco), where
`pip install mujoco` is the whole of it.

So it is found in this order:

* the `gz` (or `ign`) on `PATH`, if there is one;
* the one a ROS installation put under `/opt/ros/<distro>`, run with the
  environment ROS's setup script leaves — ROS keeps Gazebo off `PATH` until that
  script has run;
* and on a machine with neither, PartCAD's `docker` sandbox builds the run's
  environment from the image `dockerImage` in `partcad.yaml` names:
  `osrf/ros:jazzy-simulation`, OSRF's own image with Gazebo Harmonic in it,
  which is then the second case.

`PC_GZ` names an installation `PATH` does not know about. A machine with none of
these is told which way to arrange one, rather than shown a traceback.

The server and the topic reader are both subcommands of that one program —
`gz sim` and `gz topic` — and are run as such.

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
| `gravity` | the scene's | m/s², in the scene's frame. Overrides the scene's `gravity:` for this run, by writing it into the world before Gazebo loads it; unset, the run is under the scene's, or SDFormat's `[0, 0, -9.8]` when the scene states none. |
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

## So is what it weighs

The same material says what the part weighs. PartCAD works out every part's
mass, centre of mass and inertia -- what the part states, or else its solid at
the density of what it is made of -- and hands them over with the part, so a
link's `<inertial>` is exactly what `pc info` reports for it, and what the URDF
and MJCF exports of the same object say. A PTFE block weighs what PTFE weighs
rather than what aluminium does. The order is:

1. what the part states, as stated;
2. its `density` -- one it states, or else its material's;
3. the export's `density` parameter;
4. 2700 kg/m³, aluminium.

This exporter works none of that out. A link of several shapes -- two materials
in one link -- is added up by PartCAD's `mass_properties`, the one copy of that
arithmetic all three exporters share, so it balances where it should. A link
with no solid in it that states no mass is written without an `<inertial>`, and
the export says so.

It needs the PartCAD that does this, and says so: `partcad:` in `partcad.yaml`
makes an older one refuse the package.

## Gravity, and the fluid a scene is filled with

A PartCAD scene may say what its world is like, beside where things are in it:
its `gravity:` in m/s², and the `medium:` it is filled with, as a reference to a
material (`//pub/std/manufacturing/material/fluid:water`).

**Gravity** is the world's `<gravity>`. A scene that states none gets no
`<gravity>` element, so Gazebo applies SDFormat's own `[0, 0, -9.8]` exactly as
it always did. Which one a run uses is, in order: a `simulate:`'s own
`params: {gravity: ...}`, written into the world before Gazebo loads it; the
scene's; SDFormat's. An explicit `gravity` on the `world` export does the same
for a file written with `pc export`. Neither this package's export nor its
simulation states a default of its own, because one would beat every scene that
does. A run reports the gravity it was actually under, read out of the world it
ran.

**A fluid is not modelled**, and the export says so in a warning rather than
writing a world that pretends otherwise. SDFormat has no world-level fluid —
`<atmosphere>` is a temperature and a pressure for sensors — and Gazebo's answer,
the `gz-sim-buoyancy-system` with a `<uniform_fluid_density>`, is not the
straightforward fit it looks like:

* On **Gazebo Harmonic**, the release the image above carries, it measures a
  mesh's volume without its `<scale>`. Every mesh PartCAD writes is in
  millimetres under a scale of 0.001, so every link would be buoyed by a volume
  10⁹ times too large. It is fixed upstream in
  [gazebosim/gz-sim#3909](https://github.com/gazebosim/gz-sim/pull/3909), in no
  release this package can assume yet.
* A world that names **any** system of its own gets none of Gazebo's defaults —
  physics, user commands, and the scene broadcaster whose poses a run reads — so
  writing one means writing all of those as well, and keeping that list in step
  with Gazebo's.

Viscous drag has no world-level system at all. So a Gazebo run of a scene filled
with water is a run in a vacuum, and says so;
[partcad-sim-mujoco](https://github.com/partcad/partcad-sim-mujoco) is the
plugin that models a fluid today.

What that means, item by item, for a world written from a scene that names a
`medium:`:

| | Gazebo, through this package | MuJoCo, through partcad-sim-mujoco |
| --- | --- | --- |
| the scene's `gravity:` | modelled: the world's `<gravity>` | modelled |
| buoyancy (ρ · V · g) | **not modelled** | modelled |
| at the centre of buoyancy, so a body with its weight low down rights itself | **not modelled** | modelled, in a run |
| drag (density) and viscous resistance (viscosity) | **not modelled** | modelled |
| a free surface: floating at a waterline, partly submerged | **not modelled** | **not modelled** |
| added mass, lift | **not modelled** | **not modelled** |

PartCAD hands this exporter everything buoyancy needs all the same -- each
shape's measured `volume` and its `centerOfVolume`, the centroid of that volume,
beside its mass -- so that writing Gazebo's buoyancy system, once a Gazebo that
scales a mesh's volume can be assumed, needs nothing new from PartCAD. Until
then the two are counted as stated rather than reported as properties SDFormat
cannot hold: nothing about the part is lost by a world that has no fluid in it.

## Tests

```shell
pytest
```

`test_simulate_gazebo.py` needs nothing but the standard library and says why an
end-to-end run is not among what it checks. `test_snapshot.py` needs numpy, and
`partcad` for the half that reads a world: it places one by pose messages a real
`gz topic -e` printed. `test_world.py` needs `partcad`
installed: the reader and the writer are written against the sandbox contract
that lives there — `ocp_serialize`, `urdf_common`, `mass_properties` and
`primitive_shapes` — and the tests import them the same way a sandbox does. CI
installs it from PartCAD's `devel` branch, so that a change to that contract
fails here before it is released.

`test_joints.py` needs nothing but the standard library for everything but its
last few tests, which run a pendulum and a slider in a real Gazebo and check
their `joints` against physics with a known answer. Those are skipped where
there is no Gazebo — CI's runners have none — and run in the image the
simulation itself uses:

```shell
docker run --rm -v "$PWD:/w" -w /w -e PYTHONDONTWRITEBYTECODE=1 --entrypoint bash \
    ghcr.io/partcad/partcad-sim-gazebo:latest -c \
    'python3 -m venv /tmp/v && /tmp/v/bin/pip install -q pytest \
     && /tmp/v/bin/python -m pytest -v -p no:cacheprovider test_joints.py test_simulate_gazebo.py test_partition.py'
```

`test_export_joints.py` builds mechanisms with PartCAD's own ASSY code, a
stand-in for every part's geometry, and checks every row of the table above:
the joints written, under the names they are written under, and every link
where PartCAD's `kinematics` puts its body — read back by a forward kinematics
of SDFormat written from its specification, at zero and moved. Its last tests
need a Gazebo and `partcad` both: every row passes `gz sdf -k`, a bar released
horizontal on a hinge passes the bottom at √(2mgd/I) and swings with the period
4K(sin 45°)√(I/mgd) of a right-angle swing, and a nut on a screw moves 10 mm a
turn. In the image, `pip install -q pytest numpy partcad` and run it as above.

`test_partition.py` checks the environment each run starts its programs in, with
nothing but the standard library, and then runs two worlds of one name side by
side in a real Gazebo and checks that each reports only itself. That last test is
skipped without a Gazebo, like the ones above.

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
