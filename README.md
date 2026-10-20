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
reaches `duration`. It then reports where every body was at the start and at
the end, under PartCAD's names and in the world's frame (see **Names** under
[Mechanisms](#mechanisms)):

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
Gazebo runs at a real-time factor of about one, and "about" is not something
a validation should depend on. Wall-clock time is only the `timeout` that stops a
run which is not progressing at all (see "When a run goes wrong").

**`before` is the world as it was loaded, at `time` 0.** The server is started
paused, the first reading is taken while it is — every joint at its start,
every body where the scene put it, every speed nought, since a world starts at
rest — and only then is the world set running, through its own control service
(`/world/<name>/control`). A pendulum started at 90° reads exactly 90 in
`before`. Every reading's `time` is the simulated instant its pose message was
taken at, read out of the message; `after` is the first one at or past
`duration`.

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
each of them is, as `joints` beside `bodies`: **one entry per PartCAD joint**,
under PartCAD's name for it, in the vocabulary
[partcad-sim-mujoco](https://github.com/partcad/partcad-sim-mujoco) reports
them in — the terms an interface's `motion:` is written in, **degrees** for a
turn and **millimetres** for a move, counted from **PartCAD's zero**. So one
validation reads both engines alike.

```json
"joints": {
  "swing":  {"type": "revolute", "pos": 93.2, "vel": 358.6},
  "drop":   {"type": "prismatic", "pos": -896.4, "vel": -4188.9},
  "nut":    {"type": "screw", "pos": {"turnZ": 95.7, "moveZ": 0.16}, "vel": {"turnZ": 114.6, "moveZ": 3.18}},
  "hooke":  {"type": "universal", "pos": {"turnZ": 15.7, "turnY": 8.5}, "vel": {"turnZ": 57.3, "turnY": -28.6}},
  "socket": {"type": "ball", "pos": {"turnX": 10.3, "turnY": 20.2, "turnZ": 30.5},
             "quat": [0.943, 0.129, 0.145, 0.271], "vel": [32.8, 8.4, 64.0]},
  "loose":  {"type": "floating", "pos": {"moveZ": 149.1, "moveY": 0.0, "moveX": 0.0,
                                          "turnY": 0.0, "turnX": 0.0, "turnZ": 0.0},
             "quat": [1, 0, 0, 0],
             "vel": {"moveZ": 1520.6, "moveY": 0.0, "moveX": 0.0, "turnY": 0.0, "turnX": 0.0, "turnZ": 0.0,
                     "angular": [0, 0, 0]}}
}
```

| `type` | in SDFormat | `pos` | `vel` |
| --- | --- | --- | --- |
| `revolute` | a `revolute`, bounded or not | degrees | deg/s |
| `prismatic` | a `prismatic` | mm | mm/s |
| `screw` | a `screw` | `{turn: degrees, move: mm}`, the move following the turn by the thread | the same, per second |
| `universal` | a `universal` | `{turn: degrees, turn: degrees}`, in order | deg/s each |
| `ball` | a `ball` | its three turns, where they can make the rotation; and `quat`, the rotation from PartCAD's zero in its joint frame (`w x y z`) | its angular velocity `[x, y, z]`, deg/s about its moving joint frame |
| `floating` | no joint: a model of its own | its six freedoms, read from its pose against the body it was connected to; and `quat` | each freedom's rate by parameter, mm/s and deg/s, and `angular`, its angular velocity against that body, deg/s about its moving joint frame |

* **`pos` is PartCAD's value**: a number for a joint of one freedom, an object
  keyed by parameter (`turnZ`, `moveZ`, ...) for a joint of several. The
  export writes every joint *re-zeroed* at its start (see
  [Mechanisms](#mechanisms)), so Gazebo counts each joint from where the
  assembly shows it; the run adds the start back, so a pendulum started at
  `toParams: {turnZ: 270}` reads 270 at the start, not 0.
* It is read by PartCAD's own `kinematics.reading`, out of the **joint map**
  the export writes into the names file beside the world (each joint entry's
  `map`): which PartCAD joint and parameters an SDFormat joint is, the unit
  factor, the start, and for a free body the body its pose is read against.
  No copy of that arithmetic lives here.
* **A ball** is read from its two links' poses rather than from its joint
  state: Gazebo publishes two of a ball joint's three coordinates. Its
  rotation is exact at the reading's instant.
* **A floating child** is read from its pose against the body it was
  connected to, and its rates from the two bodies' velocities in the world.
* **Speeds of a ball and of a free body** are what the last step before the
  reading moved the bodies by, over the step: Gazebo publishes no link's
  velocity on any topic, so a `PosePublisher` system on the models concerned
  publishes every link's pose every step (the pose messages a run reads are a
  sixtieth of a second apart, too far to tell a speed by). Gazebo's DART steps
  by semi-implicit Euler, which moves a body by the speed it ends the step
  with, so the last step's difference *is* the engine's speed at the
  reading -- a free fall reads g·t exactly -- the same instant the joint
  states give every other joint's. At the first step there is no step before
  it, and the next step's speed is read.
* A turn nothing bounds is a **`revolute`**, whatever the world calls it —
  PartCAD has no `continuous`, and the MuJoCo plugin reports a hinge as a
  `revolute` too. Write one as a `revolute` with no `<limit>`: Harmonic's DART
  physics builds a `continuous` joint as a *fixed* one, and says so only in the
  server's own log.
* **No `effort`**, which the MuJoCo plugin reports. Gazebo's joint state
  publisher has a field for it and leaves it empty — it stays empty while a
  force is being applied to the joint — and a zero nobody measured would be
  read as a measurement.
* Not listed, because nothing there moves: a `fixed` joint, and every joint of
  a `<static>` model. Not read: joints of a model a world brings in with
  `<include>`, which are in another file — PartCAD's exporter writes everything
  into the one.
* **A world PartCAD did not write** has no names file, and so no joint map: a
  `revolute`, `continuous`, `prismatic` or `screw` joint is reported by its own
  coordinate, zero where the world placed the child link, a screw by its turn.
  A ball, a `universal`, a `revolute2` or a `gearbox` joint is named in
  `warnings` instead (there is no saying which of a universal joint's turns is
  which parameter), and so is one Gazebo ran with no freedom to move. A reading
  never quietly leaves out something that moved. Such a joint is reported under
  its own name, and as **`<model>::<joint>`** where another model has a joint
  of the same name — both of them, so that neither is the one the plain name
  means. A nested model is `outer::inner`.
* A world with no joints to read — a scene of parts — reports
  `"joints": {}`, so a validation can walk `after["joints"]` without asking
  first whether it is there.

```yaml
validation: after["joints"]["swing"]["pos"] > 80    # it swung down
```

Gazebo publishes a model's joint states only if the model carries its
`JointStatePublisher` system, and a world has no reason to. So when there is a
joint to read, the server runs a copy of the world with that system added to
each model that has one, on a topic of the run's own — and a `PosePublisher` to
each model with a ball or a floating child, or that a floating child is read
against — written beside the original, because a world names its
meshes relative to where it is, and removed when the run ends. A world with
nothing to read runs exactly as it was handed over. The states arrive every step
and each reading takes the one of its own simulated instant, so a reading's
joints and its bodies are the same moment; the first reading is the first
instant every joint stream has reached.

### Loads

Every body in a reading also says **what acts on it**, in the vocabulary
[partcad-sim-mujoco](https://github.com/partcad/partcad-sim-mujoco#loads)
states it in, as far as Gazebo tells a run: its `mass` and its centre of mass
(`com`, mm), and `loads`, each with its `kind`, what it comes `from`, where it
acts (`at`, mm), and the `force` (N) on the body and the `torque` (N·m) about
`at`, in the world's frame:

```json
"arm": {
  "pos": [500.0, 0.0, 1000.0], "quat": [...], "mass": 1.0, "com": [599.9, 0.0, 991.2],
  "loads": [
    {"kind": "joint", "from": "frame", "joint": "knee", "at": [0.0, 0.0, 1000.0],
     "force": [0.0, 0.0, 9.81], "torque": [0.0, -5.885, 0.0]},
    {"kind": "gravity", "from": "arm", "at": [599.9, 0.0, 991.2],
     "force": [0.0, 0.0, -9.81], "torque": [0.0, 0.0, 0.0]}
  ]
}
```

What Gazebo gives, and what it does not:

* **What a joint passes** — its constraint, and its spring, damper and friction
  — is what Gazebo's force-torque sensor measures, so the world that runs has
  one on every joint between two bodies, and the ForceTorque system on a model
  (a system attached to the world would turn Gazebo's defaults off). Each
  measures the wrench the parent applies to the child, about the joint's
  origin; the run turns it into the world's frame. The child reports it from
  its parent, and the parent the same reversed. The arm above — 1 kg, its
  centre 600 mm out, held level by its hinge's spring — reads its weight and
  m·g·d to four figures, as it does in MuJoCo.
* **A body's weight** is its link's `<inertial>` under the world's gravity, at
  its centre of mass.
* **Not reported, and said in `warnings`:** what holds a body a scene fixes —
  Harmonic makes no force-torque sensor on a joint to the world; a body's
  **contacts** and its **acceleration**, which this plugin does not read from
  Gazebo, so a body's loads here need not add up to its mass times anything;
  and, at the first reading, every joint's load: it is taken while the world is
  paused, and the sensors measure nothing then. So is a joint whose body is
  missing: a joint is stated about its child, so one whose child is not in a
  reading is not reported, one whose parent is not is reported on its child
  alone, and one whose link is none of the bodies in the names file gets no
  sensor at all.

Like the poses, loads are read for a world PartCAD exported — the names file
beside it places each body in the world's frame — and a result says which body
each part is in, as `parts`, with a PartCAD that can say which parts a body is
(`kinematics.parts_entry()`); with an older one a run reports the same loads and
no `parts`. `params: {loads: false}` reports poses alone.

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
* the child of a **floating** joint is held by nothing: it is a `<model>` of
  its own, at its start, with no joint at all — SDFormat's free body, which
  Gazebo's DART gives a free joint of its own;
* an element marked `fixed: true` is welded to the world by a fixed joint to
  `world`.

Each joint is **one SDFormat joint of SDFormat's own type**, or refused —
never a chain through links nobody built (section 8.13 of PartCAD's
[simulation design record](https://github.com/partcad/partcad/blob/devel/docs/design/simulation.md)):

| PartCAD's type (`kinematics.classify`) | SDFormat 1.11, as Gazebo Harmonic's DART builds it |
| --- | --- |
| `revolute`, bounded on either side or on neither | `revolute`, a side it leaves open left out of `<limit>` — **never `continuous`**, which DART builds as fixed |
| `prismatic` | `prismatic` |
| `screw` | `screw`, its `<screw_thread_pitch>` from the thread |
| `universal` | `universal` (`<axis>`, `<axis2>`) |
| `ball` | `ball` |
| `floating` | no joint: the child a model of its own |
| `cylindrical`, `planar` (moves first or turn first), `composite` | **refused** |

* **Refusals.** SDFormat has no joint for a cylindrical or a planar pair, and a
  joint joins exactly two links; DART builds `revolute2` and `gearbox` as
  welds, and refuses a second joint on a child that has one. The only way left
  to write one would be through links of a mass nobody measured — the 1 g
  "dummy links" this exporter used to write, which stood in for bodies a
  mechanism really has (a sleeve's carriage, a gimbal's ring). So the export
  fails, naming every such joint at once, its type and the engine, before it
  writes anything, and gives the two ways on: simulate the mechanism in MuJoCo
  ([partcad-sim-mujoco](https://github.com/partcad/partcad-sim-mujoco)), which
  writes every type; or model the body the joint stands for and join it by
  native joints.

  ```text
  Gazebo cannot simulate the cylindrical joint 'collar-shaft': the joints SDFormat has and Gazebo Harmonic's DART
  builds are revolute, prismatic, screw, universal and ball, ... Either simulate the mechanism in MuJoCo
  (partcad-sim-mujoco), which writes every type; or model the body the joint stands for - for a cylindrical pair a
  sleeve and its carriage, ... - and join it by native joints
  ```

* **Names.** Every joint keeps PartCAD's name — `<child>-<target>`, or the
  connection's `joint:` — so `after["joints"]["shoulder_pan"]` reads it. Every
  name goes through SDFormat's own rules (`SDF_NAMES` in `export_world.py`,
  applied by PartCAD's `wrappers/naming.py`): libsdformat reserves `world` and
  every `__…__`, and refuses `::` anywhere; and since a world's and a model's
  names are parts of Gazebo's transport topics, the alphabet is ASCII letters,
  digits, `_`, `.` and `-`. A name outside them is written as the nearest one
  inside (`world-2`, `a_b`), and the export says which -- so a scene called
  `world`, PartCAD's default world, is the SDFormat world `world-2` and still
  `world` in what a run reports. No two things in a world share a name, which
  is why a model's first link is not called after the model: Gazebo reports
  every pose by the bare name, and a link sharing its model's name was reported
  in the model's place. PartCAD's names are claimed first, so a name this
  exporter makes up (a model's `_link`, the ground plane) is the one told
  apart.
* **What a run reports.** With `names_file: true`, which the simulation asks
  for, the export writes `<world>.names.json` beside the world: which SDFormat
  name each body and joint became, which model and link each body is, and each
  joint's **map** back into PartCAD's terms (`kinematics.joint_map`; for a
  ball, the joint's frame in each of its two links as well, and for a floating
  child the body it is). The run reads it, reports every body, joint and the
  world under PartCAD's name, every joint in PartCAD's terms (see
  [Joints](#joints)), and states each body at its link's pose **in the world's
  frame** -- the model's pose composed with the link's, since Gazebo publishes a
  link relative to its model -- which is what the MuJoCo plugin reports for the
  same scene. A world with no such file is reported as Gazebo publishes it:
  every entity under its own name, relative to its parent.
* **Starting values, by re-zeroing.** SDFormat states no starting value for a
  joint — `<axis><initial_position>` was in 1.6 and 1.7, deprecated in 1.8 and
  gone from 1.9 — and Harmonic's gz-sim reads none from a world (the system
  that would set one from a `<model_state>` is gz-sim #2359, an unmerged
  draft). So every joint is written so that **Gazebo's zero is PartCAD's
  start** (`kinematics.rezeroed`): every link where the assembly shows it, in
  its model's frame, and every joint's frame where that configuration puts it
  — a joint of several freedoms takes a start into its frame only where that
  freedom commutes with every freedom before it, so a universal joint's first
  turn's start is in its frame and its second's in the child, and a ball's
  starting rotation is in its frame. Every position the joint states shifts
  with it: a bound `[l, u]` is written as `[l - s, u - s]`, a spring's
  reference `r` as `r - s`, and a mimic's offset `c` as `c + m s_leader -
  s_follower`. A start outside the joint's bounds is reported: Gazebo puts
  such a joint at its stop the moment it starts. The MuJoCo plugin starts a
  hinge by its own `ref` instead; the URDF exporter keeps PartCAD's zero.
* **Frames.** A joint's `<pose>` is the frame it turns or slides in, relative
  to its child link, and its `<axis><xyz expressed_in="...">` is in its parent
  link.
* **Bounds.** In radians or metres. A side that is open is left out:
  from SDFormat 1.10 an omitted `<lower>` is −∞ and an omitted `<upper>` +∞,
  which DART takes as no stop (1.9's default was 10¹⁶, which this used to
  write out). A screw is bounded by its turn; a bound on its move is reported.
  A ball's bound — one angle, a cone about its zero — is reported: SDFormat
  has none, and DART would bound each of its three coordinates, which is a box.
* **Physics.** `damping`, `friction`, `springStiffness` and `springReference`
  are `<axis><dynamics>`; `maxEffort` and `maxVelocity` are `<limit><effort>`
  and `<velocity>`. All in SI per radian or per metre; `springReference`, a
  position of the joint, converts from degrees or millimetres. They are written
  on every axis but one another axis moves (a screw's move). A ball's damping
  is written on an `<axis>` of its own, which DART applies to each of its three
  coordinates; its friction, spring and limits would be a box over those
  coordinates, and are reported, not written. So are soft limits and anything
  else a joint states.
* **The version** is SDFormat **1.11**, the newest libsdformat 14 reads, which
  is what Gazebo Harmonic carries. `version:` on the export writes an older one:
  a screw's pitch is `<thread_pitch>` before 1.10 (radians a metre, positive for
  a left-handed thread) and a mimic is reported rather than written before 1.11.
* **Mimics** are `<mimic>`. Gazebo Harmonic reads one and does not enforce it:
  its DART has no mimic constraint, so a mimicking joint moves freely there.
* **`static`** (the export's default, for a file to look at) welds every body
  nothing holds: a model with nothing moving in it is `<static>`, and the root of
  a mechanism gets a fixed joint to `world`, so a pendulum exported on its own
  still swings. A run asks for `static: false`. **`flatten` is gone**, and
  reported when it is passed.
* **`self_collide`** (on for a run) lets the links of one model collide.
  Gazebo's DART never collides the two links of one joint; two links of one
  mechanism that no joint joins — a part on a turning arm and the frame the arm
  turns in — collide only with it on.

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
  `ghcr.io/partcad/partcad-sim-gazebo`, this package's own image of ROS Jazzy's
  simulation stack, which is Gazebo Harmonic (`container/Dockerfile`), and
  which is then the second case. It is published for linux/amd64 and
  linux/arm64, each built natively, as one tag: OSRF's own
  `osrf/ros:jazzy-simulation`, which it is built the way of, is amd64 alone.

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
| `timestep` | Gazebo's, 0.001 | Integration step, in seconds: written into the world as `<physics><max_step_size>`, the step Gazebo's DART takes. (It used to be handed to the server as `gz sim -z`, which is an update rate, iterations a second of the wall clock, and changed only how fast a run went, never its step.) The run goes at real time whatever the step. |
| `gravity` | the scene's | m/s², in the scene's frame. Overrides the scene's `gravity:` for this run, by writing it into the world before Gazebo loads it; unset, the run is under the scene's, or SDFormat's `[0, 0, -9.8]` when the scene states none. |
| `timeout` | PartCAD's | Wall-clock seconds the whole run may take. PartCAD works it out and hands it to every run; `300.0` where it does not. See below. |
| `startup_timeout` | `120.0` | Wall-clock seconds the server may take to publish its first pose. |
| `loads` | `true` | Report what acts on every body, in every reading (see [Loads](#loads)); `false` reports poses alone and runs the world without force-torque sensors. |
| `samples` | `0` | Report the state at this many evenly spaced instants too, as `samples`: each the first pose message at or past its instant that no sample before it took, at its own `time` (an instant or so late at worst). Gazebo publishes poses sixty times a second *of the wall clock*, so a run asking for more readings a simulated second than that runs slower than real time to give them (`<real_time_factor>`), never more than one a step; if fewer come back than were asked for, `warnings` says how many of how many. |

The reader and the writer take parameters of their own; `partcad.yaml` lists
each one beside what it does.

## When a run goes wrong

A Gazebo run is several programs -- the server, a `gz topic -e` for the poses,
and one for the joints of each model that has any -- and every one of them can
wait for ever on another. A subscriber whose server has died hears nothing and
says nothing; so does one whose server hangs while it loads the world, or
publishes in another transport partition. So nothing here waits blindly:

- **A server that dies fails the run at once**, whenever it dies, with its exit
  status and the end of what it wrote, and with how far the run had got. A world
  the server refuses -- Harmonic exits, with status 0, on two links of one
  name -- used to leave the run waiting for its first pose for ever:

  ```text
  Gazebo published no poses for world 'w': its server stopped before the first one (exit status 0).
  The end of what the server wrote:
  [Err] [Server.cc:86] Error Code 2: Msg: link with name[l] already exists.
  [Err] [Server.cc:86] Error Code 9: Msg: Failed to load a world.
  ```

- **A first pose that does not come within `startup_timeout`** fails the run,
  saying which topic it listened on and in which partition.
- **Every wait is inside the run's budget**, `timeout`: for poses, for joint
  states (each joint stream is waited for at most ten seconds, and once for one
  that has said nothing), and for `gz sim --versions`. PartCAD hands every run
  its budget and stops the run when it is spent; this script stops a tenth of
  it earlier -- at least a second, at most thirty -- and says how far it got:
  `Gazebo did not reach 10 s of simulated time within 270 s of waiting (the run's
  budget is 300 s). 812 pose messages of world 'w' had been received, the last
  at t = 3.2 s of the 10 s asked for.`
- **Nothing outlives the run.** Every program is started in a process group of
  its own, and on every way out -- an answer, a failure, an exception -- each
  group is asked to stop and killed five seconds later, with whatever it
  started; the world's copy with the joint publishers in it is removed. The
  server's standard error is read as it comes, so a server with a lot to say
  never blocks on a full pipe.

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

`test_safety.py` proves each of the nets in "When a run goes wrong" with a fake
`gz` -- a few lines of Python found through `PC_GZ` -- that dies before its first
pose, hangs while it starts, publishes and then dies, or is too slow for its
budget, and checks that every program it was run as is gone afterwards. It needs
nothing but the standard library and a POSIX system. Its last test runs a world a
real Gazebo refuses, and is skipped without one, like the ones below.

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
     && /tmp/v/bin/python -m pytest -v -p no:cacheprovider test_joints.py test_simulate_gazebo.py test_partition.py test_loads.py'
```

`test_export_joints.py` builds mechanisms with PartCAD's own ASSY code, a
stand-in for every part's geometry, and checks every row of the table above,
each joint started away from its zero: the one joint written, under PartCAD's
name, with no `continuous` and no link nobody built; a refusal for each type
SDFormat has none for, naming every such joint at once and writing nothing;
the bounds omitted where they are open and shifted by the start where they are
not; and every link where PartCAD's `kinematics` puts its body — read back by a
forward kinematics of SDFormat written from its specification, at Gazebo's
zero (the assembly as drawn) and moved (PartCAD's start plus the move). Its
last tests need a Gazebo and `partcad` both: every row passes `gz sdf -k`;
started away from its zero with nothing to move it, every joint reads its
start; under a gravity along no axis, every reading put through PartCAD's own
joint is where Gazebo has the body, and a ball's speed is the turn of its
readings over time; a bar released horizontal on a hinge passes the bottom at
√(2mgd/I) and swings with the period 4K(sin 45°)√(I/mgd) of a right-angle
swing, and started half a turn round reads 180° and its bottom unwrapped; a
stop holds a hinge where PartCAD says, not where its start would; a carriage
falls from its start to its stop; a floating child falls free, at g·t; a lid
falling free beside the swinging bar has, against the bar, the rates the
engine's own velocities give and turns at the bar's speed; and a nut on a screw
moves 10 mm a turn. In the image, `pip install -q pytest numpy partcad`
and run it as above.

`test_loads.py` checks the force-torque sensors a run adds to a world — one
on every joint between two bodies, none on a static model's, the system once
on a model — the weights it reads out of the world, a sensor's wrench
turned into the world's frame at the joint on both bodies, and a joint whose
body is missing named in `warnings` rather than dropped, with PartCAD alone;
its last test runs an arm held level by its hinge's spring in a real Gazebo and
holds the hinge to the arm's weight and the moment of it.

`test_readings.py` reads each kind of joint back through the joint map the
exporter wrote, with PartCAD alone: a number for one freedom and an object
keyed by parameter for several, from PartCAD's zero; a ball from its links'
poses and its speed from their poses over the last step; a floating child from
its pose against the body it was connected to, and its rates from both bodies'
poses over the last step.

`test_names.py` checks SDFormat's rules, the sidecar, and a reading under
PartCAD's names in the world's frame with PartCAD alone; its `in_gazebo` tests
run a scene called `world` in a real Gazebo and hold every body to where
PartCAD's own joint puts it, and the last runs the same scene in MuJoCo as well
and holds the two engines to one pose. That one needs a checkout of
[partcad-sim-mujoco](https://github.com/partcad/partcad-sim-mujoco) named by
`PC_SIM_MUJOCO`, and `mujoco` installed, and is skipped otherwise.

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
