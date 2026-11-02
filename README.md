
[![Build and Test (Noble-ROS2)](https://github.com/ADVRHumanoids/xbot2_mujoco/actions/workflows/build-and-test-ros2-noble.yml/badge.svg)](https://github.com/ADVRHumanoids/xbot2_mujoco/actions/workflows/build-and-test-ros2-noble.yml)

# xbot2_mujoco
The MuJoCo simulator interfaced with Xbot2. The design is based around two main components:
 - The `MjcfGenerator`: turns a URDF into an MJCF file suitable for MuJoCo, by
   - converting the URDF into an MJCF file
   - merging it with user-defined XML files (e.g. to add a world, actuators, defaults, ...)
   - manipulating the resulting XML with the custom `<reference>` and `<setattribute>` tags
 - The `MjXbot2Bridge`: connects the simulation to an instance of Xbot2 running in a separate executable; communication happens via a UNIX socket, and uses JSON for serialization

## Velocity control with integral error

For a joint controlled by velocity, use MuJoCo's built-in `intvelocity`
actuator in the merged MJCF instead of a `position` actuator. Keep the actuator
name equal to the joint name so the Python bridge can find it:

```xml
<actuator>
  <intvelocity name="joint_name" joint="joint_name" kp="4" kv="2"
               inheritrange="1" />
</actuator>
```

The XBot command's `dq` is the velocity target, `d` is the proportional
velocity gain, `k` is the integral gain, and `tau` is feedforward torque. The
resulting joint torque is `tau + d*(dq - joint_velocity) +
k*integral(dq - joint_velocity)`. MuJoCo keeps the integral state in actuator
activation; the bridge initializes it to the current joint position on startup
and after a respawn. `inheritrange="1"` clamps that state to the joint range to
limit windup, so the joint must have a range. Position actuators retain the
existing position/velocity control law.

## Python actuator command delay

The Python simulator can delay complete XBot2 commands (position, velocity,
torque, and gains) by a fixed amount of **simulation time**:

```bash
python -m xbot2_py_mujoco.simulate ... --actuator-delay 0.02
```

`SimulatorWrapper(actuator_delay_s=0.02)` provides the same setting in Python.
The default is zero seconds. Until the first delayed command becomes due, the
model's initial actuator controls and gains remain in effect. At each physics
step, the newest due command is applied; superseded commands are skipped.
Pausing simulation also pauses the delay clock. The setting delays incoming
commands, not the physical actuator force response.

With `--remote-control-endpoint` enabled, change the delay while running:

```bash
mjctl actuator-delay --seconds 0.01
```

The new value applies to subsequently received commands. Commands already
waiting retain their deadlines. `mjctl restore` resets the value to the
startup setting, also leaving existing deadlines intact.
The equivalent remote JSON request is
`{"command": "set_actuator_delay", "delay": 0.01}`.


## Viewer pause controls

The default viewer key callback handles **Space** to pause/resume and **Right
Arrow** to advance approximately 16.7 ms of simulation time while paused. Advances run the normal
physics, bridge communication, remote-control processing, clock publication, and
terrain scans, with existing pacing and decimation. Repeated presses queue
additional advances. Full physics timesteps are used, so each advance may exceed
the requested duration by less than one timestep.

**+ / -** change the target real-time factor (RTF) in steps of 0.1, with a minimum
of 0.1. The main `=` key (the unshifted `+` key) and keypad `+ / -` also work. The
new target is printed, and pacing is rebased without accumulating timing error.
These controls also work while paused, affecting subsequent advances or resume.
For custom callbacks, call `set_target_rtf(value)` with a finite positive value.

While paused without an advance, only viewer synchronization and window-close
handling run. **R** respawn requests are ignored during pause, including advances;
entering pause discards pending respawns. Resuming clears active and queued
advances and restarts pacing without counting the idle pause.

`SimulatorWrapper.paused` is a read-only property. Call `set_paused()` to pause,
`set_paused(False)` to resume, or `advance(duration_s=0.1)` to queue a finite,
positive simulation-time advance while paused. Calls to `advance` while running
are ignored. These methods can also control headless simulations.

Passing `viewer_key_callback` replaces the default callback completely; custom
callbacks must implement these controls themselves, for example:

```python
def on_key(key):
    if key == ord(' '):
        sim.set_paused(not sim.paused)
    elif key == 262:  # GLFW_KEY_RIGHT
        sim.advance(duration_s=0.01666)
    elif key in (ord('+'), ord('='), 334):
        sim.set_target_rtf(round(sim.target_rtf + 0.1, 10))
    elif key in (ord('-'), 333):
        sim.set_target_rtf(max(0.1, round(sim.target_rtf - 0.1, 10)))
```

The callback records requests; physics and timing changes run on the simulation
thread. A simulation phase already executing when pause is requested may finish.


# Custom tags

Custom tags are XML elements that are **not** valid MuJoCo MJCF — they are processed and removed by `_finalize_xml()` before the tree is handed to MuJoCo.  They must be direct children of the root `<mujoco>` element and are typically injected via `merge_xml()`.

---

### `<reference element="E" name="N">`

Appends every child of the `<reference>` element into the first `<E name="N">` element found anywhere in the tree, then removes the `<reference>` tag.  
Use this to attach children (e.g. `<site>`, `<camera>`) to bodies or other elements that originate from the URDF and cannot be edited directly.

**Input (merged via `merge_xml`):**
```xml
<mujoco>
  <reference element="body" name="base_link">
    <site name="base_link" pos="0 0 0" size="0.01" />
  </reference>
</mujoco>
```

**Processed output:**
```xml
<body name="base_link" pos="0 0 0.5">
  <inertial .../>
  <geom .../>
  <site name="base_link" pos="0 0 0" size="0.01"/>  <!-- injected -->
</body>
<!-- <reference> element removed -->
```

---

### `<setattribute element="E" name="N">`

Sets one or more attributes on the first `<E name="N">` element found in the tree, using nested `<attribute name="..." value="..."/>` children.  The `<setattribute>` tag is then removed.  
Use this to assign MuJoCo attributes (e.g. `childclass`) to bodies that come from the URDF.

**Input (merged via `merge_xml`):**
```xml
<mujoco>
  <setattribute element="body" name="fl_hip_link">
    <attribute name="childclass" value="robot_leg" />
  </setattribute>
</mujoco>
```

**Processed output:**
```xml
<body name="fl_hip_link" pos="..." childclass="robot_leg">  <!-- attribute injected -->
  ...
</body>
<!-- <setattribute> element removed -->
```

---

### `<q_init>`

Declares the initial joint configuration.  Each `<joint name="..." value="..."/>` child sets `gen.q_init[name] = float(value)`.  The tag is removed entirely from the MJCF output (it is not a valid MuJoCo element).  
`create_mj_model_data()` applies these values to `data.joint(name).qpos`
and the corresponding position actuator control after creating the model. For
an `intvelocity` actuator, it initializes the actuator activation to the joint
position and sets its velocity control to zero.

**Input (merged via `merge_xml`):**
```xml
<mujoco>
  <q_init>
    <joint name="fl_hip"  value="0.15" />
    <joint name="fl_knee" value="-0.4" />
  </q_init>
</mujoco>
```

**Processed output:**
```python
gen.q_init == {"fl_hip": 0.15, "fl_knee": -0.4}
# <q_init> element removed from the final MJCF
```
`create_mj_model_data()` then applies:
```python
data.joint("fl_hip").qpos  = 0.15;  data.actuator("fl_hip").ctrl  = 0.15
data.joint("fl_knee").qpos = -0.4;  data.actuator("fl_knee").ctrl = -0.4
```

---

# MJCF (XML) generation

`MjcfGenerator` converts a URDF into a MuJoCo-ready MJCF file. It supports common use needs such as:
 - including the robot into a separate world file
 - adding actuators, contact filtering, defaults (as well as any other mujoco tag that should specified as direct child of the root `<mujoco>` element)
 - adding children to any XML element using the `<reference>` tag
 - adding attributes to any XML element using the  `<setattribute>` tag
 - specifying an initial robot configuration

See [config.xml](python/tests/resources/config.xml) file used for unit tests as an example!

The class is used as follows:

```python
gen = MjcfGenerator(name='robot', urdf_str=urdf_str, output_dir='/tmp/robot_mujoco')
gen.merge_xml(Path('options.xml'))   # merge extra MJCF options (repeatable)
gen.merge_xml(Path('world.xml'))
mjcf_str = gen.generate_mjcf_string()
model = mujoco.MjModel.from_xml_string(mjcf_str)
```

### Steps performed

**1. Construction (`__init__`)**
- Resolves `package://` URIs in the URDF and symlinks (or copies) mesh assets into `<output_dir>/assets/`
- Convert the resolved URDF to produce an initial MJCF (`<name>.orig.xml`)

**2. `merge_xml(xml_str | Path)`**
- Merges an additional MJCF snippet into the tree using a recursive strategy: child elements are matched by tag (and `name` attribute when present); existing nodes are recursed into, new nodes are appended. This allows injecting `<default>`, `<actuator>`, `<sensor>`, `<contact>`, `<option>`, custom tags, etc.

**3. `generate_mjcf_string()` / `create_mj_model_data()`**
- Calls `_finalize_xml()` which processes custom tags injected via `merge_xml`:
  - **`<reference element="E" name="N">`** — appends its children into the first `<E name="N">` element found in the tree, then removes itself. Useful for adding `<site>` or other children to bodies defined in the URDF.
  - **`<setattribute element="E" name="N">`** — sets attributes (via nested `<attribute name="..." value="..."/>`) on the target element, then removes itself. Useful for assigning MuJoCo `childclass` to bodies.
  - **`<q_init>`** — parses `<joint name="..." value="..."/>` children to populate `gen.q_init`, then removes itself.
- Serialises the final tree to a string and writes it to `<output_dir>/<name>.xml`
- `create_mj_model_data()` additionally calls `mujoco.MjModel.from_xml_string()` and applies `q_init` to the returned `MjData`

### Output directory layout
```
<output_dir>/
  <name>.urdf        # pre-processed URDF (assets resolved)
  <name>.orig.xml    # raw output from mujoco_compile
  <name>.xml         # final MJCF (written by generate_mjcf_string)
  assets/            # mesh/texture files (symlinked or copied)
```

# Open stairs terrain

The terrain generator supports `MeshOpenPyramidStairsTerrainCfg` and
`MeshOpenInvertedPyramidStairsTerrainCfg`, matching the open stairs geometry in
`kyon_isaac`. Four thin box plates form each rectangular tread ring, with a
central landing and a catch floor 0.5 m below the lowest walking surface.
Step height interpolates with terrain difficulty. Dimensions are in meters:

```python
from xbot2_py_mujoco.utils.terrain_generator import (
    MeshOpenPyramidStairsTerrainCfg,
    MeshOpenInvertedPyramidStairsTerrainCfg,
    TerrainGenerator,
    TerrainGeneratorCfg,
)

stairs = dict(
    step_height_range=(0.10, 0.30),
    step_width=0.3,
    platform_width=3.0,
    border_width=1.0,
    tread_thickness=0.02,
    tread_overlap=0.05,
)
generator = TerrainGenerator(TerrainGeneratorCfg(
    seed=0,
    size=(8.0, 8.0),
    num_rows=4,
    num_cols=2,
    curriculum=True,
    sub_terrains={
        "open_stairs": MeshOpenPyramidStairsTerrainCfg(**stairs),
        "open_stairs_inv": MeshOpenInvertedPyramidStairsTerrainCfg(**stairs),
    },
))
generator.save("/tmp/open_stairs/world.xml")
```

`step_width` controls horizontal advance; `tread_overlap` extends an upper
tread over its lower neighbor. Thickness must be smaller than the minimum
step height, and overlap must be nonnegative and smaller than `step_width`.
Inverted stairs also require a positive opening above the landing. Generated
terrain geoms use group **2**, and `spawn_locations.yaml` places spawns above
each central landing using `spawn_height`.

# Terrain height scan

The Python simulator supports a local N×M grid of vertical rays for terrain
observations. No lidar or MJCF sensor configuration is needed:

**Model requirement:** use group **0** for robot visuals and group **1** for robot
collisions. Both groups are excluded from raycasting, so they can still be
hidden independently in the viewer. Assign terrain and any obstacles to scan to
group **2** (or another included group). Configure these groups in your MJCF;
the scanner does not assign or change them. The terrain generator utility
assigns group **2** to generated terrain geoms, including outer borders, through
its `terrain` default class. For example:

```xml
<default>
  <default class="robot_visual">
    <geom group="0" contype="0" conaffinity="0"/>
  </default>
  <default class="robot_collision">
    <geom group="1"/>
  </default>
</default>
<worldbody>
  <geom name="terrain" type="plane" size="10 10 0.1" group="2"/>
  <body name="base_link">
    <geom class="robot_visual" type="box" size="0.2 0.1 0.1"/>
    <geom class="robot_collision" type="box" size="0.2 0.1 0.1"/>
    <!-- Apply these classes to the corresponding geoms in descendant links. -->
  </body>
</worldbody>
```

Existing per-geom `group` attributes or nested default classes can override this
inheritance; ensure every robot geom uses an excluded group. The constructor
checks the named robot subtree. Any environment geoms in groups 0 and 1 are
also skipped. To use another convention, pass `excluded_geom_groups`, a list or
tuple of group IDs (0–5); the default is `(0, 1)`.

From the command line, enable scanning with `--terrain-scan-body`:

```bash
python -m xbot2_py_mujoco.simulate --urdf robot.urdf --xml terrain.xml \
  --terrain-scan-body base_link \
  --terrain-scan-shape 17 11 --terrain-scan-spacing 0.1 0.1 \
  --terrain-scan-z-offset 2 --terrain-scan-max-distance 5 \
  --terrain-scan-excluded-groups 0 1 --terrain-scan-decimation 100
```

Scanning is disabled unless `--terrain-scan-body` is provided. The options shown
above use the defaults. Add `--no-terrain-scan-visualization` to compute scans
without showing spheres. Scanning also works with `--headless`.

From Python:

```python
from xbot2_py_mujoco.terrain_scan import TerrainScanner
from xbot2_py_mujoco.simulator_wrapper import SimulatorWrapper

scanner = TerrainScanner(
    model, robot_body="base_link",  # robot root: all descendants are excluded
    shape=(17, 11), spacing=(0.1, 0.1),
    z_offset=2.0, max_distance=5.0,
    excluded_geom_groups=(0, 1),
)
sim = SimulatorWrapper(model, terrain_scanner=scanner, terrain_scan_decimation=10)
sim.run()  # call repeatedly in your simulation loop
# After 10 steps, sim.terrain_scan holds the latest scan (initially None).
```

For on-demand observations, call `scan = scanner.scan(data)` between simulation
steps. Use the same model and data as the simulator, on the simulation thread.
The scanner refreshes kinematics to use the current position after integration.
It does not advance time or change controls, velocities, forces, or model settings.

The grid is centered on the root body's world position and follows its yaw:
the forward axis is the body's X axis projected onto world XY, and the lateral
axis is perpendicular to it in that plane. Roll and pitch do not tilt the grid
or the rays. If the body's X axis is vertical, its heading is undefined and the
grid falls back to world X. `shape=(N, M)` means N forward samples and M lateral
samples; spacing is in meters. Odd dimensions sample the center; even
dimensions straddle it symmetrically. Rays start at the body's world Z plus
`z_offset`, point along world −Z, and stop at `max_distance`. Choose the start
height above the terrain of interest. The first surface below that height wins.

`scan.heights` contains world Z in meters, indexed `[forward_index, lateral_index]`.
`scan.x` and `scan.y` are N×M arrays giving each ray's world coordinates (including
at missed rays); `scan.center`,
`scan.ray_start_height`, and `scan.time` record the observation pose and simulation
time. Misses have a NaN height, `scan.valid == False`, and `scan.geom_ids == -1`.
For a relative-height policy input, subtract `scan.center[2]` from the heights
and handle misses using the validity mask before passing them to the policy.

Each grid point uses one `mujoco.mj_ray` call with groups 0 and 1 excluded and
static geoms included. Visible geoms in the remaining groups are considered,
including dynamic objects, primitives, meshes, and height fields. Fully
transparent geoms are skipped by MuJoCo's ray API. Select the robot's root body
to center the scan and validate all its links. Keep group assignments unchanged
while using the scanner, and recreate it after recompiling the model. Use
`terrain_scan_decimation` to match the policy's observation rate.

With `ros=True` and scanning enabled, each new scan is also published as
`sensor_msgs/msg/PointCloud2` on `/heightscan`. Change the topic with
`--terrain-scan-topic TOPIC` or `SimulatorWrapper(terrain_scan_topic=TOPIC)`.
Publishing uses sensor-data QoS (best effort, volatile) at the configured scan
decimation and also works headless. With `ros=False` / `--disable-ros`, scans
stay local and no cloud publisher is created.

The cloud's `frame_id` is the scan body name, which should match the robot root
link's ROS frame (for example `pelvis`). XYZ coordinates use that body's full
local frame, including roll and pitch, captured when the scan was acquired.
The header stamp uses MuJoCo simulation time. Consumers using TF should use the
same simulation time base. The organized cloud has `height=N`, `width=M`, and
FLOAT32 `x`, `y`, `z` fields, in forward/lateral matrix order. Missed rays retain
their slots as NaN XYZ points; `is_dense` is false if any point is invalid.
ROS environments need `sensor_msgs` and `sensor_msgs_py` installed.

When the viewer is enabled, the latest scan appears as spheres of radius 2.5 cm
centered at valid terrain hit points. Hue varies linearly with world height:
blue for the lowest hit, green midway, and red for the highest. The color range
is recomputed for each scan; flat scans are green and missed rays have no marker.
Markers update with the scan and coexist with remote payload and force overlays.
Set `terrain_scan_visualization=False` in `SimulatorWrapper` to hide them.

For a custom viewer, append markers under its lock with
`scan.add_visuals(viewer.user_scn, radius=0.025, height_range=(-0.5, 0.5))`.
An explicit `height_range` fixes the color scale across scans, with heights
outside it clamped to the endpoint colors. Clear `viewer.user_scn.ngeom` before
rebuilding your overlays, then call `viewer.sync()`. Markers add no collision
geometry and are limited by the scene's available geom capacity.

# Remote control

Install the Python package from the repository root to get the `mjctl` command
(activate your Python environment first):

```bash
python -m pip install -e ./python
mjctl --help
mjctl wrench --help
```

Re-run the installation after updating an existing checkout to register the new
entrypoint. The installed package also supports
`python -m xbot2_py_mujoco.mjctl` with the same arguments.

`mjctl` sends one command to a running simulator and prints the server's JSON
reply. Start the simulator with remote control enabled as shown below, then run
`mjctl` in another terminal. No ROS connection is needed by the client.

| Command | Arguments | Effect |
| --- | --- | --- |
| `friction` | `BODY... --values SLIDING TORSIONAL ROLLING [--regex]` | Set friction on directly attached geoms. Sliding is dimensionless; torsional and rolling coefficients are in meters. All must be non-negative. |
| `wrench` | `BODY --force X Y Z --torque X Y Z --duration SECONDS [--frame world\|body] [--regex]` | Apply force (N) and torque (Nm) at the COM for a positive duration in simulation time. Both vectors are required; use zeros where needed. |
| `body` | `BODY... [--mass KG] [--com X Y Z] [--regex]` | Set positive total mass and/or COM in local meters. At least one option is required; rotational inertia stays unchanged. |
| `payload` | `BODY --mass KG --position X Y Z [--regex]` | Attach a positive point mass at a local position in meters, replacing the previous payload relative to startup properties. |
| `torque-limits` | `JOINT... --limit VALUE [--exact]` | Set a non-negative symmetric net actuator limit in Nm for hinges or N for slides. Zero disables output. Joint selectors are full-name regex by default. |
| `actuator-delay` | `--seconds SEC` | Set the uniform XBot2 command delay in non-negative simulation seconds. Zero disables it. Pending commands keep their deadlines. |
| `velocity-noise` | `JOINT... --stddev VALUE [--exact]` | Add independent Gaussian noise to joint velocity feedback sent to XBot2. Standard deviation is in rad/s for hinges and m/s for slides; zero disables it. Joint selectors are full-name regex by default. |
| `restore` | None | Restore API-managed startup properties and actuator delay, clear velocity feedback noise, remove payloads, cancel wrenches, and clear all applied forces without resetting simulation state. |

Body selectors use exact names by default; `--regex` enables Python regular
expressions matched against the entire name. Quote shell patterns such as
`'.*_foot'`. For `wrench` and `payload`, a pattern may select multiple bodies;
each receives the same wrench or payload. Unmatched selectors or invalid values
reject the request without mutation. Use model body/joint names, not geom or
actuator names.

Global options precede the command:

```bash
mjctl --endpoint tcp://127.0.0.1:5555 --timeout 10 restore
```

`--endpoint` defaults to `tcp://127.0.0.1:5555` and must target the enabled
server. `--timeout` defaults to 5 wall-clock seconds per send/reply, independently
of a wrench's simulation-time duration. A missing server or stalled simulation
produces a diagnostic on stderr instead of waiting indefinitely. A timeout does
not cancel a request already sent; it may have taken effect, so check the
simulation before retrying (especially for additive wrenches).

JSON replies go to stdout, including server error replies. Exit codes are **0**
for success, **1** for server rejection, communication failure or an invalid
reply, **2** for CLI usage errors, and **130** for interruption. Numeric property
validation and name resolution are performed by the server; CLI timeout and
wrench duration must be finite and positive. Every command has its own
`--help`, with units, behavior and an example.

The Python simulator can expose an optional ZeroMQ `REP` endpoint for changing
contact properties and applying timed external wrenches. Enable it with:

```bash
python -m xbot2_py_mujoco.simulate ... \
  --remote-control-endpoint tcp://127.0.0.1:5555
```

The endpoint is disabled unless the option is provided. Bind to a non-loopback
address only on a trusted network: the initial protocol has no authentication or
encryption.

Requests and responses are JSON objects. Contact properties are changed on all
geoms directly attached to each named body (descendant bodies are not included):

Body commands accept `"regex": true` to interpret selectors as Python regular
expressions matched against the **entire body name**. Without this option, body
names are exact matches. Use `.*` for arbitrary prefixes or suffixes, for example
`"bodies": [".*_foot"]`. Overlapping selectors are deduplicated. Invalid patterns
or any selector with no matches reject the whole request before mutation.
`apply_wrench` and `add_payload` accept either one `"body"` selector or a list of
`"bodies"`; a regex can select multiple bodies, with the same force or payload
applied independently to each match. Responses contain the resolved body names.
`mjctl`'s `friction`, `body`, `payload`, and `wrench` commands expose
regex selection through `--regex`, for example:

```bash
mjctl friction '.*_foot' \
  --regex --values 0.05 0.0001 0.00001
```

```json
{
  "id": "contact-1",
  "command": "set_contact_parameters",
  "bodies": ["left_foot", "right_foot"],
  "parameters": {
    "friction": [0.8, 0.005, 0.001],
    "solref": [0.004, 1.2],
    "condim": 3
  }
}
```

Supported properties are `friction`, `solref`, `solimp`, `margin`, `gap`,
`condim`, and `priority`. A request is rejected without making changes if any
body or value is invalid.

Timed wrenches use force followed by torque, expressed in the world frame by
default and applied at the body center of mass. Duration is measured in simulation time and
overlapping commands add together:

```json
{
  "id": "push-1",
  "command": "apply_wrench",
  "body": "base_link",
  "force": [100.0, 0.0, 0.0],
  "torque": [0.0, 0.0, 5.0],
  "duration": 0.25
}
```

Set `"frame": "body"` on an `apply_wrench` request to express both force and torque
in the body's local axes. Their world directions are recomputed from the current
body orientation every simulation step for the whole duration. The application
point remains the body's center of mass. World-frame and body-frame wrenches can
overlap and add together. `mjctl` exposes this as `--frame body`:

```bash
mjctl wrench base_link \
  --force 100 0 0 --torque 0 0 5 --duration 1.0 --frame body
```

Body mass and center of mass can be changed for one or more bodies. `com` is
expressed in the body's local frame. MuJoCo derived inertial constants are
recomputed after the change; the body's rotational inertia is not changed.

```json
{
  "id": "body-1",
  "command": "set_body_properties",
  "bodies": ["base_link"],
  "properties": {
    "mass": 25.0,
    "com": [0.02, 0.0, 0.08]
  }
}
```

To attach a point payload, use `add_payload`. Its positive `mass` is added to the
body's startup mass, and `position` is expressed in the body's local frame, in
meters. The combined center of mass and rotational inertia are computed using
the parallel-axis theorem, using the body's mass, COM, and inertial properties
captured when the server started.
The payload has no intrinsic rotational inertia and adds no collision geometry.
Each request replaces the payload on that body and recomputes from those startup
properties, including after direct body-property edits. Repeating the same request
leaves the body properties unchanged. Simulation pose, velocity, and time are
preserved.

```json
{
  "id": "payload-1",
  "command": "add_payload",
  "body": "base_link",
  "mass": 5.0,
  "position": [0.2, 0.0, 0.1]
}
```

`mjctl` supports the same operation:

```bash
mjctl payload base_link \
  --mass 5.0 --position 0.2 0.0 0.1
```

Joint torque limits can be changed while the simulation is running using
`set_joint_torque_limits`. Joint selectors use full-name regex matching by
default; set `"regex": false` for exact names. The non-negative `limit` sets a
symmetric range `[-limit, limit]` on the net actuator torque at each selected
hinge joint, after gearing and summing actuator outputs. For slide joints it
limits force in newtons instead of torque in Nm. A zero limit disables actuator
output at the selected joints. Free and ball joints are rejected.

```json
{
  "command": "set_joint_torque_limits",
  "joints": [".*_knee", ".*_ankle.*"],
  "limit": 40.0
}
```

```bash
mjctl torque-limits '.*_knee' \
  --limit 40
```

Use `--exact` with `torque-limits` or `velocity-noise` to select literal joint names.
For example, `mjctl velocity-noise '.*_knee' --stddev 0.05` adds zero-mean
noise to each selected velocity feedback sample. The simulator's true joint
velocities and actuator calculations remain unchanged. The limits take
effect on the next simulation step without resetting pose, velocity, controls,
or time. Existing actuator force limits still apply and can further restrict
output. These joint limits constrain actuator output, not external contact forces
or remotely applied wrenches.

The `restore` command restores all contact, body, and joint torque-limit properties managed by this
API to the values captured when the server started. It also cancels scheduled
wrenches, removes added payloads (including their inertia changes), and clears
both Cartesian and generalized applied-force arrays. It restores the startup
actuator command delay and clears velocity feedback noise; queued commands keep
their existing deadlines. It does
not reset simulation position, velocity, control, or time.

```json
{
  "id": "restore-1",
  "command": "restore"
}
```

Successful responses contain `{"ok": true, "result": ...}`. Invalid requests
return `{"ok": false, "error": {"code": ..., "message": ...}}`; the optional
request `id` is echoed in either case. Use `mjctl` for the commands above; the legacy
[`python/examples/remote_control_client.py`](python/examples/remote_control_client.py)
script delegates to the same entrypoint.

When the viewer is enabled, remote payloads appear as translucent amber spheres
at their specified local positions, following the attached body. Sphere volume
is proportional to payload mass: a 1 kg payload has a 5 cm radius. Replacing a
payload updates its sphere; `restore` removes it. A direct body-property edit
also removes that body's payload marker.

Red arrows show the net active remote force on each body, starting at its current
center of mass. Arrow length scales with force magnitude (100 N gives a 0.5 m
arrow). Body-frame forces track the body's orientation, overlapping forces sum,
and expired or cancelled forces disappear. Pure torques have no force arrow.
These markers are viewer overlays and add no mass or collision geometry.
