
[![Build and Test (Noble-ROS2)](https://github.com/ADVRHumanoids/xbot2_mujoco/actions/workflows/build-and-test-ros2-noble.yml/badge.svg)](https://github.com/ADVRHumanoids/xbot2_mujoco/actions/workflows/build-and-test-ros2-noble.yml)

# xbot2_mujoco
The MuJoCo simulator interfaced with Xbot2. The design is based around two main components:
 - The `MjcfGenerator`: turns a URDF into an MJCF file suitable for MuJoCo, by
   - converting the URDF into an MJCF file
   - merging it with user-defined XML files (e.g. to add a world, actuators, defaults, ...)
   - manipulating the resulting XML with the custom `<reference>` and `<setattribute>` tags
 - The `MjXbot2Bridge`: connects the simulation to an instance of Xbot2 running in a separate executable; communication happens via a UNIX socket, and uses JSON for serialization


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
`create_mj_model_data()` applies these values to `data.joint(name).qpos` and `data.actuator(name).ctrl` after creating the model.

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

# Remote control

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

Timed wrenches use force followed by torque, expressed in the world frame and
applied at the body center of mass. Duration is measured in simulation time and
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

Body mass and center of mass can be changed for one or more bodies. `com` is
expressed in the body's local frame. MuJoCo derived inertial constants are
recomputed after the change; the body's rotational inertia is not changed.

```json
{
  "id": "payload-1",
  "command": "set_body_properties",
  "bodies": ["base_link"],
  "properties": {
    "mass": 25.0,
    "com": [0.02, 0.0, 0.08]
  }
}
```

The `restore` command restores all contact and body properties managed by this
API to the values captured when the server started. It also cancels scheduled
wrenches and clears both Cartesian and generalized applied-force arrays. It does
not reset simulation position, velocity, control, or time.

```json
{
  "id": "restore-1",
  "command": "restore"
}
```

Successful responses contain `{"ok": true, "result": ...}`. Invalid requests
return `{"ok": false, "error": {"code": ..., "message": ...}}`; the optional
request `id` is echoed in either case. A command-line example is available at
[`python/examples/remote_control_client.py`](python/examples/remote_control_client.py).

