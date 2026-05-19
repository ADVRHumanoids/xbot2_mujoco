
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



