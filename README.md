# xbot2_mujoco
The MuJoCo simulator interfaced with Xbot2. The design is based around two main components:
 - The `MjcfGenerator`: turns a URDF into an MJCF file suitable for MuJoCo
 - The `MjXbot2Bridge`: connects the simulation to an instance of Xbot2 running in a separate executable

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



