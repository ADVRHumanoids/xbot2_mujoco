"""Human-readable dump of a MuJoCo model, the same as adarl's mujoco_utils.format_mj_model."""
import os
import tempfile

import mujoco
import numpy as np


def format_mj_model(model: mujoco.MjModel, *, full_dump: bool = False) -> str:
    """Return a structured human-readable description of a mujoco.MjModel.

    `print(model)` is uninformative and `mujoco.mj_printModel` produces a wall
    of low-level array dumps. This walks the main tables (bodies / joints /
    dofs / actuators / sensors / cameras / geoms / sites) showing names, indices,
    and the most useful per-element fields.

    Parameters
    ----------
    full_dump : bool
        If True, append the full mujoco.mj_printModel output (very long).
    """
    M = model
    out: list[str] = []

    def name_of(objtype: int, oid: int, default: str = "<unnamed>") -> str:
        n = mujoco.mj_id2name(M, objtype, int(oid))
        return n if n is not None else default

    def fmt_arr(a) -> str:
        return np.array2string(np.asarray(a), precision=3, suppress_small=True, separator=" ")

    mjtJoint = mujoco.mjtJoint
    jtype_map = {
        int(mjtJoint.mjJNT_FREE):  "FREE",
        int(mjtJoint.mjJNT_BALL):  "BALL",
        int(mjtJoint.mjJNT_SLIDE): "SLIDE",
        int(mjtJoint.mjJNT_HINGE): "HINGE",
    }

    out.append("=== MjModel ===")
    out.append(f"  nq={M.nq}  nv={M.nv}  na={M.na}  nu={M.nu}")
    out.append(f"  nbody={M.nbody}  njnt={M.njnt}  ngeom={M.ngeom}  nsite={M.nsite}")
    out.append(f"  nsensor={M.nsensor}  ncam={M.ncam}  nlight={M.nlight}  neq={M.neq}  ntendon={M.ntendon}")
    out.append(f"  nmesh={M.nmesh}  nhfield={M.nhfield}  ntex={M.ntex}  nmat={M.nmat}")
    out.append(f"  opt: timestep={M.opt.timestep}  integrator={int(M.opt.integrator)}  "
               f"iterations={M.opt.iterations}  ls_iterations={M.opt.ls_iterations}  "
               f"impratio={M.opt.impratio}")

    mjtObj = mujoco.mjtObj
    out.append("")
    out.append("=== Bodies ===")
    for bid in range(M.nbody):
        pid = int(M.body_parentid[bid])
        pname = name_of(mjtObj.mjOBJ_BODY, pid) if pid != bid else "<root>"
        out.append(f"  [{bid:3d}] {name_of(mjtObj.mjOBJ_BODY, bid):30s} "
                   f"parent={pname:30s} mass={float(M.body_mass[bid]):10.4g}  "
                   f"pos={fmt_arr(M.body_pos[bid])}  ipos={fmt_arr(M.body_ipos[bid])}")

    out.append("")
    out.append("=== Joints ===")
    for jid in range(M.njnt):
        jtype = jtype_map.get(int(M.jnt_type[jid]), str(int(M.jnt_type[jid])))
        body = name_of(mjtObj.mjOBJ_BODY, int(M.jnt_bodyid[jid]))
        out.append(f"  [{jid:3d}] {name_of(mjtObj.mjOBJ_JOINT, jid):30s} "
                   f"type={jtype:5s}  body={body:30s} "
                   f"qpos[{int(M.jnt_qposadr[jid]):3d}] dof[{int(M.jnt_dofadr[jid]):3d}]  "
                   f"limited={bool(M.jnt_limited[jid])}  range={fmt_arr(M.jnt_range[jid])}")

    out.append("")
    out.append("=== DoFs ===")
    for did in range(M.nv):
        jid = int(M.dof_jntid[did])
        jname = name_of(mjtObj.mjOBJ_JOINT, jid)
        jtype = jtype_map.get(int(M.jnt_type[jid]), str(int(M.jnt_type[jid])))
        out.append(f"  [{did:3d}] {jname:30s} {jtype:5s}  "
                   f"damping={float(M.dof_damping[did]):10.4g}  "
                   f"armature={float(M.dof_armature[did]):10.4g}  "
                   f"frictionloss={float(M.dof_frictionloss[did]):10.4g}")

    if M.nu > 0:
        out.append("")
        out.append("=== Actuators ===")
        for aid in range(M.nu):
            trntype = int(M.actuator_trntype[aid])
            trnid = int(M.actuator_trnid[aid, 0])
            if trntype == int(mujoco.mjtTrn.mjTRN_JOINT):
                target = "joint:" + name_of(mjtObj.mjOBJ_JOINT, trnid)
            else:
                target = f"trntype={trntype} trnid={trnid}"
            out.append(f"  [{aid:3d}] {name_of(mjtObj.mjOBJ_ACTUATOR, aid):30s} "
                       f"target={target:38s} ctrlrange={fmt_arr(M.actuator_ctrlrange[aid])}  "
                       f"forcerange={fmt_arr(M.actuator_forcerange[aid])}  "
                       f"gear={fmt_arr(M.actuator_gear[aid])}")

    if M.nsensor > 0:
        out.append("")
        out.append("=== Sensors ===")
        for sid in range(M.nsensor):
            objtype = int(M.sensor_objtype[sid])
            objid = int(M.sensor_objid[sid])
            obj = name_of(objtype, objid) if objtype != 0 else "-"
            out.append(f"  [{sid:3d}] {name_of(mjtObj.mjOBJ_SENSOR, sid):30s} "
                       f"type={int(M.sensor_type[sid]):3d}  adr={int(M.sensor_adr[sid]):4d}  "
                       f"dim={int(M.sensor_dim[sid]):2d}  obj={obj}")

    if M.ncam > 0:
        out.append("")
        out.append("=== Cameras ===")
        for cid in range(M.ncam):
            body = name_of(mjtObj.mjOBJ_BODY, int(M.cam_bodyid[cid]))
            out.append(f"  [{cid:3d}] {name_of(mjtObj.mjOBJ_CAMERA, cid):30s} "
                       f"body={body:30s} mode={int(M.cam_mode[cid])}  "
                       f"pos={fmt_arr(M.cam_pos[cid])}  fovy={float(M.cam_fovy[cid]):.2f}")

    if M.ngeom > 0:
        out.append("")
        out.append("=== Geoms ===")
        for gid in range(M.ngeom):
            body = name_of(mjtObj.mjOBJ_BODY, int(M.geom_bodyid[gid]))
            out.append(f"  [{gid:3d}] {name_of(mjtObj.mjOBJ_GEOM, gid):30s} "
                       f"type={int(M.geom_type[gid]):2d}  body={body:30s} "
                       f"size={fmt_arr(M.geom_size[gid])}  pos={fmt_arr(M.geom_pos[gid])}  "
                       f"contype={int(M.geom_contype[gid])} conaffinity={int(M.geom_conaffinity[gid])}")

    if M.nsite > 0:
        out.append("")
        out.append("=== Sites ===")
        for sid in range(M.nsite):
            body = name_of(mjtObj.mjOBJ_BODY, int(M.site_bodyid[sid]))
            out.append(f"  [{sid:3d}] {name_of(mjtObj.mjOBJ_SITE, sid):30s} "
                       f"body={body:30s} pos={fmt_arr(M.site_pos[sid])}")

    out.append("")
    out.append("=== opt ===")
    out.append(str(M.opt))

    if full_dump:
        tmp = tempfile.NamedTemporaryFile(suffix=".txt", delete=False)
        tmp.close()
        try:
            mujoco.mj_printModel(M, tmp.name)
            with open(tmp.name, "r") as f:
                out.append("")
                out.append("=== mj_printModel (verbose) ===")
                out.append(f.read())
        finally:
            os.unlink(tmp.name)

    return "\n".join(out)


def print_mj_model(model: mujoco.MjModel, *, full_dump: bool = False, file: str | None = None) -> None:
    """Pretty-print a mujoco.MjModel. See `format_mj_model` for options.

    If `file` is given, the text is written to that path instead of stdout.
    """
    text = format_mj_model(model, full_dump=full_dump)
    if file is not None:
        with open(file, "w") as f:
            f.write(text)
    else:
        print(text)
