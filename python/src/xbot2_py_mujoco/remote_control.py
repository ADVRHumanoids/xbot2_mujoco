"""ZeroMQ/JSON remote control for a running MuJoCo simulation."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from typing import Any

import mujoco
import numpy as np
import zmq


class CommandError(ValueError):
    """A client-visible command validation error."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class _ActiveWrench:
    body_id: int
    wrench: np.ndarray
    expires_at: float
    frame: str


@dataclass(frozen=True)
class _PointPayload:
    mass: float
    position: np.ndarray


class RemoteControlServer:
    """Non-blocking REP server whose mutations are driven by the sim thread."""

    _RESTORABLE_MODEL_FIELDS = (
        "geom_friction",
        "geom_solref",
        "geom_solimp",
        "geom_margin",
        "geom_gap",
        "geom_condim",
        "geom_priority",
        "body_mass",
        "body_ipos",
        "body_inertia",
        "body_iquat",
        "jnt_actfrclimited",
        "jnt_actfrcrange",
    )

    _VECTOR_PARAMETERS = {
        "friction": ("geom_friction", 3),
        "solref": ("geom_solref", 2),
        "solimp": ("geom_solimp", 5),
    }
    _SCALAR_PARAMETERS = {
        "margin": ("geom_margin", float),
        "gap": ("geom_gap", float),
        "condim": ("geom_condim", int),
        "priority": ("geom_priority", int),
    }

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        endpoint: str,
        max_requests_per_step: int = 16,
    ):
        if not endpoint:
            raise ValueError("Remote-control endpoint must not be empty")
        if max_requests_per_step <= 0:
            raise ValueError("max_requests_per_step must be positive")

        self.model = model
        self.data = data
        self._constant_data: mujoco.MjData | None = None
        self.max_requests_per_step = max_requests_per_step
        self._active_wrenches: list[_ActiveWrench] = []
        self._payloads: dict[int, _PointPayload] = {}
        self._managed_body_ids: set[int] = set()
        self._original_model_values = {
            name: getattr(model, name).copy()
            for name in self._RESTORABLE_MODEL_FIELDS
        }
        self._socket = zmq.Context.instance().socket(zmq.REP)
        self._socket.setsockopt(zmq.LINGER, 0)
        try:
            self._socket.bind(endpoint)
        except Exception:
            self._socket.close()
            raise
        self.endpoint = self._socket.getsockopt_string(zmq.LAST_ENDPOINT)

    def close(self) -> None:
        if self._socket is not None:
            self.clear_wrenches()
            self._socket.close()
            self._socket = None

    def clear_wrenches(self) -> None:
        if self._managed_body_ids:
            self.data.xfrc_applied[list(self._managed_body_ids)] = 0.0
        self._active_wrenches.clear()
        self._managed_body_ids.clear()

    def process_requests(self) -> int:
        """Process queued requests without blocking and return their count."""
        if self._socket is None:
            return 0

        processed = 0
        while processed < self.max_requests_per_step:
            try:
                payload = self._socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break

            request_id = None
            try:
                request = json.loads(payload)
                if isinstance(request, dict):
                    request_id = request.get("id")
                result = self.handle_request(request)
                response = {"id": request_id, "ok": True, "result": result}
            except CommandError as exc:
                response = {
                    "id": request_id,
                    "ok": False,
                    "error": {"code": exc.code, "message": str(exc)},
                }
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                response = {
                    "id": request_id,
                    "ok": False,
                    "error": {"code": "invalid_json", "message": str(exc)},
                }
            except Exception as exc:
                response = {
                    "id": request_id,
                    "ok": False,
                    "error": {"code": "internal_error", "message": str(exc)},
                }

            self._socket.send_json(response)
            processed += 1
        return processed

    def handle_request(self, request: Any) -> dict[str, Any]:
        if not isinstance(request, dict):
            raise CommandError("invalid_request", "Request must be a JSON object")
        command = request.get("command")
        if command == "set_contact_parameters":
            return self._set_contact_parameters(request)
        if command == "set_body_properties":
            return self._set_body_properties(request)
        if command == "add_payload":
            return self._add_payload(request)
        if command == "apply_wrench":
            return self._schedule_wrench(request)
        if command == "set_joint_torque_limits":
            return self._set_joint_torque_limits(request)
        if command == "restore":
            return self._restore()
        raise CommandError("unknown_command", f"Unknown command: {command!r}")

    def apply_wrenches(self) -> None:
        """Apply all unexpired remote wrenches for the next MuJoCo step."""
        now = float(self.data.time)
        active = [item for item in self._active_wrenches if item.expires_at > now]

        if any(item.frame == "body" for item in active):
            # mj_step integrates qpos after computing xmat. Refresh kinematics
            # here so local wrenches follow the current pose, without a lag.
            mujoco.mj_kinematics(self.model, self.data)
        if self._managed_body_ids:
            self.data.xfrc_applied[list(self._managed_body_ids)] = 0.0
        for item in active:
            self.data.xfrc_applied[item.body_id] += self._world_wrench(item)

        self._active_wrenches = active
        self._managed_body_ids = {item.body_id for item in active}

    def _world_wrench(self, item: _ActiveWrench) -> np.ndarray:
        if item.frame == "world":
            return item.wrench
        rotation = self.data.xmat[item.body_id].reshape(3, 3)
        return np.concatenate((rotation @ item.wrench[:3], rotation @ item.wrench[3:]))

    def update_visuals(self, scene: mujoco.MjvScene) -> None:
        """Rebuild the remote-control overlay in a dedicated viewer user scene."""
        scene.ngeom = 0
        active = [item for item in self._active_wrenches if item.expires_at > self.data.time]
        if not self._payloads and not active:
            return
        # Refresh body poses after integration without changing simulation state
        # or the forces applied by the simulation thread.
        mujoco.mj_kinematics(self.model, self.data)
        identity = np.eye(3).ravel()
        for body_id, payload in self._payloads.items():
            if scene.ngeom >= scene.maxgeom:
                return
            position = self.data.xpos[body_id] + (
                self.data.xmat[body_id].reshape(3, 3) @ payload.position
            )
            # A 1 kg payload has a 5 cm radius; volume scales linearly with mass.
            radius = 0.05 * np.cbrt(payload.mass)
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                geom, mujoco.mjtGeom.mjGEOM_SPHERE, np.full(3, radius),
                position, identity, np.array([1.0, 0.65, 0.1, 0.65], dtype=np.float32),
            )
            geom.label = f"{payload.mass:g} kg"
            scene.ngeom += 1

        forces: dict[int, np.ndarray] = {}
        for item in active:
            forces.setdefault(item.body_id, np.zeros(3))[:] += self._world_wrench(item)[:3]
        for body_id, force in forces.items():
            if np.linalg.norm(force) < 1e-12:
                continue
            if scene.ngeom >= scene.maxgeom:
                return
            start = self.data.xipos[body_id].copy()
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), start, identity,
                np.array([1.0, 0.15, 0.1, 1.0], dtype=np.float32),
            )
            # MuJoCo's arrow primitive supplies the cylinder shaft and cone tip.
            # Display 100 N as a 0.5 m arrow, starting at the combined body COM.
            mujoco.mjv_connector(
                geom, mujoco.mjtGeom.mjGEOM_ARROW, 0.0333, start, start + 0.005 * force,
            )
            geom.label = f"F={np.linalg.norm(force):g} N"
            scene.ngeom += 1

    def _set_contact_parameters(self, request: dict[str, Any]) -> dict[str, Any]:
        names, body_ids = self._resolve_bodies(request)

        parameters = request.get("parameters")
        if not isinstance(parameters, dict) or not parameters:
            raise CommandError("invalid_parameter", "'parameters' must be a non-empty object")

        validated = self._validate_contact_parameters(parameters)
        geom_ids = [
            geom_id
            for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) in body_ids
        ]
        if not geom_ids:
            raise CommandError("no_geoms", "None of the requested bodies has a directly attached geom")

        # Validation and name resolution are deliberately completed before mutation.
        for name, value in validated.items():
            if name in self._VECTOR_PARAMETERS:
                array_name = self._VECTOR_PARAMETERS[name][0]
            else:
                array_name = self._SCALAR_PARAMETERS[name][0]
            getattr(self.model, array_name)[geom_ids] = value

        return {
            "bodies": names,
            "geoms": [self.model.geom(geom_id).name for geom_id in geom_ids],
            "parameters": validated,
        }

    def _set_body_properties(self, request: dict[str, Any]) -> dict[str, Any]:
        names, body_ids = self._resolve_bodies(request)

        properties = request.get("properties")
        if not isinstance(properties, dict) or not properties:
            raise CommandError("invalid_parameter", "'properties' must be a non-empty object")
        unknown = sorted(properties.keys() - {"mass", "com"})
        if unknown:
            raise CommandError("invalid_parameter", f"Unknown body properties: {unknown}")

        validated: dict[str, Any] = {}
        if "mass" in properties:
            mass = self._finite_number(properties["mass"], "mass")
            if mass <= 0.0:
                raise CommandError("invalid_parameter", "'mass' must be positive")
            validated["mass"] = mass
        if "com" in properties:
            validated["com"] = self._vector(properties["com"], "com", 3)

        if 0 in body_ids:
            raise CommandError("invalid_parameter", "The world body cannot be modified")

        # Resolve and validate the entire request before changing the model.
        if "mass" in validated:
            self.model.body_mass[body_ids] = validated["mass"]
        if "com" in validated:
            self.model.body_ipos[body_ids] = validated["com"]
        for body_id in body_ids:
            self._payloads.pop(body_id, None)

        # Mass and inertial-frame changes affect derived model constants such as
        # subtree masses and inverse weights.
        self._update_model_constants()
        return {"bodies": names, "properties": validated}

    def _add_payload(self, request: dict[str, Any]) -> dict[str, Any]:
        names, body_ids = self._resolve_bodies(request, single=True)
        mass = self._finite_number(request.get("mass"), "mass")
        if mass <= 0.0:
            raise CommandError("invalid_parameter", "Payload 'mass' must be positive")
        position = self._vector(request.get("position"), "position", 3)
        # Compute all selected payloads before mutating any body.
        updates = [self._payload_properties(body_id, mass, position) for body_id in body_ids]
        properties = {}
        for name, body_id, (total_mass, com, inertia, quaternion) in zip(names, body_ids, updates):
            self.model.body_mass[body_id] = total_mass
            self.model.body_ipos[body_id] = com
            self.model.body_inertia[body_id] = inertia
            self.model.body_iquat[body_id] = quaternion
            self._payloads[body_id] = _PointPayload(mass, np.asarray(position))
            properties[name] = {"mass": total_mass, "com": com.tolist()}
        self._update_model_constants()
        result = {"payload": {"mass": mass, "position": position}}
        if len(names) == 1:
            result.update(body=names[0], properties=properties[names[0]])
        else:
            result.update(bodies=names, properties=properties)
        return result

    def _payload_properties(
        self, body_id: int, mass: float, position: list[float],
    ) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
        # Always combine with the startup body properties so a new request
        # replaces the previous payload, including its COM and inertia changes.
        original = self._original_model_values
        body_mass = float(original["body_mass"][body_id])
        body_com = original["body_ipos"][body_id]
        if body_id == 0 or body_mass <= 0.0:
            raise CommandError("invalid_parameter", "Payload requires a body with positive mass")

        # Express the original principal inertia in the body's local frame.
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, original["body_iquat"][body_id])
        rotation = rotation.reshape(3, 3)
        inertia = rotation @ np.diag(original["body_inertia"][body_id]) @ rotation.T
        total_mass = body_mass + mass
        with np.errstate(over="ignore", invalid="ignore"):
            com = (body_mass / total_mass) * body_com + (
                mass / total_mass
            ) * np.asarray(position)
            offset = np.asarray(position) - body_com
            # Parallel-axis theorem about the combined COM; the point payload
            # has no intrinsic inertia. The reduced mass includes both shifts.
            inertia += (body_mass * (mass / total_mass)) * (
                np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset)
            )
        if not (math.isfinite(total_mass) and np.isfinite(com).all()
                and np.isfinite(inertia).all()):
            raise CommandError("invalid_parameter", "Payload produces non-finite body properties")

        principal_inertia, axes = np.linalg.eigh(inertia)
        if np.any(principal_inertia <= 0.0):
            raise CommandError("invalid_parameter", "Payload requires positive body inertia")
        if np.linalg.det(axes) < 0.0:
            axes[:, 0] *= -1.0
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, axes.ravel())

        return total_mass, com, principal_inertia, quaternion

    def _update_model_constants(self) -> None:
        # mj_setConst resets qpos to the model's initial pose and overwrites
        # derived data. Use reusable scratch data to leave the running robot
        # untouched; the next simulation step recomputes its derived quantities.
        if self._constant_data is None:
            self._constant_data = mujoco.MjData(self.model)
        mujoco.mj_setConst(self.model, self._constant_data)

    def _restore(self) -> dict[str, Any]:
        active_wrenches = len(self._active_wrenches)
        for name, original in self._original_model_values.items():
            getattr(self.model, name)[:] = original

        self._update_model_constants()
        self.data.xfrc_applied[:] = 0.0
        self.data.qfrc_applied[:] = 0.0
        self._active_wrenches.clear()
        self._managed_body_ids.clear()
        self._payloads.clear()
        return {
            "restored": True,
            "cleared_wrenches": active_wrenches,
        }

    def _validate_contact_parameters(self, parameters: dict[str, Any]) -> dict[str, Any]:
        allowed = self._VECTOR_PARAMETERS.keys() | self._SCALAR_PARAMETERS.keys()
        unknown = sorted(parameters.keys() - allowed)
        if unknown:
            raise CommandError("invalid_parameter", f"Unknown contact parameters: {unknown}")

        validated: dict[str, Any] = {}
        for name, (_, size) in self._VECTOR_PARAMETERS.items():
            if name not in parameters:
                continue
            value = parameters[name]
            if not isinstance(value, list) or len(value) != size:
                raise CommandError("invalid_parameter", f"'{name}' must contain {size} numbers")
            result = [self._finite_number(item, name) for item in value]
            if name == "friction" and any(item < 0.0 for item in result):
                raise CommandError("invalid_parameter", "'friction' values must be non-negative")
            validated[name] = result

        for name, (_, value_type) in self._SCALAR_PARAMETERS.items():
            if name not in parameters:
                continue
            value = parameters[name]
            number = self._finite_number(value, name)
            if value_type is int:
                if isinstance(value, bool) or not number.is_integer():
                    raise CommandError("invalid_parameter", f"'{name}' must be an integer")
                result: int | float = int(number)
            else:
                result = number
            if name == "condim" and result not in (1, 3, 4, 6):
                raise CommandError("invalid_parameter", "'condim' must be one of 1, 3, 4, or 6")
            if name == "priority" and result < 0:
                raise CommandError("invalid_parameter", "'priority' must be non-negative")
            if name == "margin" and result < 0.0:
                raise CommandError("invalid_parameter", "'margin' must be non-negative")
            validated[name] = result
        return validated

    def _schedule_wrench(self, request: dict[str, Any]) -> dict[str, Any]:
        names, body_ids = self._resolve_bodies(request, single=True)
        force = self._vector(request.get("force"), "force", 3)
        torque = self._vector(request.get("torque"), "torque", 3)
        frame = request.get("frame", "world")
        if frame not in ("world", "body"):
            raise CommandError("invalid_parameter", "'frame' must be 'world' or 'body'")
        duration = self._finite_number(request.get("duration"), "duration")
        if duration <= 0.0:
            raise CommandError("invalid_parameter", "'duration' must be positive")

        expires_at = float(self.data.time) + duration
        for body_id in body_ids:
            self._active_wrenches.append(
                _ActiveWrench(body_id, np.asarray(force + torque, dtype=float), expires_at, frame)
            )
        self._managed_body_ids.update(body_ids)
        if len(names) == 1:
            return {"body": names[0], "expires_at": expires_at}
        return {"bodies": names, "expires_at": expires_at}

    def _set_joint_torque_limits(self, request: dict[str, Any]) -> dict[str, Any]:
        names, joint_ids = self._resolve_names(
            request.get("joints"), "joint", request.get("regex", True),
        )
        limit = self._finite_number(request.get("limit"), "limit")
        if limit < 0.0:
            raise CommandError("invalid_parameter", "'limit' must be non-negative")
        if any(self.model.jnt_type[joint_id] not in (
            mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE,
        ) for joint_id in joint_ids):
            raise CommandError("invalid_parameter", "Torque limits require hinge or slide joints")
        # Clamp the net generalized actuator force at the joint, after gearing
        # and summing actuators. The bridge does not overwrite these fields.
        self.model.jnt_actfrcrange[joint_ids] = [-limit, limit]
        self.model.jnt_actfrclimited[joint_ids] = True
        return {"joints": names, "limit": limit}

    def _resolve_bodies(
        self, request: dict[str, Any], single: bool = False,
    ) -> tuple[list[str], list[int]]:
        if single and "body" in request:
            if "bodies" in request:
                raise CommandError("invalid_parameter", "Specify either 'body' or 'bodies'")
            body = request["body"]
            if not isinstance(body, str) or not body:
                raise CommandError("invalid_parameter", "'body' must be a non-empty string")
            selectors = [body]
        else:
            selectors = request.get("bodies")
        return self._resolve_names(selectors, "body", request.get("regex", False))

    def _resolve_names(
        self, selectors: Any, kind: str, regex: Any,
    ) -> tuple[list[str], list[int]]:
        if not isinstance(regex, bool):
            raise CommandError("invalid_parameter", "'regex' must be a boolean")
        if not isinstance(selectors, list) or not selectors or any(
            not isinstance(value, str) or not value for value in selectors
        ):
            raise CommandError("invalid_parameter", f"{kind} selectors must be a non-empty list of strings")
        get_object = getattr(self.model, kind)
        count = self.model.nbody if kind == "body" else self.model.njnt
        selected: dict[int, str] = {}
        for selector in selectors:
            if regex:
                try:
                    pattern = re.compile(selector)
                except re.error as exc:
                    raise CommandError("invalid_regex", f"Invalid regex {selector!r}: {exc}") from exc
                matches = [
                    (object_id, get_object(object_id).name) for object_id in range(count)
                    if get_object(object_id).name and pattern.fullmatch(get_object(object_id).name)
                ]
                if not matches:
                    raise CommandError("no_matches", f"No {kind} matches regex {selector!r}")
                selected.update(matches)
            else:
                try:
                    obj = get_object(selector)
                except KeyError as exc:
                    raise CommandError(f"unknown_{kind}", f"Unknown {kind}: {selector!r}") from exc
                selected[int(obj.id)] = selector
        return list(selected.values()), list(selected)

    def _vector(self, value: Any, name: str, size: int) -> list[float]:
        if not isinstance(value, list) or len(value) != size:
            raise CommandError("invalid_parameter", f"'{name}' must contain {size} numbers")
        return [self._finite_number(item, name) for item in value]

    @staticmethod
    def _finite_number(value: Any, name: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise CommandError("invalid_parameter", f"'{name}' must contain finite numbers")
        result = float(value)
        if not math.isfinite(result):
            raise CommandError("invalid_parameter", f"'{name}' must contain finite numbers")
        return result
