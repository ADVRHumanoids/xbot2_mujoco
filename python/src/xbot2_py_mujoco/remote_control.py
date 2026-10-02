"""ZeroMQ/JSON remote control for a running MuJoCo simulation."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
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
        self.max_requests_per_step = max_requests_per_step
        self._active_wrenches: list[_ActiveWrench] = []
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
        if command == "apply_wrench":
            return self._schedule_wrench(request)
        if command == "restore":
            return self._restore()
        raise CommandError("unknown_command", f"Unknown command: {command!r}")

    def apply_wrenches(self) -> None:
        """Apply all unexpired remote wrenches for the next MuJoCo step."""
        now = float(self.data.time)
        active = [item for item in self._active_wrenches if item.expires_at > now]

        if self._managed_body_ids:
            self.data.xfrc_applied[list(self._managed_body_ids)] = 0.0
        for item in active:
            self.data.xfrc_applied[item.body_id] += item.wrench

        self._active_wrenches = active
        self._managed_body_ids = {item.body_id for item in active}

    def _set_contact_parameters(self, request: dict[str, Any]) -> dict[str, Any]:
        bodies = request.get("bodies")
        if not isinstance(bodies, list) or not bodies:
            raise CommandError("invalid_parameter", "'bodies' must be a non-empty list")
        if any(not isinstance(name, str) or not name for name in bodies):
            raise CommandError("invalid_parameter", "Every body name must be a non-empty string")

        parameters = request.get("parameters")
        if not isinstance(parameters, dict) or not parameters:
            raise CommandError("invalid_parameter", "'parameters' must be a non-empty object")

        validated = self._validate_contact_parameters(parameters)
        body_ids = [self._body_id(name) for name in dict.fromkeys(bodies)]
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
            "bodies": list(dict.fromkeys(bodies)),
            "geoms": [self.model.geom(geom_id).name for geom_id in geom_ids],
            "parameters": validated,
        }

    def _set_body_properties(self, request: dict[str, Any]) -> dict[str, Any]:
        bodies = request.get("bodies")
        if not isinstance(bodies, list) or not bodies:
            raise CommandError("invalid_parameter", "'bodies' must be a non-empty list")
        if any(not isinstance(name, str) or not name for name in bodies):
            raise CommandError("invalid_parameter", "Every body name must be a non-empty string")

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

        names = list(dict.fromkeys(bodies))
        body_ids = [self._body_id(name) for name in names]
        if 0 in body_ids:
            raise CommandError("invalid_parameter", "The world body cannot be modified")

        # Resolve and validate the entire request before changing the model.
        if "mass" in validated:
            self.model.body_mass[body_ids] = validated["mass"]
        if "com" in validated:
            self.model.body_ipos[body_ids] = validated["com"]

        # Mass and inertial-frame changes affect derived model constants such as
        # subtree masses and inverse weights.
        mujoco.mj_setConst(self.model, self.data)
        return {"bodies": names, "properties": validated}

    def _restore(self) -> dict[str, Any]:
        active_wrenches = len(self._active_wrenches)
        for name, original in self._original_model_values.items():
            getattr(self.model, name)[:] = original

        mujoco.mj_setConst(self.model, self.data)
        self.data.xfrc_applied[:] = 0.0
        self.data.qfrc_applied[:] = 0.0
        self._active_wrenches.clear()
        self._managed_body_ids.clear()
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
        body = request.get("body")
        if not isinstance(body, str) or not body:
            raise CommandError("invalid_parameter", "'body' must be a non-empty string")
        body_id = self._body_id(body)
        force = self._vector(request.get("force"), "force", 3)
        torque = self._vector(request.get("torque"), "torque", 3)
        duration = self._finite_number(request.get("duration"), "duration")
        if duration <= 0.0:
            raise CommandError("invalid_parameter", "'duration' must be positive")

        expires_at = float(self.data.time) + duration
        self._active_wrenches.append(
            _ActiveWrench(body_id, np.asarray(force + torque, dtype=float), expires_at)
        )
        self._managed_body_ids.add(body_id)
        return {"body": body, "expires_at": expires_at}

    def _body_id(self, name: str) -> int:
        try:
            return int(self.model.body(name).id)
        except KeyError as exc:
            raise CommandError("unknown_body", f"Unknown body: {name!r}") from exc

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
