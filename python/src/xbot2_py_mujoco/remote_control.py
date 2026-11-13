"""ZeroMQ/JSON transport for the simulator's shared control API."""

import json
import zmq

from .simulation_control import CommandError, SimulationControl


class RemoteControlServer:
    """Non-blocking REP adapter; all mutations live in SimulationControl."""

    def __init__(self, model, data, endpoint, max_requests_per_step=16,
                 actuator_bridge=None, *, control=None):
        if not endpoint:
            raise ValueError('Remote-control endpoint must not be empty')
        if max_requests_per_step <= 0:
            raise ValueError('max_requests_per_step must be positive')
        self.control = control if control is not None else SimulationControl(model, data, actuator_bridge)
        self._owns_control = control is None
        self.max_requests_per_step = max_requests_per_step
        self._socket = zmq.Context.instance().socket(zmq.REP)
        self._socket.setsockopt(zmq.LINGER, 0)
        try:
            self._socket.bind(endpoint)
        except Exception:
            self._socket.close()
            raise
        self.endpoint = self._socket.getsockopt_string(zmq.LAST_ENDPOINT)

    def __getattr__(self, name):
        return getattr(self.control, name)

    def handle_request(self, request):
        return self.control.handle_request(request)

    def clear_wrenches(self, body_name=None):
        self.control.clear_wrenches(body_name)

    def apply_wrenches(self):
        self.control.apply_wrenches()

    def update_visuals(self, scene):
        self.control.update_visuals(scene)

    @property
    def actuator_bridge(self):
        return self.control.actuator_bridge

    @actuator_bridge.setter
    def actuator_bridge(self, value):
        self.control.actuator_bridge = value

    def close(self):
        if self._socket is not None:
            if self._owns_control:
                self.control.clear_wrenches()
            self._socket.close()
            self._socket = None

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
