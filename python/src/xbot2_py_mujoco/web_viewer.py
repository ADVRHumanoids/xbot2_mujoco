"""Studio web adapter; imported only when web viewing is requested."""
from contextlib import nullcontext
from copy import copy
from dataclasses import dataclass

import mujoco
from mujoco.studio import app, launch_web, messages, plugin, sim
from mujoco.studio.viewer_protocol import ViewerConfig
from mujoco.studio.dear_imgui import dear_imgui as imgui


@dataclass(frozen=True)
class _OverlaySnapshot(messages.Snapshot):
    geoms: tuple


class _App(app.App):
    def __init__(self, key_callback):
        super().__init__()
        self._key_callback = key_callback

    def handle_keyboard_events(self):
        # Retain the wrapper's key semantics instead of Studio's history/speed
        # shortcuts. Camera and visualization shortcuts remain Studio's own.
        if not imgui.GetIO().WantCaptureKeyboard:
            keys = [(imgui.Key.Space, 32), (imgui.Key.RightArrow, 262),
                    (imgui.Key.R, ord('R')), (imgui.Key.Equal, ord('=')),
                    (imgui.Key.Minus, ord('-')), (imgui.Key.KeypadAdd, 334),
                    (imgui.Key.KeypadSubtract, 333)]
            for key, code in keys:
                if imgui.IsKeyPressed(key, False):
                    self._key_callback(code)
                    return
        super().handle_keyboard_events()

    @messages.handler
    def _on_overlays(self, snapshot: _OverlaySnapshot):
        self.viewer.extra_geoms = list(snapshot.geoms)


class _Controls(plugin.SimPlugin):
    def __init__(self, simulator):
        self.simulator = simulator

    @messages.handler
    def _on_controls(self, snapshot: messages.StepControlSnapshot):
        self.simulator.set_paused(snapshot.pause_state != sim.PauseState.UNPAUSED)
        self.simulator.set_target_rtf(snapshot.speed / 100.0)
        return True

    @messages.handler
    def _on_pause(self, event: messages.RequestPauseEvent):
        self.simulator.set_paused(event.pause_state != sim.PauseState.UNPAUSED)
        return True

    @messages.handler
    def _on_advance(self, event: messages.SingleStepEvent):
        self.simulator.advance(0.01666)
        return True

    @messages.handler(priority=messages.Priority.CRITICAL)
    def _on_load(self, event: messages.LoadModelEvent):
        # The bridge and scanners retain this model: replacing it through the
        # Studio file picker would invalidate their indices and data pointers.
        return True


class WebViewerHandle:
    """The passive viewer surface used by SimulatorWrapper, backed by Studio."""

    def __init__(self, model, data, key_callback, simulator, *, port=0, open_browser=False):
        self._model = model
        self._data = data
        self._simulator = simulator
        self.user_scn = mujoco.MjvScene(model, maxgeom=10000)
        self._handle = launch_web.launch(
            ViewerConfig(gfx='web', title='XBot2 MuJoCo', http_port=port,
                         open_browser=open_browser),
            plugins=[_App(key_callback), _Controls(simulator)],
        )
        try:
            self._handle.set_model(model, data=data)
        except BaseException:
            self._handle.close()
            raise

    def lock(self):
        # Only the simulation thread touches this scene. Copies cross the
        # Studio endpoint, so the viewer never reads it concurrently.
        return nullcontext()

    def sync(self, *, state_only=True):
        self._handle.sync()
        self._handle.send_to_viewer(_OverlaySnapshot(tuple(
            copy(self.user_scn.geoms[i]) for i in range(self.user_scn.ngeom)
        )))
        self._handle.send_to_viewer(messages.StepControlSnapshot(
            pause_state=(sim.PauseState.NORMAL_PAUSED if self._simulator.paused
                         else sim.PauseState.UNPAUSED),
            speed=100.0 * self._simulator.target_rtf,
        ))

    def is_running(self):
        # Closing a browser tab leaves the server available for reconnection.
        return self._handle.is_running()

    def close(self):
        self._handle.close()
