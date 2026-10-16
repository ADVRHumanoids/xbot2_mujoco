"""Exercise the public CLI over a real ZeroMQ connection."""

import json
import subprocess
import sys
import threading
import uuid

import pytest
import zmq

from xbot2_py_mujoco.mjctl import main


@pytest.fixture
def peer():
    socket = zmq.Context.instance().socket(zmq.REP)
    socket.setsockopt(zmq.RCVTIMEO, 2000)
    socket.setsockopt(zmq.LINGER, 0)
    endpoint = f"inproc://mjctl-test-{uuid.uuid4()}"
    socket.bind(endpoint)
    requests = []
    errors = []
    threads = []

    def reply(response):
        def serve():
            try:
                requests.append(socket.recv_json())
                socket.send_json(response)
            except Exception as exc:
                errors.append(exc)
        thread = threading.Thread(target=serve)
        threads.append(thread)
        thread.start()

    yield endpoint, requests, reply
    for thread in threads:
        thread.join(timeout=3)
    socket.close()
    assert not errors


@pytest.mark.parametrize("arguments, expected", [
    (["friction", ".*_foot", "--regex", "--values", ".8", ".005", ".001"],
     {"command": "set_contact_parameters", "bodies": [".*_foot"], "regex": True,
      "parameters": {"friction": [.8, .005, .001]}}),
    (["wrench", "base", "--force", "100", "0", "0", "--torque", "0", "0", "5",
      "--duration", ".25", "--frame", "body"],
     {"command": "apply_wrench", "body": "base", "force": [100, 0, 0],
      "torque": [0, 0, 5], "duration": .25, "frame": "body", "regex": False}),
    (["body", "left", "right", "--mass", "25", "--com", ".02", "0", ".08"],
     {"command": "set_body_properties", "bodies": ["left", "right"], "regex": False,
      "properties": {"mass": 25, "com": [.02, 0, .08]}}),
    (["payload", "base", "--mass", "5", "--position", ".2", "0", ".1"],
     {"command": "add_payload", "body": "base", "mass": 5,
      "position": [.2, 0, .1], "regex": False}),
    (["torque-limits", ".*_knee", "--limit", "40"],
     {"command": "set_joint_torque_limits", "joints": [".*_knee"], "limit": 40, "regex": True}),
    (["torque-limits", "knee", "--exact", "--limit", "0"],
     {"command": "set_joint_torque_limits", "joints": ["knee"], "limit": 0, "regex": False}),
    (["actuator-delay", "--seconds", ".02"],
     {"command": "set_actuator_delay", "delay": .02}),
    (["actuator-delay", "--seconds", "0"],
     {"command": "set_actuator_delay", "delay": 0}),
    (["velocity-noise", ".*_knee", "--stddev", ".05"],
     {"command": "set_velocity_feedback_noise", "joints": [".*_knee"],
      "stddev": .05, "regex": True}),
    (["velocity-noise", "left_knee", "--exact", "--stddev", "0"],
     {"command": "set_velocity_feedback_noise", "joints": ["left_knee"],
      "stddev": 0, "regex": False}),
    (["restore"], {"command": "restore"}),
])
def test_cli_sends_command_and_prints_json(peer, capsys, arguments, expected):
    endpoint, requests, reply = peer
    response = {"ok": True, "result": {"accepted": True}}
    reply(response)
    assert main(["--endpoint", endpoint, *arguments]) == 0
    assert requests == [expected]
    captured = capsys.readouterr()
    assert json.loads(captured.out) == response
    assert not captured.err


@pytest.mark.parametrize("response", [
    {"ok": False, "error": {"code": "unknown_body", "message": "Unknown body: missing"}},
    {"unexpected": "reply"},
    ["unexpected"],
])
def test_failed_reply_returns_nonzero(peer, capsys, response):
    endpoint, _, reply = peer
    reply(response)
    assert main(["--endpoint", endpoint, "restore"]) == 1
    assert json.loads(capsys.readouterr().out) == response


def test_absent_server_times_out(capsys):
    endpoint = f"inproc://mjctl-absent-{uuid.uuid4()}"
    assert main(["--endpoint", endpoint, "--timeout", ".01", "restore"]) == 1
    captured = capsys.readouterr()
    assert not captured.out
    assert "timed out" in captured.err
    assert "may have taken effect" in captured.err


@pytest.mark.parametrize("arguments", [
    [], ["body", "base"], ["--timeout", "0", "restore"],
    ["--timeout", "nan", "restore"], ["--timeout", "inf", "restore"],
    ["wrench", "base", "--force", "1", "0", "0", "--torque", "0", "0", "0", "--duration", "-1"],
    ["actuator-delay"], ["actuator-delay", "--seconds", "-1"],
    ["actuator-delay", "--seconds", "nan"], ["actuator-delay", "--seconds", "inf"],
    ["velocity-noise"], ["velocity-noise", "knee", "--stddev", "-1"],
    ["velocity-noise", "knee", "--stddev", "nan"],
    ["velocity-noise", "knee", "--stddev", "inf"],
])
def test_invalid_usage(arguments):
    with pytest.raises(SystemExit) as exc:
        main(arguments)
    assert exc.value.code == 2


@pytest.mark.parametrize("command", [None, "friction", "wrench", "body", "payload", "torque-limits", "actuator-delay", "velocity-noise", "restore"])
def test_help_works_without_server(command, capsys):
    with pytest.raises(SystemExit) as exc:
        main(([command] if command else []) + ["--help"])
    assert exc.value.code == 0
    help_text = capsys.readouterr().out
    assert "mjctl" in help_text
    assert "Example" in help_text


def test_module_entrypoint():
    result = subprocess.run([sys.executable, "-m", "xbot2_py_mujoco.mjctl", "--help"],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0
    assert "mjctl" in result.stdout
