#!/usr/bin/env python3
"""Command-line client for the simulator's ZeroMQ/JSON remote-control API."""

import argparse
import json
import math
import sys

import zmq


def _positive_seconds(value: str) -> float:
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("must be a finite, positive number of seconds")
    return seconds


def _nonnegative_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be a finite, non-negative number")
    return number


def build_parser() -> argparse.ArgumentParser:
    """Build CLI help and argument parsing without opening a connection."""
    parser = argparse.ArgumentParser(
        prog="mjctl",
        description="Change properties and apply disturbances to a running MuJoCo simulation.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Enable the server with:
  python -m xbot2_py_mujoco.simulate ... --remote-control-endpoint tcp://127.0.0.1:5555

Examples:
  mjctl friction '.*_foot' --regex --values 0.8 0.005 0.001
  mjctl wrench base_link --force 100 0 0 --torque 0 0 0 --duration 0.25
  mjctl body base_link --mass 25 --com 0.02 0 0.08
  mjctl payload base_link --mass 5 --position 0.2 0 0.1
  mjctl torque-limits '.*_knee' --limit 40
  mjctl actuator-delay --seconds 0.02
  mjctl velocity-noise '.*_knee' --stddev 0.05
  mjctl restore

Use 'mjctl COMMAND --help' for details. Global options go before COMMAND.
Replies are JSON. Exit codes: 0 success, 1 server/transport failure,
2 invalid CLI usage, 130 interrupted. A timeout does not undo a sent command.
""",
    )
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5555",
                        help="Server ZeroMQ endpoint (default: %(default)s)")
    parser.add_argument("--timeout", type=_positive_seconds, default=5.0, metavar="SECONDS",
                        help="Wall-clock wait per send/reply (default: %(default)s seconds)")
    subparsers = parser.add_subparsers(dest="command", required=True, metavar="COMMAND")

    def command(name, summary, details, example):
        return subparsers.add_parser(name, help=summary, description=details,
                                    epilog=f"Example: {example}")

    contact = command("friction", "Set contact friction on selected bodies",
                      "Set friction on all geoms directly attached to selected bodies; "
                      "descendant bodies are excluded.",
                      "mjctl friction '.*_foot' --regex --values 0.8 0.005 0.001")
    contact.add_argument("bodies", nargs="+", help="Exact body names, or full-name patterns with --regex")
    contact.add_argument("--values", nargs=3, type=float, required=True,
                         metavar=("SLIDING", "TORSIONAL", "ROLLING"),
                         help="Non-negative friction: sliding coefficient (dimensionless), "
                              "torsional and rolling coefficients (meters)")

    wrench = command("wrench", "Apply a timed force and torque",
                     "Apply a wrench at the body center of mass. Duration uses simulation "
                     "time; overlapping wrenches add together. Body-frame directions "
                     "follow the body's orientation every step.",
                     "mjctl wrench base_link --force 100 0 0 --torque 0 0 5 --duration 1 --frame body")
    wrench.add_argument("body", help="Exact body name, or full-name pattern with --regex")
    wrench.add_argument("--force", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"),
                        help="Force components in newtons")
    wrench.add_argument("--torque", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"),
                        help="Torque components in Nm; use 0 0 0 for a pure force")
    wrench.add_argument("--duration", type=_positive_seconds, required=True, metavar="SECONDS",
                        help="Positive duration in simulation seconds")
    wrench.add_argument("--frame", choices=("world", "body"), default="world",
                        help="Axes for both force and torque (default: %(default)s)")

    body = command("body", "Set body mass and/or center of mass",
                   "Set mass and/or local center of mass for selected bodies. At least one "
                   "option is required. Rotational inertia is unchanged. Removes payload "
                   "markers on those bodies.",
                   "mjctl body base_link --mass 25 --com 0.02 0 0.08")
    body.add_argument("bodies", nargs="+", help="Exact body names, or full-name patterns with --regex")
    body.add_argument("--mass", type=float, help="Positive total body mass in kg")
    body.add_argument("--com", nargs=3, type=float, metavar=("X", "Y", "Z"),
                      help="Center of mass in meters in the body's local frame")

    payload = command("payload", "Attach or replace a point payload",
                      "Add a point mass to the body's startup properties and recompute COM "
                      "and rotational inertia. Replaces the previous payload, including "
                      "after body-property edits; adds no collision geometry.",
                      "mjctl payload base_link --mass 5 --position 0.2 0 0.1")
    payload.add_argument("body", help="Exact body name, or full-name pattern with --regex")
    payload.add_argument("--mass", type=float, required=True, help="Positive payload mass in kg")
    payload.add_argument("--position", nargs=3, type=float, required=True,
                         metavar=("X", "Y", "Z"),
                         help="Payload position in meters in the body's local frame")

    for command_parser in (contact, wrench, body, payload):
        command_parser.add_argument("--regex", action="store_true",
                                    help="Use Python regex full-name matching; quote patterns in the shell. "
                                         "Each match receives the same values")

    limits = command("torque-limits", "Set symmetric joint actuator torque/force limits",
                     "Limit net actuator output after gearing and summing actuators. Supports "
                     "hinge and slide joints; zero disables output. Existing actuator limits "
                     "still apply. Contact forces and external wrenches are unaffected.",
                     "mjctl torque-limits '.*_knee' --limit 40")
    limits.add_argument("joints", nargs="+", help="Joint regex patterns (full-name match)")
    limits.add_argument("--limit", type=float, required=True,
                        help="Non-negative symmetric limit in Nm (N for slide joints); zero disables output")
    limits.add_argument("--exact", dest="regex", action="store_false", default=True,
                        help="Treat joint selectors as exact names instead of regex patterns")

    delay = command("actuator-delay", "Set XBot2 actuator command delay",
                    "Delay complete XBot2 commands in simulation time. Zero disables the "
                    "delay. Commands already pending keep their existing deadlines.",
                    "mjctl actuator-delay --seconds 0.02")
    delay.add_argument("--seconds", type=_nonnegative_float, required=True,
                       help="Finite, non-negative delay in simulation seconds")

    noise = command("velocity-noise", "Set joint velocity feedback noise",
                    "Add independent, zero-mean Gaussian noise to velocity feedback sent "
                    "to XBot2. The standard deviation uses rad/s for hinge joints and m/s "
                    "for slide joints. Zero disables noise for selected joints.",
                    "mjctl velocity-noise '.*_knee' --stddev 0.05")
    noise.add_argument("joints", nargs="+", help="Joint regex patterns (full-name match)")
    noise.add_argument("--stddev", type=_nonnegative_float, required=True,
                       help="Finite, non-negative velocity noise standard deviation")
    noise.add_argument("--exact", dest="regex", action="store_false", default=True,
                       help="Treat joint selectors as exact names instead of regex patterns")

    command("restore", "Restore startup properties and clear applied forces",
            "Restore all API-managed contact, body and joint limit properties to startup "
            "values, remove payloads, cancel wrenches, and clear Cartesian and generalized "
            "applied forces. Pose, velocity, controls and simulation time are preserved.",
            "mjctl restore")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Send one command, print its JSON reply, and return a shell exit status."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "friction":
        request = {
            "command": "set_contact_parameters",
            "bodies": args.bodies,
            "parameters": {"friction": args.values},
        }
    elif args.command == "wrench":
        request = {
            "command": "apply_wrench",
            "body": args.body,
            "force": args.force,
            "torque": args.torque,
            "duration": args.duration,
            "frame": args.frame,
        }
    elif args.command == "body":
        properties = {}
        if args.mass is not None:
            properties["mass"] = args.mass
        if args.com is not None:
            properties["com"] = args.com
        if not properties:
            parser.error("body requires --mass and/or --com")
        request = {
            "command": "set_body_properties",
            "bodies": args.bodies,
            "properties": properties,
        }
    elif args.command == "payload":
        request = {
            "command": "add_payload",
            "body": args.body,
            "mass": args.mass,
            "position": args.position,
        }
    elif args.command == "torque-limits":
        request = {
            "command": "set_joint_torque_limits",
            "joints": args.joints,
            "limit": args.limit,
        }
    elif args.command == "actuator-delay":
        request = {"command": "set_actuator_delay", "delay": args.seconds}
    elif args.command == "velocity-noise":
        request = {
            "command": "set_velocity_feedback_noise",
            "joints": args.joints,
            "stddev": args.stddev,
        }
    else:
        request = {"command": "restore"}

    if hasattr(args, "regex"):
        request["regex"] = args.regex

    socket = zmq.Context.instance().socket(zmq.REQ)
    try:
        socket.setsockopt(zmq.LINGER, 0)
        timeout_ms = max(1, math.ceil(min(args.timeout, 2_147_483.647) * 1000))
        socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        socket.connect(args.endpoint)
        socket.send_json(request)
        response = socket.recv_json()
        print(json.dumps(response, indent=2))
        if not isinstance(response, dict) or not isinstance(response.get("ok"), bool):
            print("mjctl: invalid server response: expected a JSON object with boolean 'ok'", file=sys.stderr)
            return 1
        return 0 if response["ok"] else 1
    except zmq.Again:
        print(f"mjctl: timed out communicating with {args.endpoint}; "
              "check that remote control is enabled. A sent command may have taken effect.", file=sys.stderr)
        return 1
    except (zmq.ZMQError, ValueError) as exc:
        print(f"mjctl: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        socket.close()


if __name__ == "__main__":
    raise SystemExit(main())
