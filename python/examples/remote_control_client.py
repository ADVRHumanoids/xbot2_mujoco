#!/usr/bin/env python3
"""Small client for the xbot2_py_mujoco remote-control endpoint."""

import argparse
import json

import zmq


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5555")
    subparsers = parser.add_subparsers(dest="command", required=True)

    contact = subparsers.add_parser("friction")
    contact.add_argument("bodies", nargs="+")
    contact.add_argument("--values", nargs=3, type=float, required=True,
                         metavar=("SLIDING", "TORSIONAL", "ROLLING"))

    wrench = subparsers.add_parser("wrench")
    wrench.add_argument("body")
    wrench.add_argument("--force", nargs=3, type=float, required=True)
    wrench.add_argument("--torque", nargs=3, type=float, required=True)
    wrench.add_argument("--duration", type=float, required=True)
    wrench.add_argument("--frame", choices=("world", "body"), default="world")

    body = subparsers.add_parser("body")
    body.add_argument("bodies", nargs="+")
    body.add_argument("--mass", type=float)
    body.add_argument("--com", nargs=3, type=float, metavar=("X", "Y", "Z"))

    payload = subparsers.add_parser("payload")
    payload.add_argument("body")
    payload.add_argument("--mass", type=float, required=True)
    payload.add_argument("--position", nargs=3, type=float, required=True,
                         metavar=("X", "Y", "Z"))

    for command_parser in (contact, wrench, body, payload):
        command_parser.add_argument("--regex", action="store_true",
                                    help="Match body names with full-name regular expressions")

    limits = subparsers.add_parser("torque-limits")
    limits.add_argument("joints", nargs="+", help="Joint regex patterns (full-name match)")
    limits.add_argument("--limit", type=float, required=True,
                        help="Symmetric torque limit in Nm (N for slide joints); zero disables output")
    limits.add_argument("--exact", dest="regex", action="store_false", default=True,
                        help="Treat joint selectors as exact names instead of regex patterns")

    subparsers.add_parser("restore")

    args = parser.parse_args()
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
    else:
        request = {"command": "restore"}

    if hasattr(args, "regex"):
        request["regex"] = args.regex

    socket = zmq.Context.instance().socket(zmq.REQ)
    socket.connect(args.endpoint)
    socket.send_json(request)
    print(json.dumps(socket.recv_json(), indent=2))
    socket.close()


if __name__ == "__main__":
    main()
