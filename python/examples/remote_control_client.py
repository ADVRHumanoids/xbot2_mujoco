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

    body = subparsers.add_parser("body")
    body.add_argument("bodies", nargs="+")
    body.add_argument("--mass", type=float)
    body.add_argument("--com", nargs=3, type=float, metavar=("X", "Y", "Z"))

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
    else:
        request = {"command": "restore"}

    socket = zmq.Context.instance().socket(zmq.REQ)
    socket.connect(args.endpoint)
    socket.send_json(request)
    print(json.dumps(socket.recv_json(), indent=2))
    socket.close()


if __name__ == "__main__":
    main()
