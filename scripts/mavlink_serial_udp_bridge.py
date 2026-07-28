#!/usr/bin/env python3

import argparse
import select
import socket

import serial


def parse_endpoint(value):
    host, port = value.rsplit(":", 1)
    return host, int(port)


def main():
    parser = argparse.ArgumentParser(
        description="Forward a serial MAVLink byte stream to multiple UDP listeners."
    )
    parser.add_argument("--device", required=True)
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--bind-port", type=int, default=14570)
    parser.add_argument(
        "--endpoint",
        action="append",
        type=parse_endpoint,
        dest="endpoints",
        required=True,
    )
    args = parser.parse_args()

    serial_port = serial.Serial(
        args.device,
        args.baud,
        timeout=0,
        write_timeout=0,
        exclusive=True,
    )
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind(("0.0.0.0", args.bind_port))

    print(
        f"Serial {args.device} at {args.baud} baud; "
        f"UDP return port {args.bind_port}; endpoints {args.endpoints}",
        flush=True,
    )

    while True:
        readable, _, _ = select.select([serial_port.fileno(), udp_socket], [], [])

        if serial_port.fileno() in readable:
            data = serial_port.read(65536)

            if data:
                for endpoint in args.endpoints:
                    udp_socket.sendto(data, endpoint)

        if udp_socket in readable:
            data, _ = udp_socket.recvfrom(65536)

            if data:
                serial_port.write(data)


if __name__ == "__main__":
    main()
