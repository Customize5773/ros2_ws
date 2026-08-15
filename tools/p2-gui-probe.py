#!/usr/bin/env python3
"""P2 GUI UDP probe — send synthetic joystick commands to gui_bridge for
controlled experiments on roll/pitch spikes, gain calibration, etc.

Usage patterns:
  # Sustained yaw 100% for 10s (reproduce roll/pitch spike)
  python3 p2-gui-probe.py --mode sustained --axis yaw --value 100 --duration 10

  # Step response: surge 100% for 5s, then 0
  python3 p2-gui-probe.py --mode step --axis surge --value 100 --duration 5

  # Pulsed yaw (0.5s on/off, 10 cycles)
  python3 p2-gui-probe.py --mode pulsed --axis yaw --value 100 --on 0.5 --off 0.5 --cycles 10

  # Multi-axis step: surge+100, sway+50, heave-30
  python3 p2-gui-probe.py --mode multi --cmd surge=100,sway=50,heave=-30 --duration 5

All commands are sent as UDP JSON {name, value} to the gui_bridge cmd_port.
The probe arms first, waits for the arming to register, then sends the
configured command pattern.
"""

import argparse
import json
import socket
import time


CMD_PORT_DEFAULT = 14550


def send(sock, target, name, value):
    msg = json.dumps({'name': name, 'value': value}).encode('utf-8')
    sock.sendto(msg, target)


def arm_then_probe(args):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ('127.0.0.1', args.cmd_port)

    # Arm
    send(sock, target, 'arm', True)
    time.sleep(0.3)

    if args.mode == 'sustained':
        send(sock, target, args.axis, args.value)
        time.sleep(args.duration)
        send(sock, target, args.axis, 0.0)

    elif args.mode == 'step':
        send(sock, target, args.axis, args.value)
        time.sleep(args.duration)
        send(sock, target, args.axis, 0.0)

    elif args.mode == 'pulsed':
        for _ in range(args.cycles):
            send(sock, target, args.axis, args.value)
            time.sleep(args.on)
            send(sock, target, args.axis, 0.0)
            time.sleep(args.off)

    elif args.mode == 'multi':
        cmds = args.cmd.split(',')
        for cmd in cmds:
            name, val = cmd.split('=')
            send(sock, target, name, float(val))
        time.sleep(args.duration)
        for cmd in cmds:
            name, _ = cmd.split('=')
            send(sock, target, name, 0.0)

    elif args.mode == 'yaw_ramp':
        # Gradually increase yaw to find the threshold
        for pct in range(0, int(args.value) + 1, 10):
            send(sock, target, args.axis, float(pct))
            time.sleep(1.0)
        time.sleep(3.0)
        send(sock, target, args.axis, 0.0)

    # Disarm / stop after experiment
    send(sock, target, 'stop', None)
    time.sleep(0.2)
    sock.close()


def main():
    ap = argparse.ArgumentParser(description='P2 GUI UDP probe')
    ap.add_argument('--cmd-port', type=int, default=CMD_PORT_DEFAULT)
    ap.add_argument('--mode', choices=['sustained', 'step', 'pulsed', 'multi', 'yaw_ramp'],
                    required=True)
    ap.add_argument('--axis', choices=['surge', 'sway', 'yaw', 'heave'],
                    default='yaw')
    ap.add_argument('--value', type=float, default=100.0,
                    help='Joystick value (-100..100)')
    ap.add_argument('--duration', type=float, default=5.0)
    ap.add_argument('--on', type=float, default=0.5, help='Pulse on duration (s)')
    ap.add_argument('--off', type=float, default=0.5, help='Pulse off duration (s)')
    ap.add_argument('--cycles', type=int, default=10)
    ap.add_argument('--cmd', type=str, default='',
                    help='Comma-separated axis=value pairs for multi mode')
    args = ap.parse_args()
    arm_then_probe(args)
    print(f'Probe complete (mode={args.mode}, axis={args.axis}, value={args.value})')


if __name__ == '__main__':
    main()
