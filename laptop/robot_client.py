#!/usr/bin/env python3
"""Talk to a Robot Duels Pico W over Wi-Fi (UDP port 4210). Standard library only.

  python3 laptop/robot_client.py info                  # robot's status (incl. battery)
  python3 laptop/robot_client.py wheeltest             # spin each wheel in turn
  python3 laptop/robot_client.py drive                 # keyboard driving
  python3 laptop/robot_client.py move forward --time 1 # one timed move
  python3 laptop/robot_client.py servo 90              # turn the head (30-150, 90 = ahead)
  python3 laptop/robot_client.py eyes blink            # LED eyes: on | off | blink
  python3 laptop/robot_client.py ping -n 200 --rate 50 # round-trip timing
  python3 laptop/robot_client.py led on|off|auto       # blink test
  python3 laptop/robot_client.py find                  # list robots on this network
  python3 laptop/robot_client.py send "any text"       # raw command

Default host is robot1.local. Use --host robot3.local or --host <ip> for others.
Lift the wheels off the table the first time you drive.
"""
import argparse
import os
import select
import socket
import sys
import time

PORT = 4210
SEND_HZ = 20  # robot stops if it hears nothing for 300 ms, so resend well inside that

MOVES = {  # (forward, slide right, spin clockwise)
    "forward": (1, 0, 0), "back": (-1, 0, 0), "left": (0, -1, 0), "right": (0, 1, 0),
    "cw": (0, 0, 1), "ccw": (0, 0, -1), "stop": (0, 0, 0),
    "forward-left": (1, -1, 0), "forward-right": (1, 1, 0), "back-left": (-1, -1, 0), "back-right": (-1, 1, 0),
}
WHEELS = ["M1 front-left", "M2 rear-left", "M3 rear-right", "M4 front-right"]


def make_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("", 0))
    return s


def request(host, text, timeout=1.0):
    s = make_socket()
    t0 = time.perf_counter()
    s.sendto(text.encode(), (host, PORT))
    if not select.select([s], [], [], timeout)[0]:
        raise SystemExit(f"No reply from {host}:{PORT} within {timeout:g} s. "
                         "Is the laptop on the robot's Wi-Fi network?")
    data, addr = s.recvfrom(512)
    return data.decode(errors="replace"), (time.perf_counter() - t0) * 1000, addr


def cmd_find(args):
    s = make_socket()
    # Single broadcast packets get dropped now and then on Wi-Fi, so send a few,
    # to both the global and this subnet's broadcast address.
    targets = ["255.255.255.255"]
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))  # no packet is sent; just picks the outgoing interface
        my_ip = probe.getsockname()[0]
        targets.append(my_ip.rsplit(".", 1)[0] + ".255")  # assumes a /24 network, the usual case
    except OSError:
        pass
    print("Robots that answered:")
    found = {}
    end = time.time() + max(args.timeout, 1.5)
    next_send, sends = 0.0, 0
    while (left := end - time.time()) > 0:
        if sends < 3 and time.time() >= next_send:
            for t in targets:
                s.sendto(b"hello", (t, PORT))
            sends += 1
            next_send = time.time() + 0.3
        if select.select([s], [], [], min(left, 0.1))[0]:
            data, addr = s.recvfrom(512)
            if addr[0] not in found:
                found[addr[0]] = data.decode(errors="replace")
                print(f"  {addr[0]:15s}  {found[addr[0]]}")
    if not found:
        print("  none (check you're on the same network)")


def cmd_ping(args):
    s = make_socket()
    host = socket.gethostbyname(args.host)
    period = 1.0 / args.rate
    sent = {}
    rtts = []
    start = time.perf_counter()
    next_send = start
    seq = 0
    deadline = None
    while True:
        now = time.perf_counter()
        if seq < args.count and now >= next_send:
            sent[seq] = now
            s.sendto(f"ping {seq}".encode(), (host, PORT))
            seq += 1
            next_send += period
            if seq == args.count:
                deadline = now + args.timeout
        if deadline and now > deadline:
            break
        wait = max(0.0, min(next_send - now if seq < args.count else 0.05, 0.05))
        if select.select([s], [], [], wait)[0]:
            data, _ = s.recvfrom(512)
            t = time.perf_counter()
            parts = data.split()
            if len(parts) == 2 and parts[0] == b"pong" and parts[1].isdigit():
                k = int(parts[1])
                if k in sent:
                    rtts.append((t - sent.pop(k)) * 1000)
            if deadline and not sent:
                break
    n = args.count
    got = len(rtts)
    print(f"{host}: sent {n}, received {got}, lost {n - got} ({100 * (n - got) / n:.1f}%)")
    if rtts:
        r = sorted(rtts)
        p = lambda q: r[min(len(r) - 1, int(q / 100 * len(r)))]  # noqa: E731
        print(f"round trip ms: min {r[0]:.1f}  median {p(50):.1f}  p95 {p(95):.1f}  "
              f"p99 {p(99):.1f}  max {r[-1]:.1f}")


class Driver:
    """Keeps resending the current command so the robot's failsafe stays armed."""

    def __init__(self, host):
        self.s = make_socket()
        self.addr = (host, PORT)

    def send(self, text):
        self.s.sendto(text.encode(), self.addr)
        while select.select([self.s], [], [], 0)[0]:  # throw away the "ok" replies
            self.s.recvfrom(512)

    def hold(self, text, seconds):
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            self.send(text)
            time.sleep(1 / SEND_HZ)

    def stop(self):
        for _ in range(3):
            self.send("stop")
            time.sleep(0.02)


def drive_cmd(v, speed):
    return f"drive {v[0] * speed} {v[1] * speed} {v[2] * speed}"


def cmd_move(args):
    d = Driver(args.host)
    try:
        d.hold(drive_cmd(MOVES[args.direction], args.speed), args.time)
    finally:
        d.stop()
    print(f"{args.direction} at {args.speed}% for {args.time:g} s, then stop")


def cmd_wheeltest(args):
    d = Driver(args.host)
    print("Each wheel should spin FORWARD (the top of the wheel moving toward the front of the car).")
    print("If one spins backward, set its MOTORn_REVERSED to 1 in robot/robot_wifi/config.h.\n")
    try:
        for i, name in enumerate(WHEELS):
            speeds = [0, 0, 0, 0]
            speeds[i] = args.speed
            print(f"  {name} ...", flush=True)
            d.hold("wheels " + " ".join(map(str, speeds)), args.time)
            d.stop()
            time.sleep(0.6)
    finally:
        d.stop()
    print("\nDone. Nothing moved? Check the battery is connected and the power switch is ON.")


KEYMAP = {"w": "forward", "s": "back", "a": "left", "d": "right", "q": "ccw", "e": "cw",
          "\x1b[A": "forward", "\x1b[B": "back", "\x1b[D": "left", "\x1b[C": "right", " ": "stop"}


def cmd_drive(args):
    import termios
    import tty

    d = Driver(args.host)
    speed, move = args.speed, "stop"
    print("W/S or arrows: forward/back   A/D: slide left/right   Q/E: spin   SPACE: stop\n"
          "+/-: speed   X: quit.   Keys LATCH: the robot keeps going until you press another key or SPACE.\n")
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        next_send = 0.0
        while True:
            if select.select([fd], [], [], 0.01)[0]:
                key = os.read(fd, 3).decode(errors="ignore")
                k = key.lower() if len(key) == 1 else key
                if k in ("x", "\x03"):
                    break
                if k in ("+", "="):
                    speed = min(100, speed + 10)
                elif k in ("-", "_"):
                    speed = max(10, speed - 10)
                elif k in KEYMAP:
                    move = KEYMAP[k]
                print(f"\r  {move:<8s} speed {speed:3d}%   ", end="", flush=True)
                next_send = 0  # send the change right away
            if time.perf_counter() >= next_send:
                d.send(drive_cmd(MOVES[move], speed) if move != "stop" else "stop")
                next_send = time.perf_counter() + 1 / SEND_HZ
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        d.stop()
        print("\nstopped")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="robot1.local", help="robot name or IP (default robot1.local)")
    ap.add_argument("--timeout", type=float, default=1.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    sub.add_parser("find")
    p = sub.add_parser("ping")
    p.add_argument("-n", "--count", type=int, default=100)
    p.add_argument("--rate", type=float, default=50, help="pings per second")
    p = sub.add_parser("led")
    p.add_argument("mode", choices=["on", "off", "auto"])
    p = sub.add_parser("send")
    p.add_argument("text")
    p = sub.add_parser("drive", help="keyboard driving")
    p.add_argument("--speed", type=int, default=50, help="starting speed %% (default 50)")
    p = sub.add_parser("move", help="one timed move, then stop")
    p.add_argument("direction", choices=list(MOVES))
    p.add_argument("--speed", type=int, default=50)
    p.add_argument("--time", type=float, default=1.0, help="seconds (default 1)")
    p = sub.add_parser("servo", help="turn the head: 30-150 degrees, 90 = straight ahead")
    p.add_argument("angle", type=int)
    p = sub.add_parser("eyes", help="LED eyes on/off, or blink now")
    p.add_argument("state", choices=["on", "off", "blink"])
    p = sub.add_parser("wheeltest", help="spin each wheel forward in turn")
    p.add_argument("--speed", type=int, default=50)
    p.add_argument("--time", type=float, default=1.5, help="seconds per wheel")
    args = ap.parse_args()

    if args.cmd != "find":
        try:  # resolve robot1.local once, not on every packet
            args.host = socket.gethostbyname(args.host)
        except OSError:
            raise SystemExit(f"Can't find {args.host}. Is the robot on and on this network? Try: find")

    if args.cmd == "find":
        cmd_find(args)
    elif args.cmd == "ping":
        cmd_ping(args)
    elif args.cmd == "drive":
        cmd_drive(args)
    elif args.cmd == "move":
        cmd_move(args)
    elif args.cmd == "wheeltest":
        cmd_wheeltest(args)
    else:
        text = {"info": "info", "led": f"led {getattr(args, 'mode', '')}",
                "servo": f"servo {getattr(args, 'angle', '')}",
                "eyes": f"eyes {getattr(args, 'state', '')}"}.get(args.cmd) or args.text
        reply, ms, addr = request(args.host, text, args.timeout)
        print(f"{reply}   ({ms:.1f} ms from {addr[0]})")


if __name__ == "__main__":
    sys.exit(main())
