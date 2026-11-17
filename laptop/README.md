# Laptop code

Everything in this folder runs **on the laptop** and talks to the robots over Wi-Fi. The robot firmware is in [`../robot/`](../robot/).

| File | What it does |
|---|---|
| [`robot_client.py`](robot_client.py) | Control and test robots from the terminal. Standard library only |
| [`upload_wifi.sh`](upload_wifi.sh) | Build the robot firmware and send it to a robot over Wi-Fi |
| [`upload_usb.sh`](upload_usb.sh) | First-time firmware load over USB for a new Pico W |

## robot_client.py

Run these from the repo root. The robot defaults to `robot1.local`; pick another with `--host robot3.local` or `--host <ip>`.

```bash
python3 laptop/robot_client.py find
```
```bash
python3 laptop/robot_client.py info
```
```bash
python3 laptop/robot_client.py drive
```
```bash
python3 laptop/robot_client.py move forward --time 1
```
```bash
python3 laptop/robot_client.py wheeltest
```
```bash
python3 laptop/robot_client.py servo 120
```
```bash
python3 laptop/robot_client.py eyes blink
```
```bash
python3 laptop/robot_client.py ping -n 500
```

- `find` lists every robot on the network.
- `info` shows status: battery, wheels, head, eyes and Wi-Fi.
- `drive` is keyboard driving: WASD/arrows, Q/E spin, space stop, +/- speed, X quit.
- `move` does one timed move, then stops.
- `wheeltest` spins each wheel forward in turn (lift the robot first).
- `servo` turns the head (30–150°, 90 = straight ahead).
- `eyes` takes `on`, `off` or `blink`.
- `ping` measures round-trip time and loss.

You can also drive from a browser at `http://robot1.local/`. On macOS, Chrome needs **System Settings → Privacy & Security → Local Network** turned on to reach the robots; Safari works without it.

## Talking to the robots from your own code (e.g. the game)

Send plain-text UDP packets to port 4210. Each command gets one reply:

| Command | Effect |
|---|---|
| `drive <vx> <vy> <turn>` | Each -100..100: forward, slide right, spin clockwise. **Resend at least every 0.3 s** or the robot stops |
| `wheels <fl> <rl> <rr> <fr>` | Per-wheel power, -100..100 (same 0.3 s rule) |
| `stop` | Stop now |
| `servo <deg>` | Head angle, 30–150 |
| `eyes on\|off\|blink` | LED eyes |
| `info` | One-line status |
| `ping <x>` | Replies `pong <x>` |

Python example:
```python
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b"drive 50 0 0", ("robot1.local", 4210))
```
