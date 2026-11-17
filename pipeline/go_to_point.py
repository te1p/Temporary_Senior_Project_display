"""Vision-guided go-to-point: the overhead camera finds the robot, and the laptop
steers it to a spot on the field.

    # Real robot, iPhone camera (Continuity Camera number from detect_live.py --list):
    python3 pipeline/go_to_point.py --source 1 --host robot1.local --robot-tag 10
    # Click on the field to send the robot there.

    # First time with a robot, or if it drives the wrong way: check the drive directions
    # against the camera. It says what is wrong (tag on backwards, motors reversed...).
    python3 pipeline/go_to_point.py --source 1 --host robot1.local --check

    # No window: drive to one target and exit (0 = arrived, 1 = failed).
    python3 pipeline/go_to_point.py ... --target 600 300 --headless

Keys in the window: click = new target, p = park behind the puck, SPACE = stop / resume,
g = grid, r = record, q = quit.

Park mode (p or --park): the robot drives to a spot --standoff-mm behind the puck,
on the side away from where it will push it (blue robots push toward orange, +x),
and turns to face it. If the straight path would hit the puck it goes around. It
keeps following the puck as it moves. When the puck is out of view it stops and
waits; it carries on once the puck is seen again.

Recording: r (or --record) saves what the window shows as an .mp4 in recordings/,
at real speed. --record-raw also saves the plain camera feed, which can be run
through the vision code again later with --source <file>.

Safety: the robot is stopped when it leaves the field, when the camera loses its
tag for more than --lost-ms, when you press SPACE or q, and on any error. If this
program dies, the robot's own 300 ms failsafe stops it.

Each frame: find tags -> field position and heading of the robot -> D* Lite path
around the other robots and pucks (dstar_nav.py, using the team's
dstar_lite_planner.py) -> steer at a point --lookahead-mm along the path, in the
robot's own frame (forward, right) -> "drive <vx> <vy> <turn>". Mecanum wheels let it
slide along the path without turning first, while holding its starting heading (or
--heading). --planner direct drives in a straight line instead.

The head servo turns to look along the path while the robot moves (--no-head to
turn that off).
"""

import argparse
import csv
import math
import select
import socket
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision" / "aruco"))
from detect_live import open_source  # noqa: E402
from field_grid import FIELD_H, FIELD_W, FieldGrid  # noqa: E402
from mark_tags import KNOWN, draw_tags, find_tags  # noqa: E402
from recorder import Recorder  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dstar_nav import DStarNav  # noqa: E402

REPO = Path(__file__).resolve().parents[1]

TARGET_BGR = (0, 255, 255)
FRONT_BGR = (0, 0, 255)
PARK_BGR = (80, 220, 80)
PATH_BGR = (255, 200, 0)
OBSTACLE_BGR = (60, 60, 255)


def wrap180(a):
    return (a + 180) % 360 - 180


def forward_right(heading_deg):
    """Robot's forward and right unit vectors in field coordinates (x right, y down).
    Heading: 0 = +x, 90 = toward the top of the field (counterclockwise from above)."""
    t = math.radians(heading_deg)
    return np.array([math.cos(t), -math.sin(t)]), np.array([math.sin(t), math.cos(t)])


class RobotLink:
    """Sends text commands to the robot over UDP. dry_run just remembers the last one."""

    def __init__(self, host, port, dry_run=False):
        self.dry_run = dry_run
        self.last = "stop"
        if dry_run:
            return
        try:
            self.addr = (socket.gethostbyname(host), port)
        except OSError:
            raise SystemExit(f"Can't find {host}. Is the robot on and on this network?")
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("", 0))

    def send(self, text, remember=True):
        """remember=False for side commands (head, eyes): self.last stays the last drive
        command, which is what gets resent to keep the robot's failsafe fed."""
        if remember:
            self.last = text
        if self.dry_run:
            return
        self.s.sendto(text.encode(), self.addr)
        while select.select([self.s], [], [], 0)[0]:  # drop the "ok" replies
            self.s.recvfrom(512)

    def drive(self, vx, vy, turn):
        self.send(f"drive {round(vx)} {round(vy)} {round(turn)}")

    def stop(self):
        for _ in range(3):
            self.send("stop")

    def info(self, timeout=1.0):
        if self.dry_run:
            return "dry run: nothing is sent"
        self.s.sendto(b"info", self.addr)
        if select.select([self.s], [], [], timeout)[0]:
            return self.s.recvfrom(512)[0].decode(errors="replace")
        return None


class HeadAim:
    """Turns the head servo to look along the way the robot is going.

    The servo takes 30-150 deg, 90 = straight ahead. sign: +1 if a bigger angle turns
    the head to the robot's left (check: robot_client.py servo 150). offset: where the
    head faces relative to the robot's driving front (180 if it faces backward).
    Smoothed and rate-limited so it sends a few commands a second, not one per frame."""

    def __init__(self, link, enabled=True, sign=1, offset=0.0):
        self.link, self.enabled, self.sign, self.offset = link, enabled, sign, offset
        self.angle = 90.0
        self.sent = None
        self.sent_at = 0.0

    def look(self, relative_deg):
        """relative_deg: direction to look, counterclockwise from the robot's front.
        None = look straight ahead."""
        if not self.enabled:
            return
        want = 90.0
        if relative_deg is not None:
            want = 90 + self.sign * wrap180(relative_deg - self.offset)
        want = float(np.clip(want, 30, 150))
        self.angle += 0.35 * (want - self.angle)
        now = time.monotonic()
        a = round(self.angle)
        if self.sent is None or (abs(a - self.sent) >= 2 and now - self.sent_at >= 0.08):
            self.link.send(f"servo {a}", remember=False)
            self.sent, self.sent_at = a, now

    def center(self):
        if self.enabled:
            self.link.send("servo 90", remember=False)


class Controller:
    """Proportional controller in the robot's frame, with a minimum speed so the
    motors actually turn, and hysteresis so it doesn't jitter at the target: once
    there, it only moves again when the target is resume x tolerance away (heading:
    twice its tolerance). Park mode uses a smaller resume so it keeps up with the puck."""

    def __init__(self, args):
        self.max_speed, self.min_speed = args.max_speed, args.min_speed
        self.kp = args.kp
        self.tol = args.tolerance_mm
        self.turn_k, self.max_turn, self.min_turn = args.turn_kp, args.max_turn, args.min_turn
        self.heading_tol = args.heading_tolerance
        self.resume = 2.0
        self.arrived = False
        self.heading_ok = False

    def command(self, pose, target, target_heading, final=True, speed_dist=None):
        """Returns ((vx, vy, turn), distance_mm, heading_error_deg).
        final=False is for waypoints: never stop there, just keep moving.
        speed_dist: distance that sets the speed (e.g. path length left), if not the
        straight distance to target."""
        x, y, heading = pose
        err = np.array(target) - np.array([x, y])
        dist = float(np.hypot(*err))
        f, r = forward_right(heading)

        herr = wrap180(target_heading - heading) if target_heading is not None else 0.0
        if self.heading_ok and abs(herr) > 2 * self.heading_tol:
            self.heading_ok = False
        if not self.heading_ok and abs(herr) <= self.heading_tol:
            self.heading_ok = True
        turn = 0.0
        if not self.heading_ok:
            # Positive turn spins clockwise, which lowers the heading.
            turn = -float(np.clip(self.turn_k * herr, -self.max_turn, self.max_turn))
            turn = math.copysign(max(abs(turn), self.min_turn), turn)

        if not final:
            self.arrived = False
        else:
            if self.arrived and dist > self.resume * self.tol:
                self.arrived = False
            if not self.arrived and dist <= self.tol:
                self.arrived = True
        if self.arrived or dist < 1e-6:
            return (0.0, 0.0, turn), dist, herr

        speed = float(np.clip(self.kp * (dist if speed_dist is None else speed_dist),
                              self.min_speed, self.max_speed))
        vx, vy = err @ f / dist * speed, err @ r / dist * speed
        return (vx, vy, turn), dist, herr

    def reset(self):
        self.arrived = False
        self.heading_ok = False


def segment_clear(a, b, center, radius):
    """True if the straight line from a to b stays at least radius away from center."""
    a, b, c = np.asarray(a, float), np.asarray(b, float), np.asarray(center, float)
    d = b - a
    t = 0.0 if not d.any() else float(np.clip((c - a) @ d / (d @ d), 0.0, 1.0))
    return float(np.hypot(*(a + t * d - c))) >= radius


def park_spot(puck_xy, push_heading, standoff):
    """standoff mm behind the puck (opposite push_heading), pulled inside the field."""
    push, _ = forward_right(push_heading)
    return np.clip(np.array(puck_xy, float) - standoff * push, [0, 0], [FIELD_W, FIELD_H])


def find_obstacles(grid, tags, own_tag, args, hidden_puck=None):
    """Every robot and puck in view except our own robot, as ((x, y), radius_mm).
    hidden_puck: last known position of a puck that is out of view right now."""
    out = []
    for tag_id, pts in tags:
        info = KNOWN.get(tag_id, {})
        if tag_id == own_tag or info.get("group") == "anchor":
            continue
        center = tuple(float(v) for v in grid.to_field([pts.mean(axis=0)])[0])
        is_robot = "robot" in info.get("label", "").lower()
        out.append((center, args.other_robot_radius_mm if is_robot else args.puck_radius_mm))
    if hidden_puck is not None and not any(i == args.puck_tag for i, _ in tags):
        out.append((tuple(hidden_puck), args.puck_radius_mm))
    return out


def plan_park(robot_xy, puck_xy, push_heading, standoff, ring_points=16):
    """Where to drive next to park behind the puck: standoff mm from it on the side
    opposite push_heading, so that facing push_heading puts the puck right in front.

    Returns (next_target, is_final, park_spot). If the straight path would hit the
    puck, it plans the shortest way around it over waypoints on a ring around the
    puck, and next_target is the next waypoint. Waypoints outside the field count
    as blocked, so it never tries to squeeze between the puck and a wall. Near a
    wall the spot is pulled inside the field, so the robot parks as far behind the
    puck as it can. Re-run it every frame: it follows the puck as it moves."""
    puck, robot = np.array(puck_xy, float), np.array(robot_xy, float)
    push, _ = forward_right(push_heading)
    park = park_spot(puck_xy, push_heading, standoff)
    # Robot center must stay this far from the puck center (less if a wall squeezes the spot).
    keep_out = min(0.85 * standoff, 0.95 * float(np.hypot(*(park - puck))))
    ring = 1.3 * standoff

    away = robot - puck
    if np.hypot(*away) < keep_out:  # too close (touching it): back straight off first
        away = away / np.hypot(*away) if away.any() else -push
        return np.clip(puck + ring * away, [0, 0], [FIELD_W, FIELD_H]), False, park
    if segment_clear(robot, park, puck, keep_out):
        return park, True, park

    # Waypoints on the ring; None where the ring leaves the field.
    nodes = []
    for k in range(ring_points):
        a = 2 * math.pi * k / ring_points
        q = puck + ring * np.array([math.cos(a), math.sin(a)])
        nodes.append(q if inside_field(q) else None)

    # Shortest path: robot -> some reachable waypoints -> neighbours on the ring -> park.
    n = ring_points
    dist = {i: float(np.hypot(*(q - robot))) for i, q in enumerate(nodes)
            if q is not None and segment_clear(robot, q, puck, keep_out)}
    prev = {i: None for i in dist}
    todo = set(dist)
    best_total, best_end = math.inf, None
    while todo:
        i = min(todo, key=dist.get)
        todo.discard(i)
        if segment_clear(nodes[i], park, puck, keep_out):
            total = dist[i] + float(np.hypot(*(park - nodes[i])))
            if total < best_total:
                best_total, best_end = total, i
        for j in ((i + 1) % n, (i - 1) % n):
            if nodes[j] is None:
                continue
            d = dist[i] + float(np.hypot(*(nodes[j] - nodes[i])))
            if d < dist.get(j, math.inf):
                dist[j], prev[j] = d, i
                todo.add(j)
    if best_end is None:  # boxed in (e.g. puck in a corner): head for the spot anyway
        return park, True, park

    path = [best_end]
    while prev[path[-1]] is not None:
        path.append(prev[path[-1]])
    path.reverse()
    # Skip waypoints the robot has already reached, so it never stalls on one.
    for i in path:
        if np.hypot(*(nodes[i] - robot)) > 0.25 * standoff:
            return nodes[i], False, park
    return park, True, park


def robot_pose(grid, tags, tag_id, tag_offset=0.0):
    """(x_mm, y_mm, heading_deg) of the robot, or None if its tag isn't in view.
    tag_offset: degrees to add when the tag isn't mounted facing the robot's front."""
    for i, pts in tags:
        if i == tag_id:
            center, front = grid.to_field([pts.mean(axis=0), (pts[0] + pts[1]) / 2])
            heading = math.degrees(math.atan2(-(front[1] - center[1]), front[0] - center[0]))
            return float(center[0]), float(center[1]), (heading + tag_offset) % 360
    return None


class Vision:
    """Camera frames -> tags, field mapping and the robot's pose, plus the preview window."""

    def __init__(self, args):
        self.cap = open_source(args.source, args.width, args.height)
        if not self.cap.isOpened():
            raise SystemExit(f"could not open camera {args.source!r}")
        self.grid = FieldGrid(args.cell_mm)
        self.tag = args.robot_tag
        self.tag_offset = args.tag_offset
        self.window = not args.headless
        self.max_window = args.max_window
        self.show_grid = False
        self.clicks = []
        self.scale = 1.0
        self.rec = Recorder(args.record_dir, "go_to_point")
        self.rec_raw = Recorder(args.record_dir, "go_to_point_raw") if args.record_raw else None
        if args.record:
            self.toggle_recording()
        if self.window:
            cv2.namedWindow("go to point")
            cv2.setMouseCallback("go to point", self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicks.append((x / self.scale, y / self.scale))

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            raise SystemExit("camera stream ended")
        tags = find_tags(frame)
        self.grid.update(tags)
        pose = robot_pose(self.grid, tags, self.tag, self.tag_offset) if self.grid.ready else None
        return frame, tags, pose

    def show(self, frame, tags, pose, target, status_lines, park=None, nav=None, goal=None):
        """Draws the overlay, records it if recording, and returns the key pressed (or -1).
        park: (puck_xy, park_spot_xy, push_heading, standoff_mm) to draw in park mode.
        nav: the DStarNav, to draw its path and grown obstacles. goal: the final destination."""
        if not self.window and not self.rec.active:
            return -1
        out = frame.copy()
        g = self.grid
        if g.ready:
            if self.show_grid:
                g.draw(out, g.matrix(tags), tags)
            border = g.to_image([(0, 0), (FIELD_W, 0), (FIELD_W, FIELD_H), (0, FIELD_H)])
            cv2.polylines(out, [np.int32(border)], True, (255, 0, 255), 3, cv2.LINE_AA)
            if park is not None:
                puck, spot, push_heading, standoff = park
                ring = [np.array(puck) + 0.85 * standoff * np.array([math.cos(a), math.sin(a)])
                        for a in np.linspace(0, 2 * math.pi, 40)]
                cv2.polylines(out, [np.int32(g.to_image(ring))], True, PARK_BGR, 1, cv2.LINE_AA)
                f, _ = forward_right(push_heading)
                a, b = g.to_image([puck, np.array(puck) + 1.2 * standoff * f])
                cv2.arrowedLine(out, tuple(np.int32(a)), tuple(np.int32(b)), PARK_BGR, 3, cv2.LINE_AA, tipLength=0.2)
                c = tuple(np.int32(g.to_image([spot])[0]))
                cv2.circle(out, c, 14, PARK_BGR, 3, cv2.LINE_AA)
                cv2.putText(out, "park", (c[0] + 16, c[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, PARK_BGR, 2,
                            cv2.LINE_AA)
            if nav is not None:
                for center, radius in nav.obstacles:
                    reach = radius + nav.robot_radius
                    ring = [np.array(center) + reach * np.array([math.cos(a), math.sin(a)])
                            for a in np.linspace(0, 2 * math.pi, 40)]
                    cv2.polylines(out, [np.int32(g.to_image(ring))], True, OBSTACLE_BGR, 1, cv2.LINE_AA)
                if len(nav.path) > 1:
                    cv2.polylines(out, [np.int32(g.to_image(nav.path))], False, PATH_BGR, 3, cv2.LINE_AA)
            if goal is not None:
                gp = tuple(np.int32(g.to_image([goal])[0]))
                cv2.drawMarker(out, gp, TARGET_BGR, cv2.MARKER_TILTED_CROSS, 22, 2)
            if target is not None:
                t = tuple(np.int32(g.to_image([target])[0]))
                cv2.drawMarker(out, t, TARGET_BGR, cv2.MARKER_CROSS, 40, 3)
                cv2.circle(out, t, 18, TARGET_BGR, 2)
                if pose is not None:
                    p = tuple(np.int32(g.to_image([pose[:2]])[0]))
                    cv2.line(out, p, t, TARGET_BGR, 2, cv2.LINE_AA)
        draw_tags(out, tags)
        if g.ready and pose is not None:
            f, _ = forward_right(pose[2])
            a, b = g.to_image([pose[:2], np.array(pose[:2]) + 150 * f])
            cv2.arrowedLine(out, tuple(np.int32(a)), tuple(np.int32(b)), FRONT_BGR, 4, cv2.LINE_AA, tipLength=0.25)
            cv2.putText(out, "front", tuple(np.int32(b) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, FRONT_BGR, 2, cv2.LINE_AA)
        for i, line in enumerate(status_lines):
            y = 32 + 30 * i
            cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5, cv2.LINE_AA)
            cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        h, w = out.shape[:2]
        self.scale = min(1.0, self.max_window / w)
        if self.scale < 1.0:
            out = cv2.resize(out, (round(w * self.scale), round(h * self.scale)))
        self.rec.write(out)
        if self.rec_raw:
            self.rec_raw.write(frame)
        if not self.window:
            return -1
        cv2.imshow("go to point", self.rec.badge(out))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            self.toggle_recording()
        return key

    def toggle_recording(self):
        recording, path = self.rec.toggle()
        if self.rec_raw:
            self.rec_raw.toggle()
        if recording:
            print(f"recording to {path}")
        else:
            self._report(path, self.rec_raw.path if self.rec_raw else None)

    def _report(self, *paths):
        for path in paths:
            if path:
                print(f"saved {path}   (open it with:  open {path})")

    def close(self):
        if self.rec.active:
            self._report(self.rec.stop(), self.rec_raw.stop() if self.rec_raw else None)
        self.cap.release()
        if self.window:
            cv2.destroyAllWindows()


def inside_field(pt, margin=0.0):
    return -margin <= pt[0] <= FIELD_W + margin and -margin <= pt[1] <= FIELD_H + margin


def wait_for_robot(vision, link, seconds=10):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        frame, tags, pose = vision.read()
        if pose is not None:
            return pose
        missing = "corner tags (0-3)" if not vision.grid.ready else f"robot tag {vision.tag}"
        key = vision.show(frame, tags, pose, None, [f"waiting for {missing}..."])
        if key == ord("q"):
            break
    raise SystemExit("robot not found: check that the corner tags and the robot's tag are in view")


def run_check(vision, link, speed):
    """Drive forward, slide right and spin clockwise in turn, measure each with the
    camera, and say what is wrong if the robot doesn't go where the code expects."""
    pose = wait_for_robot(vision, link)
    tests = [("forward", (speed, 0, 0)), ("slide right", (0, speed, 0)), ("spin clockwise", (0, 0, speed))]
    print(f"Direction check at {speed}% power. Give the robot about 30 cm of space all round.")
    print("Directions are as the camera sees the tag: 0 deg = toward the tag's FRONT arrow, 90 = its right.")
    seen = {}
    for name, cmd in tests:
        start = hold(vision, link, None, 0.6, ["check: settling"]) or pose
        end = hold(vision, link, cmd, 0.6, [f"check: {name}"])
        end = hold(vision, link, None, 0.5, ["check: settling"]) or end
        if start is None or end is None:
            print(f"  {name}: robot tag lost, can't tell")
            continue
        f, r = forward_right(start[2])
        d = np.array(end[:2]) - np.array(start[:2])
        fwd, right, turned = float(d @ f), float(d @ r), wrap180(end[2] - start[2])
        moved = float(np.hypot(fwd, right))
        if name == "spin clockwise":
            seen[name] = turned
            verdict = "PASS" if turned < -5 else "FAIL" if turned > 5 else "DID NOT TURN"
            print(f"  spin clockwise: heading changed {turned:+.0f} deg (should go down) -> {verdict}  "
                  f"(~{abs(turned) / 0.6:.0f} deg/s)")
        else:
            # Direction of travel, clockwise from the tag's FRONT: forward should be 0, right 90.
            angle = math.degrees(math.atan2(right, fwd)) if moved > 20 else None
            seen[name] = angle
            want = 0 if name == "forward" else 90
            if angle is None:
                print(f"  {name}: moved only {moved:.0f} mm -> DID NOT MOVE")
            else:
                verdict = "PASS" if abs(wrap180(angle - want)) < 30 else "FAIL"
                print(f"  {name}: went {moved:.0f} mm toward {angle:+.0f} deg (should be {want}) -> {verdict}  "
                      f"(~{moved / 0.6:.0f} mm/s)")
    link.stop()
    return diagnose(seen)


def diagnose(seen):
    fwd, right, spin = seen.get("forward"), seen.get("slide right"), seen.get("spin clockwise")
    if fwd is None or right is None or spin is None or abs(spin) <= 5:
        print("\nThe robot didn't move enough to tell. Check the battery and power switch, raise --check-speed,\n"
              "or try: python3 laptop/robot_client.py --host <robot> move forward")
        return False
    # Forward and right should be 90 deg apart, clockwise. If they are, the robot drives
    # consistently and any rotation is just how the tag sits on it.
    consistent = abs(wrap180(right - fwd - 90)) < 30
    spin_ok = spin < 0
    off = round(fwd / 90) * 90 % 360  # how far the tag is turned, to the nearest quarter turn
    if consistent and spin_ok and abs(wrap180(fwd)) < 30:
        print("\nAll directions match. go_to_point can drive this robot.")
        return True
    if consistent and spin_ok:
        print(f"\nThe robot drives correctly, but its TAG is turned {off} deg: the robot's front is at "
              f"{off} deg clockwise from the tag's FRONT arrow.")
        fix = (-off) % 360
        print(f'Fix: turn the tag so its FRONT arrow faces the robot\'s front, or set "front_offset_deg": {fix:g} '
              f"for this robot in vision/aruco/markers.json (test it once with --tag-offset {fix:g}).")
        return False
    if abs(wrap180(fwd - 180)) < 30 and abs(wrap180(right + 90)) < 30 and not spin_ok:
        print("\nEVERY direction is reversed (forward goes back, right goes left, clockwise goes counterclockwise).")
        print("The motors are wired or configured backwards. Run\n"
              "  python3 laptop/robot_client.py --host <robot> wheeltest\n"
              "and set MOTORn_REVERSED to 1 for each wheel that spins backward (robot/robot_wifi/config.h).")
        return False
    print("\nThe directions don't fit one simple cause: probably some wheels spin the wrong way.\n"
          "Run  python3 laptop/robot_client.py --host <robot> wheeltest  and fix MOTORn_REVERSED in\n"
          "robot/robot_wifi/config.h for each wheel that spins backward.")
    return False


def hold(vision, link, cmd, seconds, status):
    """Send cmd (or stop) every frame for this long. Returns the last seen pose."""
    end = time.monotonic() + seconds
    pose = None
    while time.monotonic() < end:
        frame, tags, p = vision.read()
        pose = p or pose
        link.drive(*cmd) if cmd else link.send("stop")
        if vision.show(frame, tags, p, None, status) == ord("q"):
            link.stop()
            raise SystemExit("stopped")
    return pose


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0", help="camera number, stream URL or video file")
    ap.add_argument("--host", default="robot1.local", help="robot name or IP")
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--robot-tag", type=int, default=10, help="the robot's tag ID (default 10, Blue robot 1)")
    ap.add_argument("--tag-offset", type=float,
                    help="degrees from the tag's FRONT arrow to the robot's front (default: front_offset_deg "
                         "in markers.json, else 0; --check prints the value to use)")
    ap.add_argument("--target", type=float, nargs=2, metavar=("X_MM", "Y_MM"), help="start with this target")
    ap.add_argument("--planner", choices=["dstar", "direct"], default="dstar",
                    help="dstar (default): D* Lite path around robots and pucks; direct: straight line")
    ap.add_argument("--dstar-cell-mm", type=float, default=25, help="D* grid cell size (default 25)")
    ap.add_argument("--robot-radius-mm", type=float, default=110,
                    help="our robot's radius: how far its center keeps from obstacles (default 110)")
    ap.add_argument("--puck-radius-mm", type=float, default=35, help="puck radius (default 35)")
    ap.add_argument("--other-robot-radius-mm", type=float, default=110, help="other robots' radius (default 110)")
    ap.add_argument("--lookahead-mm", type=float, default=120,
                    help="steer at the path point this far ahead (default 120)")
    ap.add_argument("--no-head", action="store_true", help="don't turn the head servo to follow the path")
    ap.add_argument("--head-sign", type=int, choices=[1, -1], default=1,
                    help="1 if 'servo 150' turns the head to the robot's left, else -1")
    ap.add_argument("--head-offset", type=float, default=0,
                    help="where the head faces relative to the driving front, deg (180 if backward)")
    ap.add_argument("--park", action="store_true",
                    help="park behind the puck, facing it, and follow it if it moves (or press p)")
    ap.add_argument("--puck-tag", type=int, default=30, help="the puck's tag ID (default 30)")
    ap.add_argument("--follow-resume", type=float, default=1.5,
                    help="park mode: drive again once the park spot is this many x --tolerance-mm away "
                         "(default 1.5 = 90 mm; lower follows more closely but twitches more)")
    ap.add_argument("--push-heading", type=float,
                    help="direction the robot will push the puck, deg (default: 0 = toward orange for blue "
                         "robots, 180 for orange)")
    ap.add_argument("--standoff-mm", type=float, default=180,
                    help="robot center to puck center when parked (default 180)")
    ap.add_argument("--heading", type=float, help="hold this heading in degrees (default: the starting heading)")
    ap.add_argument("--max-speed", type=float, default=35, help="%% power cap (default 35; stay <= 55 on AAs)")
    ap.add_argument("--min-speed", type=float, default=20, help="%% power below which the wheels don't turn")
    ap.add_argument("--kp", type=float, default=0.25, help="%% power per mm of distance (default 0.25)")
    ap.add_argument("--tolerance-mm", type=float, default=60, help="arrived when this close (default 60)")
    ap.add_argument("--turn-kp", type=float, default=1.0, help="%% turn power per degree of heading error")
    ap.add_argument("--max-turn", type=float, default=30)
    ap.add_argument("--min-turn", type=float, default=18)
    ap.add_argument("--heading-tolerance", type=float, default=12, help="heading counts as right within this, deg")
    ap.add_argument("--lost-ms", type=float, default=250, help="stop if the robot's tag is unseen this long")
    ap.add_argument("--margin-mm", type=float, default=40, help="stop if the robot goes this far outside")
    ap.add_argument("--timeout", type=float, default=30, help="headless: give up after this many seconds")
    ap.add_argument("--check", action="store_true", help="test drive directions against the camera, then exit")
    ap.add_argument("--check-speed", type=int, default=35)
    ap.add_argument("--dry-run", action="store_true", help="compute commands but send nothing")
    ap.add_argument("--headless", action="store_true", help="no window; needs --target")
    ap.add_argument("--log", type=Path, help="write a CSV of time, pose and commands")
    ap.add_argument("--record", action="store_true", help="start recording the window right away (or press r)")
    ap.add_argument("--record-raw", action="store_true", help="also record the plain camera feed")
    ap.add_argument("--record-dir", type=Path, default=REPO / "recordings", help="where videos go")
    ap.add_argument("--cell-mm", type=float)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--max-window", type=int, default=1280)
    args = ap.parse_args()
    if args.headless and not (args.target or args.check or args.park):
        ap.error("--headless needs --target, --park or --check")
    if args.park and args.planner == "dstar" and args.standoff_mm <= args.puck_radius_mm + args.robot_radius_mm:
        ap.error(f"--standoff-mm must be more than --puck-radius-mm + --robot-radius-mm "
                 f"({args.puck_radius_mm + args.robot_radius_mm:g}) or the park spot is inside the puck's zone")
    if args.push_heading is None:
        args.push_heading = 180.0 if "orange" in KNOWN.get(args.robot_tag, {}).get("label", "").lower() else 0.0
    if args.target and not inside_field(args.target):
        ap.error(f"target is outside the field (0-{FIELD_W} x 0-{FIELD_H} mm)")

    link = RobotLink(args.host, args.port, args.dry_run)
    reply = link.info()
    if reply is None:
        raise SystemExit(f"No reply from {args.host}:{args.port}. Is the robot on and on this network?")
    print(f"robot: {reply}")
    if args.tag_offset is None:
        args.tag_offset = KNOWN.get(args.robot_tag, {}).get("front_offset_deg", 0)
    if args.tag_offset:
        print(f"robot's front is {args.tag_offset:g} deg from its tag's FRONT arrow "
              f"(front_offset_deg in markers.json; --tag-offset to override)")
    vision = Vision(args)
    log_file = open(args.log, "w", newline="") if args.log else None
    log = csv.writer(log_file) if log_file else None
    if log:
        log.writerow(["t", "x_mm", "y_mm", "heading", "target_x", "target_y", "dist_mm", "command"])

    try:
        if args.check:
            sys.exit(0 if run_check(vision, link, args.check_speed) else 1)

        pose = wait_for_robot(vision, link)
        controller = Controller(args)
        click_goal = tuple(args.target) if args.target else None
        target = None
        target_heading = args.heading if args.heading is not None else pose[2]
        park = args.park
        last_puck = None
        puck_seen_at = -math.inf
        nav = DStarNav(FIELD_W, FIELD_H, args.dstar_cell_mm, args.robot_radius_mm, args.lookahead_mm) \
            if args.planner == "dstar" else None
        head = HeadAim(link, not args.no_head, args.head_sign, args.head_offset)
        state = "DRIVING" if (click_goal or park) else "IDLE (click a target or press p)"
        last_seen = time.monotonic()
        t0 = time.monotonic()
        dist = herr = 0.0
        result = 1

        while True:
            frame, tags, pose = vision.read()
            now = time.monotonic()

            for click in vision.clicks:
                if vision.grid.ready:
                    pt = tuple(float(v) for v in vision.grid.to_field([click])[0])
                    if inside_field(pt):
                        click_goal, state, park = pt, "DRIVING", False
                        controller.reset()
                        print(f"target {pt[0]:.0f}, {pt[1]:.0f} mm")
            vision.clicks.clear()

            if pose is not None:
                last_seen = now
                if not inside_field(pose, args.margin_mm) and state == "DRIVING":
                    state = "STOPPED: left the field"
                    link.stop()
            elif state == "DRIVING" and now - last_seen > args.lost_ms / 1000:
                state = "PAUSED: robot tag lost"
                link.stop()
            if state.startswith("PAUSED: robot tag lost") and pose is not None:
                state = "DRIVING"

            puck = robot_pose(vision.grid, tags, args.puck_tag) if vision.grid.ready else None
            if puck is not None:
                last_puck, puck_seen_at = puck[:2], now
            # The puck counts as visible for --lost-ms after its last sighting, so one
            # missed frame doesn't make the robot jerk to a stop.
            puck_visible = now - puck_seen_at <= args.lost_ms / 1000
            controller.resume = args.follow_resume if park else 2.0
            # Where we're ultimately going, then where to steer right now.
            final, park_draw, note, target, remaining = True, None, "", None, None
            goal = click_goal
            if park:
                target_heading = args.push_heading
                goal = None  # puck not in view: do nothing
                if puck_visible:
                    goal = tuple(park_spot(last_puck, args.push_heading, args.standoff_mm))
                    park_draw = (last_puck, goal, args.push_heading, args.standoff_mm)
            if goal is not None and pose is not None:
                if nav is not None:
                    obstacles = find_obstacles(vision.grid, tags, args.robot_tag, args,
                                               last_puck if park and puck_visible else None)
                    target, final, note, remaining = nav.step(pose[:2], goal, obstacles)
                elif park:
                    nxt, final, _ = plan_park(pose[:2], last_puck, args.push_heading, args.standoff_mm)
                    target = tuple(float(v) for v in nxt)
                else:
                    target = goal

            cmd = (0.0, 0.0, 0.0)
            if goal is None and nav is not None:
                nav.path, nav.obstacles = [], []  # nothing planned: don't draw an old path
            if state == "DRIVING" and park and goal is None:
                link.send("stop")  # puck not in view: stay put until it is
                controller.reset()
            if state == "DRIVING" and goal is not None:
                if pose is not None and target is None:
                    link.send("stop")  # no way there (note says why)
                elif pose is not None:
                    cmd, dist, herr = controller.command(pose, target, target_heading, final, remaining)
                    if cmd == (0.0, 0.0, 0.0):
                        link.send("stop")
                    else:
                        link.drive(*cmd)
                    if controller.arrived and cmd[2] == 0.0:
                        if args.headless:
                            print(f"arrived: {dist:.0f} mm from target, heading error {herr:.0f} deg, "
                                  f"{now - t0:.1f} s")
                            result = 0
                            break
                else:
                    link.send(link.last)  # keep the failsafe fed for the short --lost-ms gap

            # Head looks along the way it's going while moving, straight ahead otherwise.
            moving = state == "DRIVING" and pose is not None and target is not None and (cmd[0] or cmd[1])
            if moving:
                travel = math.degrees(math.atan2(-(target[1] - pose[1]), target[0] - pose[0]))
                head.look(wrap180(travel - pose[2]))
            else:
                head.look(None)

            if log and pose is not None:
                log.writerow([f"{now - t0:.3f}", f"{pose[0]:.1f}", f"{pose[1]:.1f}", f"{pose[2]:.1f}",
                              *(f"{v:.0f}" for v in (target or (np.nan, np.nan))), f"{dist:.1f}", link.last])

            if args.headless:
                if state.startswith("STOPPED"):
                    print(state)
                    break
                if now - t0 > args.timeout:
                    print(f"timeout: still {remaining if remaining is not None else dist:.0f} mm to go")
                    break

            where = f"robot {pose[0]:.0f}, {pose[1]:.0f} mm, {pose[2]:.0f} deg" if pose else "robot not seen"
            shown_state = "ARRIVED" if state == "DRIVING" and controller.arrived and cmd == (0, 0, 0) else state
            if park:
                shown_state = "PARK: " + ("waiting: puck not in view" if not puck_visible else shown_state)
                if nav is None and not final and puck_visible:
                    shown_state += " (going around the puck)"
            if note:
                shown_state += f" - {note}"
            if nav is not None:
                shown_state = "D* " + shown_state
            to_go = "-" if goal is None else f"{remaining if remaining is not None else dist:.0f} mm"
            lines = [f"{shown_state}   {where}", f"to go {to_go}   cmd: {link.last}",
                     "click = target   p = park at puck   SPACE = stop/resume   g = grid   r = record   q = quit"]
            key = vision.show(frame, tags, pose, target, lines, park_draw, nav, goal)
            if key == ord("q"):
                break
            if key == ord("g"):
                vision.show_grid = not vision.show_grid
            if key == ord("p"):
                park = not park
                controller.reset()
                if park:
                    state = "DRIVING"
                else:
                    click_goal, state = None, "IDLE (click a target or press p)"
                    link.stop()
            if key == ord(" "):
                if state == "DRIVING":
                    state = "STOPPED (SPACE to resume)"
                    link.stop()
                elif click_goal is not None or park:
                    state = "DRIVING"
                    controller.reset()
        sys.exit(result if args.headless else 0)
    finally:
        link.stop()
        if not args.check and "head" in locals():
            head.center()
        vision.close()
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()
