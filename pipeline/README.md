# Pipeline test: camera → laptop → robot

The overhead camera finds the robot's tag, the laptop works out where it is on the field, and it sends drive commands until the robot reaches the spot you clicked.

| File | What it does |
|---|---|
| [`go_to_point.py`](go_to_point.py) | Drives one robot to a target on the field using the camera. Includes a direction check |
| [`dstar_nav.py`](dstar_nav.py) | Plans the way there with the team's D* Lite planner ([`../dstar_lite_planner.py`](../dstar_lite_planner.py), from the `path-planner` branch) |

It uses the vision code in [`../vision/aruco/`](../vision/aruco/) (tag detection, field mapping, `markers.json`) and the robot's UDP commands from [`../laptop/README.md`](../laptop/README.md).

## With a real robot

1. Turn the robot on and check the laptop can see it:
   ```bash
   python3 laptop/robot_client.py find
   ```
2. Tape tag 10 (Blue robot 1) flat on top of the robot, with its FRONT arrow facing the robot's front.
3. Find the camera number (the iPhone is usually 1):
   ```bash
   python3 vision/aruco/detect_live.py --list
   ```
4. Check that the drive directions match what the camera sees. The robot drives forward, slides right and spins, about 10–15 cm each, so give it room:
   ```bash
   python3 pipeline/go_to_point.py --source 1 --check
   ```
   All three should say PASS. It also prints the robot's speed in mm/s.

   If the robot's driving front isn't where its tag's FRONT arrow points, the check says by how much. Put that number in `vision/aruco/markers.json` as `"front_offset_deg"` for that robot's tag. Blue robot 1 (tag 10) has `180`: it drives toward its tag's back edge. In the window, the red **front** arrow shows the robot's real front.
5. Drive it. Click anywhere inside the magenta border, and the robot goes there:
   ```bash
   python3 pipeline/go_to_point.py --source 1
   ```
   **p** parks it behind the puck (see below). **SPACE** stops it or resumes, **g** shows the grid, **r** starts or stops recording, and **q** quits.

Use `--host robot3.local --robot-tag 12` for another robot. Add `--dry-run` to see the commands on screen without sending them.

## How it steers

Every camera frame:
1. Find the robot's position (mm) and heading on the field from its tag.
2. Turn the distance to the target into the robot's own directions: how far forward and how far to the right.
3. Send `drive <vx> <vy> <turn>`. Mecanum wheels slide straight toward the target without turning first. `turn` holds the starting heading, or `--heading`.

Speed is proportional to distance, between `--min-speed` and `--max-speed`, so the robot slows down as it gets close.

**Arrived:** within `--tolerance-mm` (60 mm) of the target and within `--heading-tolerance` (12°) of the heading. Once there, it stays put until it's pushed twice that far off, so it doesn't twitch at the edge.

## Path planning with D* Lite

By default the robot doesn't drive straight at the target. It follows a path from the team's D* Lite planner that goes around every other robot and puck the camera sees. `--planner direct` drives in a straight line instead.

**Each frame:**
1. Every tag except our own robot's and the corner tags becomes an obstacle: a circle with `--puck-radius-mm` (35) for pucks and `--other-robot-radius-mm` (110) for robots.
2. D* Lite plans on a grid of `--dstar-cell-mm` (25 mm) cells. It grows each obstacle by our `--robot-radius-mm` (110), plus half a cell of margin, so the path is a path for the robot's center.
3. The robot steers at the path point `--lookahead-mm` (120) ahead. Its speed is set by how much path is left.

D* Lite keeps its search between frames and only repairs what changed. Re-planning took about 5 ms per frame in tests, with rare spikes to about 110 ms when an obstacle moves a lot.

**In the window:**
- the cyan line is the path
- thin red circles are the zones the robot's center stays out of
- the yellow circle is the point it's steering at
- the yellow × is the goal

**When there's no way through:** the status line says "no path found" and the robot stops. If the target is inside an obstacle's zone, it says "target blocked". If the robot is already inside a zone, it backs straight out first.

**Tested** on plain geometry (a point stepping along the plan, with no camera or robot), 1200 random scenes with 1–4 pucks and robots:
- about 94% reached the goal
- about 3% were truly blocked
- about 3% had a gap only slightly wider than the robot, rejected because of the half-cell margin (a smaller `--dstar-cell-mm` lets it try tighter gaps)
- none got stuck
- the worst detour was about 3× the straight-line distance

**Note for the planner's authors:** `DStarPlanner` lets paths use one more row and column than its obstacle map covers when the field size isn't a whole number of cells (903 mm ÷ 25 mm = 36.1, for example). Paths could then run along that edge straight through obstacles. `dstar_nav.py` works around it by using a grid that is a whole number of cells. The planner file itself is unchanged.

## Head and eyes

- **Head:** while the robot drives, its head servo turns to look along the path: 90° is straight ahead, and the servo reaches 30–150°. When the robot stops or arrives, the head returns to straight ahead.
- **Head direction check:** if the head looks the wrong way, check which way `python3 laptop/robot_client.py servo 150` turns it. If 150 turns it to the robot's right, add `--head-sign -1`. `--head-offset 180` is for a head that faces the robot's back. `--no-head` turns this off.
- **Eyes:** the eyes glance around on their own between blinks. That's in the robot firmware, so update it once with `laptop/upload_wifi.sh 1`.

## Parking behind the puck

Press **p**, or start with `--park`:
```bash
python3 pipeline/go_to_point.py --source 1 --park
```

The robot drives to a spot `--standoff-mm` (180 mm) behind the puck and turns to face it, so the puck is right in front, ready to push. "Behind" means opposite the direction it will push: blue robots push toward the orange side (+x, heading 0°) and orange robots toward blue. `--push-heading` sets another direction.

- **Going around the puck:** if driving straight to the spot would hit the puck, it goes around on a ring of waypoints, taking the shorter side that isn't blocked by a wall.
- **Following:** it re-plans every frame and keeps following the puck as it moves. Once parked, it drives again when the park spot moves more than `--follow-resume` × `--tolerance-mm` away (1.5 × 60 = 90 mm). Lower `--follow-resume` follows more closely but twitches more.
- **Puck out of view:** it stops and does nothing until the puck is seen again, then carries on. A 250 ms grace (`--lost-ms`) keeps a single missed frame from making it jerk to a stop.
- **Puck tag:** the puck is tag 30 by default; use `--puck-tag` for another.
- **In the window:**
  - a green arrow shows the push direction
  - "park" marks the spot
  - a thin green ring is the zone the robot keeps out of
  - the yellow target is where it's heading right now, the spot or a detour waypoint
- **Against a wall:** if the puck is closer to a wall than the robot is wide, the spot behind it is pulled inside the field, and the robot parks as close behind it as it can.
- **Standoff:** set `--standoff-mm` so the robot's front just clears the puck: half the robot's length plus the puck's radius, plus a little. Measure from the center of the robot's tag.

Heading: 0° points to the right of the field (+x) and 90° to the top, counterclockwise as seen from the camera. The robot's `turn` is positive clockwise, so the code flips the sign.

## Tuning

| Symptom | Try |
|---|---|
| Hums but doesn't move near the target | Raise `--min-speed` (default 20) |
| Overshoots or circles the target | Lower `--max-speed` (default 35) or `--kp` (default 0.25) |
| Stops short | Lower `--tolerance-mm` (default 60) |
| Never settles, keeps nudging | Raise `--tolerance-mm` or `--heading-tolerance` (default 12°) |
| Parks too close to or too far from the puck | Change `--standoff-mm` (default 180) |
| Says "no path" through gaps it could fit through | Lower `--robot-radius-mm` to your robot's real size, or use a smaller `--dstar-cell-mm` (e.g. 15) |
| Cuts corners close to obstacles | Lower `--lookahead-mm` (default 120) |
| Wobbles side to side when turning | Lower `--turn-kp` or `--max-turn` |
| Pico restarts on hard starts | Keep `--max-speed` at or below 55 on AA batteries |

## Recording a video

Press **r** in the window to start and again to stop, or add `--record` to record from the start. Videos go to `recordings/` in the repo, named with the date and time, and the path prints when one is saved:
```bash
open recordings/go_to_point_20261110-153000.mp4
```

- **What's recorded:** what the window shows (tags, border, target, front arrow, status lines), at real speed. A 20 s run makes a 20 s video even if the camera's frame rate wobbles. The red REC badge appears only in the window, not in the video.
- **`--record-raw`** also saves the plain camera feed. You can run it back through the vision code later, for example `python3 vision/aruco/detect_live.py --source recordings/go_to_point_raw_<time>.mp4`.
- **The live tag viewer** records the same way: press **r** in `vision/aruco/detect_live.py`, or pass `--record`.
- **Format:** H.264 `.mp4`, which plays in QuickTime and browsers and can be sent to a phone. Git ignores `recordings/`.

## Safety

- The robot stops if:
  - it goes more than `--margin-mm` outside the field
  - its tag is unseen for `--lost-ms` (default 250 ms)
  - you press SPACE or q
  - the program errors
- If the program dies, the robot's own 300 ms failsafe stops it.
- Keep `--max-speed` low for the first runs.

## Known limits

- **Tag height:** the tag sits above the floor, so a robot near the edge of the image reads a little farther out than it really is. The corner tags are on the floor.
- **Tag position:** the robot's position is the tag's center. If the tag isn't centered on the robot, the robot's center is offset by that much.
- **Delay:** about 100–200 ms from camera to wheels, so faster speeds overshoot more.
- **Field size:** comes from `vision/aruco/markers.json`. Rerun `vision/aruco/measure_field.py --write` after moving the corner tags.
