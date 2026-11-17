"""Live tag view: reads a camera or phone stream and marks each ArUco tag (see mark_tags.py).

    python detect_live.py --list                       # find which number is your camera / iPhone
    python detect_live.py --source 1                   # camera number (iPhone via Continuity Camera)
    python detect_live.py --source http://192.168.1.23:4747/video   # phone streaming app (DroidCam etc.)
    python detect_live.py --source clip.mp4            # a recorded video also works

Keys: q quits, g toggles the field grid, s saves the marked-up frame, the raw frame
and the occupancy matrix as snapshot_<n>*, r starts/stops recording a video.

Recording: r (or --record) saves what the window shows as an .mp4 in recordings/
(repo root), at real speed. --record-raw also saves the plain camera feed, which can
be played back through this script with --source <file>.

The field grid needs the 4 corner tags once; after that it keeps the last good
mapping while robots block corners (see field_grid.py).

iPhone + Mac: Continuity Camera makes the iPhone a normal camera, so use its number
from --list. Any phone: a webcam-streaming app gives a URL to pass to --source.
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from field_grid import FieldGrid
from mark_tags import KNOWN, draw_tags, find_tags
from recorder import Recorder

REPO = Path(__file__).resolve().parents[2]


def open_source(source, width, height):
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if source.isdigit():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # Keep only the newest frame so a network stream does not fall behind.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def list_cameras(max_index=6):
    print("Checking camera numbers (the iPhone is usually 1 or 2 when Continuity Camera is on)...")
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        if ok:
            print(f"  {i}: working, {frame.shape[1]}x{frame.shape[0]}")
        cap.release()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0", help="camera number, stream URL, or video file (default 0)")
    ap.add_argument("--list", action="store_true", help="list working camera numbers and exit")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--max-window", type=int, default=1280, help="shrink the preview window to this width")
    ap.add_argument("--cell-mm", type=float, help="grid cell size in mm (default: from camera resolution)")
    ap.add_argument("--no-grid", action="store_true", help="start with the field grid hidden")
    ap.add_argument("--record", action="store_true", help="start recording right away (or press r)")
    ap.add_argument("--record-raw", action="store_true", help="also record the plain camera feed")
    ap.add_argument("--record-dir", type=Path, default=REPO / "recordings", help="where videos go")
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return

    cap = open_source(args.source, args.width, args.height)
    if not cap.isOpened():
        raise SystemExit(f"could not open {args.source!r}. Try --list, or check the phone app's URL.")

    snapshots, last, fps = 0, time.time(), 0.0
    grid, show_grid, matrix = FieldGrid(args.cell_mm), not args.no_grid, None
    rec = Recorder(args.record_dir, "tags")
    rec_raw = Recorder(args.record_dir, "tags_raw") if args.record_raw else None

    def toggle_recording():
        recording, path = rec.toggle()
        raw = rec_raw.toggle()[1] if rec_raw else None
        if recording:
            print(f"recording to {path}")
        else:
            for p in (path, raw):
                if p:
                    print(f"saved {p}   (open it with:  open {p})")

    if args.record:
        toggle_recording()
    while True:
        ok, frame = cap.read()
        if not ok:
            print("stream ended or dropped")
            break
        tags = find_tags(frame)
        out = frame.copy()
        grid.update(tags)
        matrix = grid.matrix(tags) if grid.ready else None
        if show_grid:
            grid.draw(out, matrix, tags)
        draw_tags(out, tags)
        if show_grid:
            out = grid.readout(out, tags)

        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - last, 1e-6)
        last = now
        h, w = out.shape[:2]
        names = " ".join(str(i) for i, _ in sorted(tags, key=lambda t: t[0]))
        status = f"{w}x{h}  {fps:4.1f} fps  tags: {len(tags)}  [{names}]"
        cv2.putText(out, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(out, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

        view = out
        if w > args.max_window:
            view = cv2.resize(out, (args.max_window, round(h * args.max_window / w)))
        rec.write(view)
        if rec_raw:
            rec_raw.write(frame)
        cv2.imshow("tags (q quits, g grid, s saves, r records)", rec.badge(view.copy()) if rec.active else view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("g"):
            show_grid = not show_grid
        if key == ord("r"):
            toggle_recording()
        if key == ord("s"):
            snapshots += 1
            path = Path(f"snapshot_{snapshots}.png")
            cv2.imwrite(str(path), out)
            cv2.imwrite(str(path.with_name(f"snapshot_{snapshots}_raw.png")), frame)
            if matrix is not None:
                np.save(path.with_name(f"snapshot_{snapshots}_matrix.npy"), matrix)
            print(f"saved {path}, the unmarked frame and the matrix; tags: {[KNOWN.get(i, {}).get('label', i) for i, _ in tags]}")
    if rec.active:
        toggle_recording()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
