"""Read an image, find the ArUco tags, and write a copy with each tag and the field grid marked.

    python mark_tags.py photo.jpg                     # writes photo_tags.png
    python mark_tags.py photo.jpg -o out.png
    python mark_tags.py photo.jpg --save-matrix m.npy # also save the occupancy matrix (.npy or .csv)
    python mark_tags.py photo.jpg --cell-mm 25        # choose the grid cell size yourself
    python mark_tags.py photo.jpg --no-grid           # tags only

Each tag gets an outline, a dot at its center, an arrow toward its printed top
edge (the robot's FRONT), and its ID and label from markers.json. Tags that are
not in markers.json are outlined in red. When the 4 corner tags are visible, the
field border, the matrix grid and the cells each tag covers are drawn too, and
each robot's field position is printed (see field_grid.py).

From other code: find_tags(image), draw_tags(image, tags), or mark_tags(image).
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
CONFIG = json.loads((HERE / "markers.json").read_text())
KNOWN = {m["id"]: m for m in CONFIG["markers"]}
DETECTOR = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, CONFIG["dictionary"])),
    cv2.aruco.DetectorParameters(),
)
# BGR colors per marker group.
COLORS = {"anchor": (0, 200, 0), "robot": (255, 120, 0), "puck": (0, 190, 255)}
UNKNOWN = (0, 0, 255)


def find_tags(image):
    """Returns a list of (id, 4x2 corner array). Corner 0 -> 1 is the printed top edge."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, ids, _ = DETECTOR.detectMarkers(gray)
    if ids is None:
        return []
    return [(int(i), c[0]) for i, c in zip(ids.ravel(), corners)]


def draw_tags(out, tags):
    """Draws each tag's outline, center, heading arrow and label onto out (in place)."""
    # Scale line and text thickness with the image so it stays readable on large photos.
    s = max(out.shape[:2]) / 1000
    thick = max(2, round(2 * s))
    for tag_id, pts in tags:
        info = KNOWN.get(tag_id)
        color = COLORS.get(info["group"], (200, 200, 200)) if info else UNKNOWN
        center = pts.mean(axis=0)
        # Arrow from the center past the top edge, so the heading shows even on small tags.
        tip = center + 1.8 * ((pts[0] + pts[1]) / 2 - center)

        cv2.polylines(out, [pts.astype(np.int32)], True, color, thick)
        cv2.circle(out, tuple(center.astype(int)), thick * 2, color, -1)
        cv2.arrowedLine(out, tuple(center.astype(int)), tuple(tip.astype(int)), color, thick, tipLength=0.3)

        text = f'{tag_id} {info["label"]}' if info else f"{tag_id} ?"
        font, scale = cv2.FONT_HERSHEY_SIMPLEX, 0.6 * s
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick + 3)
        # Label under the tag (above it at the bottom of the image), kept inside the image.
        x = int(np.clip(pts[:, 0].min(), 2, out.shape[1] - tw - 2))
        y = pts[:, 1].max() + th + 10 * s
        if y > out.shape[0] - 2:
            y = pts[:, 1].min() - 10 * s
        y = int(np.clip(y, th + 2, out.shape[0] - 2))
        cv2.putText(out, text, (x, y), font, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
        cv2.putText(out, text, (x, y), font, scale, color, thick, cv2.LINE_AA)
    return out


def mark_tags(image):
    """Returns (annotated copy of image, list of (id, corners)). Tags only, no grid."""
    out = image.copy() if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    tags = find_tags(image)
    return draw_tags(out, tags), tags


def save_matrix(matrix, path):
    if path.suffix == ".csv":
        np.savetxt(path, matrix, fmt="%d", delimiter=",")
    else:
        np.save(path, matrix)


def main():
    from field_grid import FieldGrid

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("images", nargs="+", type=Path)
    ap.add_argument("-o", "--out", type=Path, help="output path (only with one input image)")
    ap.add_argument("--no-grid", action="store_true", help="mark tags only")
    ap.add_argument("--cell-mm", type=float, help="grid cell size in mm (default: from camera resolution)")
    ap.add_argument("--save-matrix", type=Path, help="save the occupancy matrix (.npy or .csv; one image only)")
    args = ap.parse_args()
    if len(args.images) > 1 and (args.out or args.save_matrix):
        ap.error("-o and --save-matrix work with one input image")

    for path in args.images:
        image = cv2.imread(str(path))
        if image is None:
            print(f"{path}: could not read image")
            continue
        tags = find_tags(image)
        out = image.copy()
        print(f"{path}: {len(tags)} tags: " + ", ".join(
            f'{i} ({KNOWN[i]["label"]})' if i in KNOWN else f"{i} (unknown)" for i, _ in sorted(tags, key=lambda t: t[0])))

        if not args.no_grid:
            grid = FieldGrid(args.cell_mm)
            used = grid.update(tags)
            if grid.ready:
                matrix = grid.matrix(tags)
                rows, cols = matrix.shape
                print(f"  field from {used} corner tags; matrix {rows} x {cols}, cell {grid.cell_mm:g} mm "
                      f"(camera resolves {grid.mm_per_px:.1f} mm/px at worst)")
                for d in grid.describe(tags):
                    where = f'cell [{d["row"]}, {d["col"]}]' if d["on_field"] else "OFF FIELD"
                    print(f'  {d["id"]} {d["label"]}: x {d["x_mm"]:.0f} mm ({d["x_mm"] / 25.4:.1f} in), '
                          f'y {d["y_mm"]:.0f} mm ({d["y_mm"] / 25.4:.1f} in), {where}, heading {d["heading_deg"]:.0f} deg')
                if args.save_matrix:
                    save_matrix(matrix, args.save_matrix)
                    print(f"  matrix saved to {args.save_matrix}")
            else:
                matrix = None
                print("  no field grid: need 4 corner tags in view")
            grid.draw(out, matrix, tags)

        draw_tags(out, tags)
        if not args.no_grid:
            out = grid.readout(out, tags)
        dest = args.out or path.with_name(f"{path.stem}_tags.png")
        cv2.imwrite(str(dest), out)
        print(f"  -> {dest}")


if __name__ == "__main__":
    main()
