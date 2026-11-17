"""Measure the field from a photo, using the printed size of the corner tags as the ruler.

    python measure_field.py photo.png              # print the field size
    python measure_field.py photo.png --write      # also put it in markers.json
    python measure_field.py a.png b.png --write    # average several photos

Only the 4 corner tags (IDs 0-3) are used: they must lie flat on the floor, all in
view. The field is the rectangle through their centers. Their black squares must
really be the size_mm in markers.json: check one with a ruler, and pass
--tag-mm with the measured size if the printer scaled them.

How: the 4 tag centers fix the perspective up to the field's width and height.
Each tag edge, which is known to be size_mm long, then gives one equation in
width^2 and height^2, and 16 edges are solved by least squares.
"""

import argparse
import re
from pathlib import Path

import cv2
import numpy as np

from mark_tags import HERE, KNOWN, find_tags

CORNERS = [0, 1, 2, 3]                      # top-left, top-right, bottom-right, bottom-left
UNIT = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])


def measure(image, tag_mm):
    tags = dict(find_tags(image))
    missing = [i for i in CORNERS if i not in tags]
    if missing:
        raise ValueError(f"corner tags not found: {missing}")
    centers = np.float32([tags[i].mean(axis=0) for i in CORNERS])
    to_unit = cv2.getPerspectiveTransform(centers, UNIT)

    rows, rhs = [], []
    for i in CORNERS:
        quad = cv2.perspectiveTransform(tags[i].reshape(-1, 1, 2).astype(np.float32), to_unit).reshape(-1, 2)
        size = tag_mm or KNOWN[i]["size_mm"]
        for k in range(4):
            du, dv = quad[(k + 1) % 4] - quad[k]
            rows.append([du * du, dv * dv])
            rhs.append(size * size)
    (w2, h2), *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)
    if w2 <= 0 or h2 <= 0:
        raise ValueError("could not solve; are all 4 corner tags flat on the floor?")
    w, h = float(np.sqrt(w2)), float(np.sqrt(h2))

    # Check: each tag edge length implied by this answer, vs the printed size.
    implied = np.sqrt(np.array(rows) @ np.array([w2, h2]))
    spread = float(np.std(implied) / np.mean(implied) * 100)
    return w, h, spread


def write_config(width, height):
    path = HERE / "markers.json"
    text = path.read_text()
    w, h = round(width), round(height)
    text = re.sub(r'"width_mm": [\d.]+', f'"width_mm": {w}', text, count=1)
    text = re.sub(r'"height_mm": [\d.]+', f'"height_mm": {h}', text, count=1)
    positions = {0: [0, 0], 1: [w, 0], 2: [w, h], 3: [0, h], 4: [round(w / 2), 0], 5: [round(w / 2), h]}
    for tag_id, xy in positions.items():
        text = re.sub(rf'(\{{"id": {tag_id},[^\n]*"field_xy_mm": )\[[^\]]*\]', rf"\g<1>[{xy[0]}, {xy[1]}]", text)
    path.write_text(text)
    print(f"markers.json updated: field {w} x {h} mm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("images", nargs="+", type=Path)
    ap.add_argument("--tag-mm", type=float, help="measured black-square size of the corner tags, if not as printed")
    ap.add_argument("--write", action="store_true", help="save the result to markers.json")
    args = ap.parse_args()

    results = []
    for path in args.images:
        image = cv2.imread(str(path))
        if image is None:
            print(f"{path}: could not read image")
            continue
        try:
            w, h, spread = measure(image, args.tag_mm)
        except ValueError as e:
            print(f"{path}: {e}")
            continue
        results.append((w, h))
        print(f"{path}: {w:.0f} x {h:.0f} mm ({w / 25.4:.1f} x {h / 25.4:.1f} in), "
              f"tag edges agree within {spread:.1f}%")
    if not results:
        raise SystemExit("no usable photos")

    w, h = np.mean(results, axis=0)
    print(f"field (between corner tag centers): {w:.0f} x {h:.0f} mm = {w / 25.4:.1f} x {h / 25.4:.1f} in")
    if args.write:
        write_config(w, h)


if __name__ == "__main__":
    main()
