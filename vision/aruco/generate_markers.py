"""Build a printable PDF of the ArUco markers listed in markers.json.

    python generate_markers.py                # writes aruco_markers.pdf next to this file
    python generate_markers.py -o out.pdf --paper a4

Markers are drawn as vector squares, so they stay sharp at any printer
resolution. Print at "Actual size" / 100% scale, never "Fit to page".
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from reportlab.lib.colors import Color, black, white
from reportlab.lib.pagesizes import A4, letter
from reportlab.lib.units import inch, mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas

HERE = Path(__file__).resolve().parent
GRAY = Color(0.45, 0.45, 0.45)
LIGHT = Color(0.7, 0.7, 0.7)

MARGIN = 0.35 * inch
GAP = 0.25 * inch
LABEL_H = 0.42 * inch   # text block under each marker
FRONT_H = 0.2 * inch    # "FRONT" arrow above robot markers

SECTIONS = [
    ("anchor", "Field anchors"),
    ("robot", "Robots"),
    ("puck", "Pucks"),
    ("spare", "Spares"),
    ("test", "Size test strip"),
]


def aruco_dict(name):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_bits(dictionary, marker_id):
    """Marker as a (n, n) bool array including the black border, True = black."""
    n = dictionary.markerSize + 2
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, n, borderBits=1)
    return img < 128


def draw_marker(c, bits, x, y, size):
    """Draw marker bits with the black square's bottom-left at (x, y)."""
    n = bits.shape[0]
    cell = size / n
    path = c.beginPath()
    for r in range(n):
        row_y = y + (n - 1 - r) * cell
        col = 0
        while col < n:
            if bits[r, col]:
                start = col
                while col < n and bits[r, col]:
                    col += 1
                path.rect(x + start * cell, row_y, (col - start) * cell, cell)
            else:
                col += 1
    c.setFillColor(black)
    c.drawPath(path, stroke=0, fill=1)


def fmt_size(size_mm):
    return f'{size_mm / 25.4:.2f} in / {size_mm:.1f} mm'


def cell_dims(m, cells_per_side):
    size = m["size_mm"] * mm
    quiet = size / cells_per_side  # one module of white, the detector's minimum
    w = size + 2 * quiet
    h = w + LABEL_H + (FRONT_H if m["group"] in ("robot", "spare") else 0)
    return size, quiet, w, h


def draw_cell(c, m, bits, x_left, y_top, cells_per_side, dict_name):
    size, quiet, w, h = cell_dims(m, cells_per_side)
    y = y_top
    if m["group"] in ("robot", "spare") and "robot" in m["label"].lower():
        c.setFillColor(GRAY)
        c.setFont("Helvetica-Bold", 8)
        c.drawCentredString(x_left + w / 2, y - FRONT_H + 4, "▲ FRONT of robot ▲")
    if m["group"] in ("robot", "spare"):
        y -= FRONT_H

    # White quiet zone with a dashed cut line around it.
    c.setFillColor(white)
    c.setStrokeColor(LIGHT)
    c.setLineWidth(0.5)
    c.setDash(3, 3)
    c.rect(x_left, y - w, w, w, stroke=1, fill=1)
    c.setDash()
    draw_marker(c, bits, x_left + quiet, y - w + quiet, size)

    # Label under the cut line.
    c.setFillColor(black)
    c.setFont("Helvetica-Bold", 10)
    c.drawCentredString(x_left + w / 2, y - w - 13, f'ID {m["id"]}  —  {m["label"]}')
    c.setFillColor(GRAY)
    c.setFont("Helvetica", 7.5)
    c.drawCentredString(x_left + w / 2, y - w - 24,
                        f'{dict_name}  ·  black square {fmt_size(m["size_mm"])}')


def page_header(c, page_w, page_h, title, dict_name):
    c.setFillColor(black)
    c.setFont("Helvetica-Bold", 13)
    c.drawString(MARGIN, page_h - MARGIN - 10, f"Robot Duels ArUco markers — {title}")
    c.setFont("Helvetica", 8)
    c.setFillColor(GRAY)
    c.drawRightString(page_w - MARGIN, page_h - MARGIN - 10,
                      f"{dict_name} · print at 100% / Actual size")
    return page_h - MARGIN - 0.45 * inch


def scale_check(c, page_w):
    """100 mm and 3 in rulers at the page bottom, to confirm the printer did not scale."""
    y = MARGIN + 6
    c.setStrokeColor(black)
    c.setLineWidth(0.8)
    for x0, length, text in ((MARGIN, 100 * mm, "this line should measure exactly 100 mm"),
                             (page_w - MARGIN - 3 * inch, 3 * inch, "this line should measure exactly 3 in")):
        c.line(x0, y, x0 + length, y)
        c.line(x0, y - 4, x0, y + 4)
        c.line(x0 + length, y - 4, x0 + length, y + 4)
        c.setFillColor(GRAY)
        c.setFont("Helvetica", 7)
        c.drawString(x0, y + 6, text)


def layout_section(c, page_w, page_h, title, items, dictionary, dict_name):
    """Flow cells left to right, wrapping rows and pages. Returns pages used."""
    cps = dictionary.markerSize + 2
    usable_w = page_w - 2 * MARGIN
    bottom = MARGIN + 0.4 * inch  # room for the scale check
    pages = 0

    def new_page():
        nonlocal pages
        if pages:
            c.showPage()
        pages += 1
        top = page_header(c, page_w, page_h, title if pages == 1 else f"{title} (cont.)", dict_name)
        scale_check(c, page_w)
        return top

    y_top = new_page()
    row, row_w = [], 0.0

    def flush(row, y_top):
        row_h = max(cell_dims(m, cps)[3] for m in row)
        if y_top - row_h < bottom:
            y_top = new_page()
        total_w = sum(cell_dims(m, cps)[2] for m in row) + GAP * (len(row) - 1)
        x = MARGIN + (usable_w - total_w) / 2
        for m in row:
            draw_cell(c, m, marker_bits(dictionary, m["id"]), x, y_top, cps, dict_name)
            x += cell_dims(m, cps)[2] + GAP
        return y_top - row_h - GAP

    for m in items:
        w = cell_dims(m, cps)[2]
        if w > usable_w:
            raise SystemExit(f'ID {m["id"]}: {fmt_size(m["size_mm"])} is too wide for this paper')
        if row and row_w + GAP + w > usable_w:
            y_top = flush(row, y_top)
            row, row_w = [], 0.0
        row_w += (GAP if row else 0) + w
        row.append(m)
    if row:
        flush(row, y_top)
    c.showPage()
    return pages


def cover_page(c, page_w, page_h, cfg, groups, subset=None):
    markers = [m for m in cfg["markers"] if subset is None or m["id"] in subset]
    y = page_h - MARGIN - 14
    c.setFont("Helvetica-Bold", 18)
    c.drawString(MARGIN, y, "Robot Duels — ArUco " + ("vision test set" if subset else "marker set"))
    y -= 18
    c.setFont("Helvetica", 9)
    c.setFillColor(GRAY)
    c.drawString(MARGIN, y, f'Dictionary {cfg["dictionary"]}. ID map: vision/aruco/markers.json '
                            "(the generator and detection scripts both read it).")
    c.setFillColor(black)

    y -= 26
    c.setFont("Helvetica-Bold", 11)
    c.drawString(MARGIN, y, "Printing and mounting")
    steps = [
        'Print at 100% / "Actual size". Never "Fit to page". Check the rulers at the bottom of each page.',
        "Measure the black square of a few markers. If it is off, put the measured size in markers.json.",
        "Use matte paper. Glue to card or foam board so it stays flat. Laminate only with a matte finish.",
        "Cut on the dashed line. Never cut inside it: the white margin is part of the marker.",
        "Mount on a light surface or keep the white margin. A dark robot top right at the black edge breaks detection.",
        "Robots: mark the front before trimming. The marker's top edge must face the robot's forward direction.",
        "Each ID may appear only once in the camera view. Keep spares off the field.",
    ]
    c.setFont("Helvetica", 9)
    for i, s in enumerate(steps, 1):
        y -= 14
        c.drawString(MARGIN + 6, y, f"{i}.  {s}")

    y -= 26
    c.setFont("Helvetica-Bold", 11)
    c.drawString(MARGIN, y, "ID map")
    y -= 4
    cols = [MARGIN + 6, MARGIN + 50, MARGIN + 120, MARGIN + 320]
    y -= 14
    c.setFont("Helvetica-Bold", 8.5)
    for x, t in zip(cols, ["ID", "Group", "Label", "Black square"]):
        c.drawString(x, y, t)
    c.setLineWidth(0.4)
    c.setStrokeColor(LIGHT)
    c.line(MARGIN, y - 4, page_w - MARGIN, y - 4)
    c.setFont("Helvetica", 8.5)
    for m in sorted(markers, key=lambda m: m["id"]):
        y -= 12
        for x, t in zip(cols, [str(m["id"]), m["group"], m["label"], fmt_size(m["size_mm"])]):
            c.drawString(x, y, t)
    if subset is None:
        used = {m["id"] for m in cfg["markers"]}
        free = [str(i) for i in range(50) if i not in used]
        y -= 16
        c.setFillColor(GRAY)
        c.drawString(MARGIN + 6, y, "Unassigned (free for later): " + ", ".join(free))

    y -= 22
    c.setFillColor(black)
    c.setFont("Helvetica-Bold", 11)
    c.drawString(MARGIN, y, "Groups")
    c.setFont("Helvetica", 8.5)
    for key, _ in SECTIONS:
        if not any(m["group"] == key for m in markers):
            continue
        y -= 13
        c.setFont("Helvetica-Bold", 8.5)
        c.drawString(MARGIN + 6, y, key)
        c.setFont("Helvetica", 8.5)
        c.drawString(MARGIN + 50, y, groups[key])
    c.showPage()


def charuco_page(c, page_w, page_h, board_cfg):
    d = aruco_dict(board_cfg["dictionary"])
    nx, ny = board_cfg["squares_x"], board_cfg["squares_y"]
    sq, mk = board_cfg["square_mm"], board_cfg["marker_mm"]
    board = cv2.aruco.CharucoBoard((nx, ny), sq / 1000, mk / 1000, d)
    px_per_mm = 24  # about 600 dpi
    img = board.generateImage((int(nx * sq * px_per_mm), int(ny * sq * px_per_mm)), marginSize=0)
    rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    top = page_header(c, page_w, page_h, "Camera calibration board (ChArUco)", board_cfg["dictionary"])
    w, h = nx * sq * mm, ny * sq * mm
    x = (page_w - w) / 2
    y = top - h - 0.1 * inch
    c.drawImage(ImageReader(_to_pil(rgb)), x, y, w, h)
    c.setFillColor(black)
    c.setFont("Helvetica", 8.5)
    lines = [
        f'{nx} x {ny} squares, square {sq:.1f} mm, marker {mk:.1f} mm, dictionary {board_cfg["dictionary"]}.',
        "Glue to something rigid and flat. Measure one square edge to edge and use that value when calibrating.",
        "Take 15–20 photos at the camera resolution you will use: tilted, near the edges and corners of the frame.",
    ]
    for i, s in enumerate(lines):
        c.drawString(MARGIN, y - 14 - 12 * i, s)
    scale_check(c, page_w)
    c.showPage()


def _to_pil(rgb):
    from PIL import Image
    return Image.fromarray(rgb)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-c", "--config", default=HERE / "markers.json", type=Path)
    ap.add_argument("-o", "--out", default=HERE / "aruco_markers.pdf", type=Path)
    ap.add_argument("--paper", choices=["letter", "a4"], default="letter")
    ap.add_argument("--ids", help="only these IDs, e.g. 0,1,2,3,10,30. Non-anchor markers share pages. "
                                  "Leaves out the calibration board.")
    args = ap.parse_args()

    cfg = json.loads(args.config.read_text())
    ids = [m["id"] for m in cfg["markers"]]
    if len(ids) != len(set(ids)):
        raise SystemExit("markers.json has duplicate IDs")
    dictionary = aruco_dict(cfg["dictionary"])

    sections = [(key, title, [m for m in cfg["markers"] if m["group"] == key]) for key, title in SECTIONS]
    subset = None
    if args.ids:
        subset = {int(i) for i in args.ids.split(",")}
        unknown = subset - set(ids)
        if unknown:
            raise SystemExit(f"IDs not in markers.json: {sorted(unknown)}")
        picked = [m for m in cfg["markers"] if m["id"] in subset]
        sections = [("anchor", "Arena", [m for m in picked if m["group"] == "anchor"]),
                    ("other", "Test markers", [m for m in picked if m["group"] != "anchor"])]

    page_w, page_h = letter if args.paper == "letter" else A4
    c = canvas.Canvas(str(args.out), pagesize=(page_w, page_h))
    c.setTitle("Robot Duels ArUco markers")
    cover_page(c, page_w, page_h, cfg, cfg["groups"], subset)
    for _, title, items in sections:
        if items:
            layout_section(c, page_w, page_h, title, items, dictionary, cfg["dictionary"])
    if subset is None and "calibration_board" in cfg:
        charuco_page(c, page_w, page_h, cfg["calibration_board"])
    c.save()
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
