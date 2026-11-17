"""Turn camera pixels into field coordinates and an occupancy matrix, using the corner tags.

The corner tags' centers are matched to their field_xy_mm in markers.json, which
gives a perspective transform (homography) from the image to the flat field. So
markers.json must hold the real distances between your corner tag centers.

Grid resolution: by default one matrix cell is the size of one camera pixel on the
field at the worst spot (usually the far edge), rounded up to a round number of mm.
A coarser grid would throw away position accuracy the camera has, and a finer one
adds nothing the camera can see. Pass cell_mm to pick your own size.

Matrix values: -1 = empty, otherwise the ID of the tag covering that cell.
matrix[row, col]: row runs along field y (down the image), col along field x.
"""

import math

import cv2
import numpy as np

from mark_tags import CONFIG, KNOWN

ROUND_MM = [1, 1.5, 2, 2.5, 3, 4, 5, 10, 20, 25, 50, 100, 200]
ANCHORS = {i: m["field_xy_mm"] for i, m in KNOWN.items() if m["group"] == "anchor" and "field_xy_mm" in m}
FIELD_W = CONFIG["field"]["width_mm"]
FIELD_H = CONFIG["field"]["height_mm"]
GROUP_BGR = {"robot": (255, 120, 0), "puck": (0, 190, 255), "spare": (200, 0, 200), "test": (180, 180, 180)}
BORDER_BGR = (255, 0, 255)


class FieldGrid:
    def __init__(self, cell_mm=None):
        self.H = None          # image px -> field mm
        self.cell_mm = cell_mm  # locked after the first estimate so the matrix shape stays fixed
        self.mm_per_px = None

    # ---- mapping -------------------------------------------------------------
    def update(self, tags):
        """Re-fit the mapping from the corner tags in view. Keeps the last good one if
        fewer than 4 are visible. Returns the number of corner tags used (0 if not refit)."""
        found = [(pts.mean(axis=0), ANCHORS[i]) for i, pts in tags if i in ANCHORS]
        if len(found) < 4:
            return 0
        img = np.float32([p for p, _ in found])
        field = np.float32([f for _, f in found])
        H, _ = cv2.findHomography(img, field, 0)
        if H is None:
            return 0
        self.H = H
        self.mm_per_px = self._worst_mm_per_px()
        if self.cell_mm is None:
            self.cell_mm = next((r for r in ROUND_MM if r >= self.mm_per_px), math.ceil(self.mm_per_px))
        return len(found)

    @property
    def ready(self):
        return self.H is not None

    def to_field(self, pts):
        return cv2.perspectiveTransform(np.float32(pts).reshape(-1, 1, 2), self.H).reshape(-1, 2)

    def to_image(self, pts):
        return cv2.perspectiveTransform(np.float32(pts).reshape(-1, 1, 2), np.linalg.inv(self.H)).reshape(-1, 2)

    def _worst_mm_per_px(self):
        """Largest field distance one image pixel spans anywhere on the field."""
        xs, ys = np.meshgrid(np.linspace(0, FIELD_W, 21), np.linspace(0, FIELD_H, 21))
        img = self.to_image(np.stack([xs.ravel(), ys.ravel()], axis=1))
        worst = 0.0
        for p in img:
            f0, fx, fy = self.to_field([p, p + (1, 0), p + (0, 1)])
            jac = np.stack([fx - f0, fy - f0], axis=1)
            worst = max(worst, np.linalg.svd(jac, compute_uv=False)[0])
        return float(worst)

    # ---- matrix --------------------------------------------------------------
    @property
    def shape(self):
        return math.ceil(FIELD_H / self.cell_mm), math.ceil(FIELD_W / self.cell_mm)

    def matrix(self, tags):
        """Occupancy matrix: -1 empty, else the ID of the tag covering the cell."""
        m = np.full(self.shape, -1, np.int32)
        shift = 4
        for tag_id, pts in tags:
            if tag_id in ANCHORS:
                continue
            # Field mm -> cell coordinates (OpenCV puts pixel centers on integers).
            cells = self.to_field(pts) / self.cell_mm - 0.5
            cv2.fillPoly(m, [np.round(cells * (1 << shift)).astype(np.int32)], int(tag_id), shift=shift)
        return m

    def describe(self, tags):
        """Field position, matrix cell and heading for each non-corner tag."""
        out = []
        rows, cols = self.shape
        for tag_id, pts in sorted(tags, key=lambda t: t[0]):
            if tag_id in ANCHORS:
                continue
            center, front = self.to_field([pts.mean(axis=0), (pts[0] + pts[1]) / 2])
            # 0 deg = +x (toward the orange side), 90 deg = toward the top of the field.
            heading = math.degrees(math.atan2(-(front[1] - center[1]), front[0] - center[0])) % 360
            row, col = int(center[1] // self.cell_mm), int(center[0] // self.cell_mm)
            out.append({
                "id": tag_id,
                "label": KNOWN[tag_id]["label"] if tag_id in KNOWN else "unknown",
                "x_mm": float(center[0]), "y_mm": float(center[1]),
                "row": row, "col": col,
                "on_field": 0 <= row < rows and 0 <= col < cols,
                "heading_deg": heading,
            })
        return out

    # ---- drawing -------------------------------------------------------------
    def draw(self, image, matrix, tags):
        """Draw the field border, grid lines and occupied cells onto image (in place)."""
        h, w = image.shape[:2]
        s = max(h, w) / 1000
        if not self.ready:
            return image

        Hinv = np.linalg.inv(self.H)
        # Occupied cells, painted by warping the matrix itself onto the image.
        color = np.zeros(matrix.shape + (3,), np.uint8)
        for tag_id in np.unique(matrix[matrix >= 0]):
            group = KNOWN.get(int(tag_id), {}).get("group")
            color[matrix == tag_id] = GROUP_BGR.get(group, (0, 0, 255))
        c = self.cell_mm
        cell_to_img = Hinv @ np.array([[c, 0, c / 2], [0, c, c / 2], [0, 0, 1]])
        painted = cv2.warpPerspective(color, cell_to_img, (w, h), flags=cv2.INTER_NEAREST)
        mask = painted.any(axis=2)
        image[mask] = (0.45 * image[mask] + 0.55 * painted[mask]).astype(np.uint8)

        # Grid lines: every cell if cells are big enough on screen, else every k-th line.
        rows, cols = self.shape
        px_per_cell = self.cell_mm / self.mm_per_px
        step = next((k for k in (1, 2, 5, 10, 20, 50, 100, 200, 500) if k * px_per_cell >= 18), 1000)
        lines = image.copy()
        thin = max(1, round(s))
        for col in range(0, cols + 1, step):
            x = min(col * c, FIELD_W)
            a, b = self.to_image([(x, 0), (x, FIELD_H)])
            cv2.line(lines, tuple(np.int32(a)), tuple(np.int32(b)), (255, 255, 255), thin, cv2.LINE_AA)
        for row in range(0, rows + 1, step):
            y = min(row * c, FIELD_H)
            a, b = self.to_image([(0, y), (FIELD_W, y)])
            cv2.line(lines, tuple(np.int32(a)), tuple(np.int32(b)), (255, 255, 255), thin, cv2.LINE_AA)
        cv2.addWeighted(lines, 0.45, image, 0.55, 0, dst=image)

        # Border through the four field corners.
        border = self.to_image([(0, 0), (FIELD_W, 0), (FIELD_W, FIELD_H), (0, FIELD_H)])
        cv2.polylines(image, [np.int32(border)], True, BORDER_BGR, max(3, round(3 * s)), cv2.LINE_AA)

        return image

    def readout(self, image, tags):
        """Returns image with a text strip added below it: grid size and each tag's position."""
        s = max(image.shape[:2]) / 1000
        if not self.ready:
            lines = ["Field grid: need 4 corner tags in view (IDs " + ", ".join(map(str, sorted(ANCHORS))) + ")"]
        else:
            rows, cols = self.shape
            c = self.cell_mm
            lines = [f"Matrix {rows} x {cols}, 1 cell = {c:g} mm ({c / 25.4:.2f} in); "
                     f"camera: {self.mm_per_px:.1f} mm/px worst case"]
            for d in self.describe(tags):
                where = f'cell [{d["row"]}, {d["col"]}]' if d["on_field"] else "OFF FIELD"
                lines.append(f'{d["id"]} {d["label"]}: x {d["x_mm"] / 25.4:.1f} in, y {d["y_mm"] / 25.4:.1f} in, '
                             f'{where}, heading {d["heading_deg"]:.0f} deg')
        font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.5 * s, max(1, round(s))
        line_h = int(24 * s)
        panel = np.zeros((line_h * len(lines) + int(12 * s), image.shape[1], 3), np.uint8)
        for i, t in enumerate(lines):
            cv2.putText(panel, t, (int(8 * s), (i + 1) * line_h), font, scale, (255, 255, 255), thick, cv2.LINE_AA)
        return np.vstack([image, panel])
