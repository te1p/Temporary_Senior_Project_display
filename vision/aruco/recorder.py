"""Record frames to an .mp4 that plays back at real speed.

Cameras (especially phones over Wi-Fi) deliver frames unevenly, so frames are written
at a steady rate: each new frame is repeated until the video catches up with the
clock. A 20 s run gives a 20 s video, whatever the camera's frame rate was.

    rec = Recorder("recordings", prefix="go_to_point")
    rec.start()            # or rec.toggle() from a key press
    rec.write(frame)       # every loop; does nothing while not recording
    path = rec.stop()      # returns the .mp4 path

Videos are H.264 (plays in QuickTime, browsers and phones). If this OpenCV has no
H.264 encoder, they fall back to MPEG-4 Part 2, which VLC plays.
"""

import time
from datetime import datetime
from pathlib import Path

import cv2


class Recorder:
    def __init__(self, folder, prefix="run", fps=30.0):
        self.folder = Path(folder)
        self.prefix = prefix
        self.fps = fps
        self.writer = None
        self.path = None
        self.size = None
        self.t0 = 0.0
        self.written = 0
        self.armed = False

    @property
    def active(self):
        return self.armed

    @property
    def elapsed(self):
        return time.monotonic() - self.t0 if self.armed else 0.0

    def start(self):
        """Start a new recording. The file opens on the first frame (to learn its size)."""
        if self.armed:
            return self.path
        self.folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.path = self.folder / f"{self.prefix}_{stamp}.mp4"
        self.armed = True
        self.writer = None
        return self.path

    def _open(self, frame):
        h, w = frame.shape[:2]
        self.size = (w - w % 2, h - h % 2)  # H.264 needs even dimensions
        for fourcc in ("avc1", "mp4v"):
            writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*fourcc), self.fps, self.size)
            if writer.isOpened():
                self.writer = writer
                break
        else:
            self.armed = False
            raise RuntimeError(f"could not open a video writer for {self.path}")
        self.t0 = time.monotonic()
        self.written = 0

    def write(self, frame):
        if not self.armed:
            return
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if self.writer is None:
            self._open(frame)
        if (frame.shape[1], frame.shape[0]) != self.size:
            frame = self._fit(frame)
        # Write this frame as many times as needed to keep the video in step with the clock.
        # (If frames arrive faster than fps, extra ones are skipped.)
        due = int((time.monotonic() - self.t0) * self.fps) + 1
        for _ in range(due - self.written):
            self.writer.write(frame)
            self.written += 1

    def _fit(self, frame):
        """Scale to fit the video's size without stretching, padding the rest with black."""
        W, H = self.size
        h, w = frame.shape[:2]
        k = min(W / w, H / h)
        small = cv2.resize(frame, (min(W, max(1, round(w * k))), min(H, max(1, round(h * k)))))
        top, left = (H - small.shape[0]) // 2, (W - small.shape[1]) // 2
        return cv2.copyMakeBorder(small, top, H - small.shape[0] - top, left, W - small.shape[1] - left,
                                  cv2.BORDER_CONSTANT)

    def stop(self):
        """Finish the file. Returns its path, or None if nothing was recorded."""
        path = self.path if self.writer is not None else None
        if self.writer is not None:
            self.writer.release()
        self.writer = None
        self.armed = False
        return path

    def toggle(self):
        """Start or stop. Returns (now_recording, path)."""
        if self.armed:
            return False, self.stop()
        return True, self.start()

    def badge(self, image):
        """Draw a red REC dot and the elapsed time in the top-right corner (in place)."""
        if not self.armed:
            return image
        w = image.shape[1]
        s = max(image.shape[:2]) / 1000
        r = max(6, round(9 * s))
        cx, cy = w - round(150 * s), round(32 * s)
        if int(self.elapsed * 2) % 2 == 0:  # blink
            cv2.circle(image, (cx, cy), r, (0, 0, 255), -1, cv2.LINE_AA)
        secs = int(self.elapsed)
        text = f"REC {secs // 60:02d}:{secs % 60:02d}"
        org = (cx + r + round(8 * s), cy + round(8 * s))
        cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7 * s, (0, 0, 0), max(3, round(5 * s)), cv2.LINE_AA)
        cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7 * s, (255, 255, 255), max(1, round(2 * s)),
                    cv2.LINE_AA)
        return image


def frame_count_and_duration(path):
    """For checking a finished recording: (frames, seconds)."""
    cap = cv2.VideoCapture(str(path))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    cap.release()
    return n, (n / fps if fps else float("nan"))
