"""Write live RealSense frames for the already-running 8765 bridge.

macOS only exposes the D435i infrared camera to a normal user process.
The root color process writes the same files when it is running.
"""

import os
import time
from pathlib import Path

import cv2

OUT = Path("/tmp/heacon-color-live.jpg")
LOG = Path("/tmp/heacon-usb-preview.log")
PID = Path("/tmp/heacon-usb-preview.pid")


def _log(message):
    line = time.strftime("%H:%M:%S ") + message + "\n"
    try:
        LOG.write_text(LOG.read_text() + line if LOG.exists() else line)
    except OSError:
        pass
    print(message, flush=True)


def _once():
    _log("open")
    capture = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
    if not capture.isOpened():
        raise RuntimeError("camera-closed")
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    misses = 0
    written = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                misses += 1
                if misses > 40:
                    raise RuntimeError("camera-lost")
                time.sleep(0.05)
                continue
            misses = 0
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                continue
            temporary = OUT.with_suffix(".part")
            temporary.write_bytes(encoded.tobytes())
            temporary.replace(OUT)
            written += 1
            if written == 1 or written % 100 == 0:
                _log("frames " + str(written))
            time.sleep(0.04)
    finally:
        capture.release()


def main():
    PID.write_text(str(os.getpid()))
    while True:
        try:
            _once()
        except Exception as exc:
            _log("retry " + str(exc))
            time.sleep(1.5)


if __name__ == "__main__":
    main()
