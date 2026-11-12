"""A synthetic camera frame and depth map, used only when HEACON_SIMULATE=1.

Without these, a machine with no RealSense attached cannot get past the
"no person in the camera" guard, so the service logic could never be exercised
in CI or on a fresh deployment. The frames are plausible, not photographs: they
must never be used to judge recognition quality, only to let the pipeline run.

``synthetic_depth`` matters because acupoint recognition segments the body by
depth (a back is anything more than ~25 mm closer than the table). A flat
colour image carries no such cue, so the simulator supplies a depth map of the
same shape with a raised torso.
"""

from __future__ import annotations

import numpy as np

WIDTH = 1280
HEIGHT = 720

#: Table height and torso height in millimetres on the simulated depth map.
TABLE_MM = 900
TORSO_MM = 700


def synthetic_frame():
    """A BGR frame with one light body on a dark table."""
    frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    frame[:] = (58, 56, 54)  # dark grey table

    # Table grid, so the frame is obviously synthetic.
    frame[::60, :] = (72, 70, 68)
    frame[:, ::60] = (72, 70, 68)

    # Torso: a light vertical band down the middle, head at the top.
    cx = WIDTH // 2
    frame[120:600, cx - 130 : cx + 130] = (168, 172, 178)
    frame[600:690, cx - 110 : cx + 110] = (150, 155, 162)

    # Arms at the sides.
    frame[200:520, cx - 250 : cx - 150] = (160, 164, 170)
    frame[200:520, cx + 150 : cx + 250] = (160, 164, 170)

    # Head, darker, at the top so the body axis is unambiguous.
    yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
    head = (xx - cx) ** 2 + (yy - 90) ** 2 <= 62**2
    frame[head] = (96, 90, 84)

    return frame


def synthetic_depth(shape=None):
    """A uint16 depth map in millimetres matching :func:`synthetic_frame`.

    Table is ``TABLE_MM`` away, the torso ``TORSO_MM``. Recognition treats
    anything closer than the table by more than 25 mm as body.
    """
    height, width = (shape[0], shape[1]) if shape else (HEIGHT, WIDTH)
    depth = np.full((height, width), TABLE_MM, np.uint16)

    cx = width // 2
    sx = width / 1280.0
    sy = height / 720.0

    def span(y0, y1, x0, x1, value):
        ys, ye = int(y0 * sy), int(y1 * sy)
        xs, xe = int(x0 * sx), int(x1 * sx)
        depth[max(0, ys) : max(0, ye), max(0, xs) : max(0, xe)] = value

    span(120, 600, cx - 130, cx + 130, TORSO_MM)   # back
    span(600, 690, cx - 110, cx + 110, TORSO_MM - 20)  # waist
    span(200, 520, cx - 250, cx - 150, TORSO_MM)   # left arm
    span(200, 520, cx + 150, cx + 250, TORSO_MM)   # right arm

    yy, xx = np.mgrid[0:height, 0:width]
    head = (xx - cx) ** 2 + (yy - int(90 * sy)) ** 2 <= (62 * sx) ** 2
    depth[head] = TORSO_MM + 30  # head sits slightly lower than the back

    return depth
