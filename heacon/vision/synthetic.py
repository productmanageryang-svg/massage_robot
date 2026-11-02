"""A synthetic camera frame, used only when HEACON_SIMULATE=1.

Without this, a machine with no RealSense attached cannot get past the
"no person in the camera" guard, so the service logic could never be
exercised in CI or on a fresh Ubuntu box. The frame is a plausible overhead
view of a prone body: it is not a photograph and must never be used to judge
recognition quality, only to let the pipeline run end to end.
"""

from __future__ import annotations

import numpy as np

WIDTH = 1280
HEIGHT = 720


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
