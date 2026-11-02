"""Tuned numbers for the route planner, in one place, with the reasons kept.

Every value here was chosen on the real arm, so changing one changes how the
machine feels under a hand. Grouped by what it controls:

* geometry       — where the tool sits relative to skin and the camera
* motion         — speed, acceleration and override limits
* gestures       — the shape of each massage technique
* force_control  — admittance, force limits and sensor handling
"""

from __future__ import annotations

# --- geometry --------------------------------------------------------------

#: Flange height above the calibrated table. Approved on site as 40 mm from skin.
WORK_LIFT_MM = -30.0
#: Allow the 16 mm press / peck dip under the 40 mm standoff, plus a little noise.
WORK_FLOOR_MM = WORK_LIFT_MM - 22.0

#: Pose for the 就位 step, taught on site 2026-10-29. The camera looks down at
#: the prone person from here, and the arm returns here on that step.
POSITION_JOINTS = [7.106, 14.041, -30.604, -0.111, -134.191, 4.470]
POSITION_POSE = {
    "x": -458.314,
    "y": -56.893,
    "z": 838.693,
    "rx": -179.987,
    "ry": -1.167,
    "rz": -177.441,
}

#: Same pose. A cropped view returns here instead of backing up in small steps.
OVERVIEW_POSE = {
    "x": POSITION_POSE["x"],
    "y": POSITION_POSE["y"],
    "z": POSITION_POSE["z"],
}
OVERVIEW_JOINTS = POSITION_JOINTS

#: Flange angles whose camera looks straight down. Tilting away faults a joint.
VERTICAL_ANGLES = {
    "rx": POSITION_POSE["rx"],
    "ry": POSITION_POSE["ry"],
    "rz": POSITION_POSE["rz"],
}

ASSUMED_BACK_LENGTH_MM = 420.0
ASSUMED_BODY_WIDTH_MM = 240.0

#: Semantic gaps from skin. The live flange pose at WORK_LIFT_MM is 40 mm.
SKIN_STANDOFF_MM = 40.0
MASSAGE_STANDOFF_MM = SKIN_STANDOFF_MM
#: Moxa sits 30 mm above the acupoint.
MOXA_STANDOFF_MM = 30.0
MOXA_CLEARANCE_MM = MOXA_STANDOFF_MM

MAX_TRAVEL_MM = 650.0
TRAVEL_CLEARANCE_MM = 50.0
HIGH_ABOVE_HOVER_MM = 80.0

# --- motion ----------------------------------------------------------------

#: Visible technique size. The low point is the approved work height.
LIFT_MM = 20.0
GRADE_STEP_MM = 10.0
SPEED_MM_S = 120
SPEED_CAP_MM_S = 180
ACC_MM_S2 = 80
HOLD_SECONDS = 25.0
OVERRIDE = 0.2
OVERRIDE_CAP = 0.50
VERTICAL_PACE = 3

# --- gestures --------------------------------------------------------------

TILT_DEG = 10.0
TWIST_DEG = 15.0
CIRCLE_MM = 25.0
SWEEP_MM = 30.0
DIP_MM = 16.0
GESTURE_LIMIT_DEG = 16.0

# --- force control ---------------------------------------------------------

HYBRID_YIELD_XY = 80.0
HYBRID_SPEED_MM_S = 260
HYBRID_FORCE_N = 8.0

#: Z search stays slow; XY yield uses the same linear cap.
HYBRID_SEARCH_MM_S = 12.0
HYBRID_SEARCH_DEG_S = 2.0
HYBRID_MASSAGE_SEARCH_DEG_S = 5.0
HYBRID_XY_SETTLE_MM = 8.0
HYBRID_XY_DONE_MM = 22.0
HYBRID_FORCE_FLOOR_MM = 50.0
HYBRID_MOXA_FLOOR_MM = 20.0
HYBRID_MOXA_SETTLE_MM = 2.5
HYBRID_MOXA_XY_DONE_MM = 5.0
HYBRID_MOXA_Z_DONE_MM = 4.0
HYBRID_TILT_YIELD_DEG = 10.0

#: Skip SetForceZero if a hand is already on the tool; zeroing under load
#: inverts the force reading and the arm lunges.
HYBRID_MOXA_ZERO_MAX_N = 5.0
#: Protocol maxima. The default FT protection band is tight enough that a
#: shove faults the controller, which the operator reads as the arm stopping.
HYBRID_MOXA_FORCE_LIMIT_N = 500.0
HYBRID_MOXA_TORQUE_LIMIT_NM = 50.0
HYBRID_MOXA_COLLIDE = 5

#: Massage: strategy 0, Z tracks force, XY are a soft spring around the path.
#: Moxa: strategy 1 as admittance around the commanded acupoint.
HYBRID_XY_MASS = 40.0
HYBRID_Z_MASS = 40.0
HYBRID_XY_DAMP = 400.0
HYBRID_Z_DAMP = 200.0
HYBRID_XY_STIFF = 280.0
HYBRID_Z_STIFF = 80.0
HYBRID_RX_STIFF = 90.0
HYBRID_RZ_STIFF = 400.0
HYBRID_RX_DAMP = 28.0
HYBRID_RZ_DAMP = 40.0
HYBRID_MOXA_MASS = 20.0
HYBRID_MOXA_XY_STIFF = 80.0
HYBRID_MOXA_Z_STIFF = 80.0
HYBRID_MOXA_XY_DAMP = 90.0
HYBRID_MOXA_Z_DAMP = 90.0
HYBRID_MOXA_ROT_STIFF = 80.0
HYBRID_MOXA_SEARCH_MM_S = 40.0

#: Huayan 柔顺 has no pose spring (its goal distance is unused). The spring is
#: implemented here as F_goal = K * error, so K is in newtons per millimetre.
HYBRID_MOXA_K_N_PER_MM = 0.28
HYBRID_MOXA_D_N_PER_MM_S = 0.06
HYBRID_MOXA_F_MAX_N = 16.0
HYBRID_MASSAGE_K_N_PER_MM = 0.22
HYBRID_MASSAGE_D_N_PER_MM_S = 0.05
HYBRID_MASSAGE_F_MAX_N = 14.0
HYBRID_MASSAGE_K_ROT = 0.12
HYBRID_MASSAGE_D_ROT = 0.03
HYBRID_MASSAGE_M_MAX = 2.0
HYBRID_MASSAGE_SEARCH_MM_S = 45.0
HYBRID_MASSAGE_ARRIVE_MM = 5.0
