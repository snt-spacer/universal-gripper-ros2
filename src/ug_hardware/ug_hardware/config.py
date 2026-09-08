"""
config.py
---------
Centralized configuration for the gripper system.
All constants, limits, and tuning parameters live here.
Do NOT scatter magic numbers across other modules.
"""

# =====================================================
# CAN BUS
# =====================================================
CAN_INTERFACE   = "slcan"
CAN_CHANNEL     = "/dev/ttyACM0"
CAN_BITRATE     = 1000000  

# =====================================================
# MOTOR IDENTITY
# =====================================================
MOTOR_CAN_ID    = 0x01
MOTOR_MSTEP     = 16        # Subdivision setting on the motor (default 16)

# =====================================================
# MECHANICAL CONSTANTS
# =====================================================
HARMONIC_RATIO  = 20.0      # Harmonic drive reduction ratio
ENCODER_CPR     = 0x4000    # Encoder counts per motor revolution (16384)

# =====================================================
# GRIPPER LIMITS  (output shaft / gripper frame)
# =====================================================
# 0°     = fully open  (0 cm aperture)
# 221.5° = fully closed (0 cm aperture)
ANGLE_OPEN_DEG      = 0.0
ANGLE_CLOSED_DEG    = 221.5
ANGLE_MIN_DEG       = 0.0       # Hard software lower limit
ANGLE_MAX_DEG       = 221.5     # Hard software upper limit

MAX_OPENING_MM      = 85.0      # Physical maximum aperture in mm
MIN_OPENING_MM      = 0.0

# =====================================================
# CALIBRATION
# =====================================================
# RAW encoder position (degrees) read when the gripper
# is FULLY OPEN during manual calibration.
#
# Calibration procedure:
#   1. Fully open the gripper manually.
#   2. Run:  motor_controller.print_raw_position_for_calibration()
#   3. Copy the printed value here.
#
# THIS VALUE MUST NOT BE CHANGEABLE FROM THE UI.
ZERO_POSITION_RAW_DEG =0 # 41.676636 #98.02

# =====================================================
# MOTOR DIRECTION CONVENTION
# =====================================================
# Physical wiring determines which CAN direction closes the gripper.
# Set these to match your physical setup.
# "CW" or "CCW"
MOTOR_DIR_OPEN  = "CW"     # Direction that opens  (decreases output angle toward 0°)
MOTOR_DIR_CLOSE = "CCW"    # Direction that closes (increases output angle toward 221.5°)

# =====================================================
# MOTION PARAMETERS
# =====================================================
# F4/F5 commands use direct RPM (0–3000).
# F6 command uses speed_param (0–1600); Vrpm = param * 6000 / (Mstep * 200)

SPEED_OPEN_RPM      = 50    # RPM for opening moves (F4 command)
SPEED_CLOSE_RPM     = 50    # RPM for search-contact closing (F6 speed_param)
SPEED_GOTO_RPM      = 50    # RPM for go-to-position moves (F4 command)
SPEED_ADJUST_RPM    = 50    # RPM for fine-force-adjustment moves (F6 speed_param)
ACCEL_NORMAL        = 5     # Normal acceleration (0–32)
ACCEL_SLOW          = 3     # Slow acceleration for sensitive moves

# =====================================================
# POSITION TOLERANCES
# =====================================================
POSITION_TOLERANCE_DEG  = 2.0   # go_to_position tolerance (±degrees)
APERTURE_TOLERANCE_MM   = 5.0   # Aperture tolerance (±mm)

# JOG watchdog — stop before hitting the hard limit
JOG_STOP_MARGIN_DEG     = 1.5   # Stop jog this many degrees before limit

# =====================================================
# TIMEOUTS
# =====================================================
GOTO_TIMEOUT_S          = 20.0  # Max time allowed for a go-to move
STARTUP_OPEN_TIMEOUT_S  = 25.0  # Timeout for initial open at startup
CAN_RECV_TIMEOUT_S      = 0.20  # Default CAN receive timeout

# =====================================================
# CONTACT / FORCE THRESHOLDS
# =====================================================
CONTACT_FZ_THRESHOLD_N  = 0.3   # Fz to enter CONTACT from CLOSE
CONTACT_MIN_FZ_N        = 0.2   # Fz below which object is considered lost (CONTACT)
HOLD_TARGET_FZ_N        = 1.5   # Desired Fz during HOLD
HOLD_TOLERANCE_N        = 0.4   # ±band around HOLD_TARGET_FZ_N
HOLD_STABLE_TIME_S      = 0.3   # Time Fz must stay in-band before going to HOLD
HOLD_MIN_FZ_N           = 0.15  # Fz below which object is lost (HOLD)
HOLD_MAX_FZ_N           = 4.0   # Fz above which we re-enter CONTACT to reajust
SAFETY_FZ_LIMIT_N       = 8.0   # Absolute Fz safety limit → SAFETY state

# =====================================================
# PID  (contact.py force control)
# =====================================================
PID_KP  = 0.05   # Proportional gain  (speed_param per Newton of error)
PID_KI  = 0.01   # Integral gain
PID_KD  = 0.002  # Derivative gain
PID_MAX_OUTPUT  = 60    # Max speed_param output from PID (clipped)
PID_MIN_OUTPUT  = 5     # Min non-zero speed_param

# =====================================================
# STATE MACHINE
# =====================================================
SM_LOOP_HZ = 20.0   # State machine update frequency

# =====================================================
# SENSOR CONFIG
# =====================================================
TEXEL_IDS       = [258, 274, 18, 2]
NUM_TEXELS      = len(TEXEL_IDS)
SENSOR_RES_XY   = 1.53  / 8033     # N / raw_count  (shear axes)
SENSOR_RES_Z    = 7.8   / 19099    # N / raw_count  (normal axis)
SENSOR_ALPHA    = 0.15              # EMA filter coefficient
SENSOR_WINDOW   = 160              # Rolling plot history length
SENSOR_CALIB_SAMPLES = 100         # Calibration sample count
