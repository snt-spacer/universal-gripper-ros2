"""
Motor driver for the Universal Gripper.

MKS SERVO42D CAN interface.

This module contains motor-specific logic only.
All physical CAN communication is delegated to can_driver.py.

No ROS 2 logic is used here.
"""

from __future__ import annotations

import threading
import time

from . import can_driver
from . import config


# ============================================================
# MOTOR CONFIGURATION
# ============================================================

MOTOR_ID = config.MOTOR_CAN_ID

HARMONIC_RATIO = config.HARMONIC_RATIO
STEPS_PER_REV = config.ENCODER_CPR

ANGLE_MIN_DEG = config.ANGLE_MIN_DEG
ANGLE_MAX_DEG = config.ANGLE_MAX_DEG

ANGLE_OPEN = config.ANGLE_OPEN_DEG
ANGLE_CLOSED = config.ANGLE_CLOSED_DEG

MAX_OPENING_MM = config.MAX_OPENING_MM
MIN_OPENING_MM = config.MIN_OPENING_MM


# ============================================================
# RUNTIME STATE
# ============================================================

motor_position_output_deg: float = 0.0

_jog_active = False
_jog_direction: str | None = None

_transaction_lock = threading.Lock()


# ============================================================
# INTERNAL CAN HELPERS
# ============================================================

def _send(data_bytes: list[int]) -> bool:
    """Send a motor command through the CAN driver."""
    return can_driver.send(MOTOR_ID, data_bytes)


def _recv_code(
    code: int,
    timeout: float = 0.25,
):
    """
    Wait for a motor response whose first data byte equals `code`.
    """

    deadline = time.monotonic() + timeout

    while True:

        remaining = deadline - time.monotonic()

        if remaining <= 0:
            return None

        msg = can_driver.recv_motor(
            timeout=min(remaining, 0.05)
        )

        if msg is None:
            continue

        if msg.data and msg.data[0] == code:
            return msg


def _rpm_to_speed_param(rpm: float) -> int:
    """
    Convert RPM to the F6 speed parameter.

    Vrpm = speed_param * 6000 / (Mstep * 200)

    Therefore:

    speed_param = Vrpm * Mstep * 200 / 6000
    """

    param = int(
        round(
            rpm
            * config.MOTOR_MSTEP
            * 200
            / 6000
        )
    )

    return max(0, min(1600, param))


# ============================================================
# ENABLE / STOP
# ============================================================

def enable_motor() -> None:
    """Enable the MKS motor driver using F3 01."""

    with _transaction_lock:
        _send([0xF3, 0x01])

    print("[MOTOR] Motor enabled.")


def stop_motor() -> None:
    """
    Immediately stop the motor.

    F4 with zero speed/axis stops position-mode motion.
    F6 with zero speed stops speed-mode motion.
    """

    global _jog_active
    global _jog_direction

    _jog_active = False
    _jog_direction = None

    print("[MOTOR] Sending STOP.")

    with _transaction_lock:

        # Immediate stop in position mode
        _send([
            0xF4,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ])

        # Stop speed mode
        _send([
            0xF6,
            0x00,
            0x00,
            0x00,
        ])


# ============================================================
# SPEED MODE F6
# ============================================================

def _send_speed(
    speed_param: int,
    direction: str,
    accel: int,
) -> None:
    """
    Send MKS F6 speed-mode command.

    Frame:

        F6
        byte2
        byte3
        acceleration

    Direction bit:

        CW  = 0x00
        CCW = 0x80
    """

    speed_param = max(
        0,
        min(1600, int(speed_param))
    )

    accel = max(
        0,
        min(32, int(accel))
    )

    dir_bit = (
        0x80
        if direction == "CCW"
        else 0x00
    )

    byte2 = (
        dir_bit
        | ((speed_param >> 8) & 0x0F)
    )

    byte3 = speed_param & 0xFF

    with _transaction_lock:
        _send([
            0xF6,
            byte2,
            byte3,
            accel,
        ])


# ============================================================
# JOG
# ============================================================

def open_motor() -> None:
    """Start continuous opening motion."""

    global _jog_active
    global _jog_direction

    current = read_ui_output_position()

    if current <= (
        ANGLE_MIN_DEG
        + config.JOG_STOP_MARGIN_DEG
    ):
        print(
            f"[MOTOR] OPEN blocked "
            f"({current:.2f}°) — already at limit."
        )
        return

    _jog_active = True
    _jog_direction = "OPEN"

    speed_param = _rpm_to_speed_param(
        config.SPEED_OPEN_RPM
    )

    _send_speed(
        speed_param,
        config.MOTOR_DIR_OPEN,
        config.ACCEL_NORMAL,
    )

    print(
        f"[MOTOR] Opening at "
        f"{config.SPEED_OPEN_RPM} RPM "
        f"from {current:.2f}°"
    )


def close_motor() -> None:
    """Start continuous closing motion."""

    global _jog_active
    global _jog_direction

    current = read_ui_output_position()

    if current >= (
        ANGLE_MAX_DEG
        - config.JOG_STOP_MARGIN_DEG
    ):
        print(
            f"[MOTOR] CLOSE blocked "
            f"({current:.2f}°) — already at limit."
        )
        return

    _jog_active = True
    _jog_direction = "CLOSE"

    speed_param = _rpm_to_speed_param(
        config.SPEED_CLOSE_RPM
    )

    _send_speed(
        speed_param,
        config.MOTOR_DIR_CLOSE,
        config.ACCEL_NORMAL,
    )

    print(
        f"[MOTOR] Closing at "
        f"{config.SPEED_CLOSE_RPM} RPM "
        f"from {current:.2f}°"
    )


def check_jog_limits() -> None:
    """
    Stop jog motion when a software limit is reached.
    """

    global _jog_active
    global _jog_direction

    if not _jog_active or _jog_direction is None:
        return

    position = read_ui_output_position()

    if (
        _jog_direction == "CLOSE"
        and position >= (
            ANGLE_MAX_DEG
            - config.JOG_STOP_MARGIN_DEG
        )
    ):
        print(
            f"[MOTOR] Upper jog limit reached "
            f"({position:.2f}°)."
        )
        stop_motor()

    elif (
        _jog_direction == "OPEN"
        and position <= (
            ANGLE_MIN_DEG
            + config.JOG_STOP_MARGIN_DEG
        )
    ):
        print(
            f"[MOTOR] Lower jog limit reached "
            f"({position:.2f}°)."
        )
        stop_motor()


# ============================================================
# STATE MACHINE SPEED COMMANDS
# ============================================================

def start_closing_slow() -> None:
    """Start slow closing for contact search."""

    speed_param = _rpm_to_speed_param(
        config.SPEED_CLOSE_RPM
    )

    _send_speed(
        speed_param,
        config.MOTOR_DIR_CLOSE,
        config.ACCEL_SLOW,
    )


def start_opening_slow() -> None:
    """Start slow opening for force adjustment."""

    speed_param = _rpm_to_speed_param(
        config.SPEED_ADJUST_RPM
    )

    _send_speed(
        speed_param,
        config.MOTOR_DIR_OPEN,
        config.ACCEL_SLOW,
    )


def start_closing_with_speed_param(
    speed_param: int,
) -> None:
    """Close using a specific F6 speed parameter."""

    speed_param = max(
        config.PID_MIN_OUTPUT,
        min(
            config.PID_MAX_OUTPUT,
            int(speed_param),
        ),
    )

    _send_speed(
        speed_param,
        config.MOTOR_DIR_CLOSE,
        config.ACCEL_SLOW,
    )


def start_opening_with_speed_param(
    speed_param: int,
) -> None:
    """Open using a specific F6 speed parameter."""

    speed_param = max(
        config.PID_MIN_OUTPUT,
        min(
            config.PID_MAX_OUTPUT,
            int(speed_param),
        ),
    )

    _send_speed(
        speed_param,
        config.MOTOR_DIR_OPEN,
        config.ACCEL_SLOW,
    )


# ============================================================
# ENCODER
# ============================================================

def read_position() -> float:
    """
    Read raw motor position using MKS command 0x30.

    Response:

        0x30
        carry : signed int32
        value : unsigned uint16
        CRC

    Total encoder ticks:

        carry * STEPS_PER_REV + value

    Motor angle:

        ticks / STEPS_PER_REV * 360

    Output-shaft angle:

        motor angle / HARMONIC_RATIO
    """

    global motor_position_output_deg

    if can_driver.bus is None:
        return motor_position_output_deg

    with _transaction_lock:

        _send([0x30])

        msg = _recv_code(
            0x30,
            timeout=0.25,
        )

    if msg is None:
        print(
            "[MOTOR] No response to "
            "0x30 position query."
        )
        return motor_position_output_deg

    if len(msg.data) < 7:
        print(
            f"[MOTOR] Short position frame "
            f"({len(msg.data)} bytes)."
        )
        return motor_position_output_deg

    data = msg.data

    carry = int.from_bytes(
        data[1:5],
        byteorder="big",
        signed=True,
    )

    value = int.from_bytes(
        data[5:7],
        byteorder="big",
        signed=False,
    )

    total_ticks = (
        carry * STEPS_PER_REV
        + value
    )

    motor_deg = (
        total_ticks
        / STEPS_PER_REV
        * 360.0
    )

    output_deg = (
        motor_deg
        / HARMONIC_RATIO
    )

    motor_position_output_deg = output_deg

    return output_deg


# ============================================================
# CALIBRATION
# ============================================================

def print_raw_position_for_calibration() -> None:
    """
    Print the raw encoder position for manual calibration.
    """

    raw = read_position()

    print()
    print("========== MOTOR CALIBRATION ==========")
    print(
        f"  RAW POSITION = {raw:.6f} deg"
    )
    print(
        "  Copy this value into "
        "config.ZERO_POSITION_RAW_DEG"
    )
    print("=======================================")
    print()


# ============================================================
# CALIBRATED POSITION
# ============================================================

def read_ui_output_position() -> float:
    """
    Return calibrated gripper output angle.

    0°     = fully open
    221.5° = fully closed
    """

    raw = read_position()

    position = (
        raw
        - config.ZERO_POSITION_RAW_DEG
    )

    position = max(
        ANGLE_MIN_DEG,
        min(
            ANGLE_MAX_DEG,
            position,
        ),
    )

    return position


# ============================================================
# KINEMATICS
# ============================================================

def get_gripper_opening_mm(
    output_deg: float,
) -> float:
    """
    Convert output angle to gripper opening.

    0°     -> 85 mm
    221.5° -> 0 mm
    """

    t = (
        output_deg - ANGLE_OPEN
    ) / (
        ANGLE_CLOSED - ANGLE_OPEN
    )

    t = max(
        0.0,
        min(1.0, t),
    )

    return MAX_OPENING_MM * (
        1.0 - t
    )


def get_current_opening_mm() -> float:
    """Return the current gripper opening."""

    return get_gripper_opening_mm(
        read_ui_output_position()
    )


def opening_mm_to_angle_deg(
    opening_mm: float,
) -> float:
    """Convert desired opening in mm to output angle."""

    opening_mm = max(
        MIN_OPENING_MM,
        min(
            MAX_OPENING_MM,
            opening_mm,
        ),
    )

    t = 1.0 - (
        opening_mm / MAX_OPENING_MM
    )

    return (
        t
        * (ANGLE_CLOSED - ANGLE_OPEN)
        + ANGLE_OPEN
    )


# ============================================================
# POSITION MODE F4
# ============================================================

def _deg_to_rel_axis_ticks(
    delta_output_deg: float,
) -> int:
    """
    Convert output-shaft degrees to motor encoder ticks.
    """

    motor_deg = (
        delta_output_deg
        * HARMONIC_RATIO
    )

    motor_revs = motor_deg / 360.0

    ticks = int(
        round(
            motor_revs
            * STEPS_PER_REV
        )
    )

    return max(
        -8_388_607,
        min(
            8_388_607,
            ticks,
        ),
    )


def _send_goto_rel_axis(
    rel_axis_ticks: int,
    speed_rpm: int = config.SPEED_GOTO_RPM,
    accel: int = config.ACCEL_NORMAL,
) -> None:
    """
    Send MKS F4 relative-position command.

    rel_axis is a signed 24-bit motor encoder tick count.
    """

    speed_rpm = max(
        0,
        min(3000, int(speed_rpm)),
    )

    accel = max(
        0,
        min(32, int(accel)),
    )

    speed_hi = (
        speed_rpm >> 8
    ) & 0xFF

    speed_lo = speed_rpm & 0xFF

    axis_24 = rel_axis_ticks & 0xFFFFFF

    byte2 = (
        axis_24 >> 16
    ) & 0xFF

    byte1 = (
        axis_24 >> 8
    ) & 0xFF

    byte0 = axis_24 & 0xFF

    with _transaction_lock:

        _send([
            0xF4,
            speed_hi,
            speed_lo,
            accel,
            byte2,
            byte1,
            byte0,
        ])


# ============================================================
# GO TO POSITION
# ============================================================

def go_to_position_deg(
    target_ui_deg: float,
    speed_rpm: int = config.SPEED_GOTO_RPM,
    accel: int = config.ACCEL_NORMAL,
    tolerance_deg: float = config.POSITION_TOLERANCE_DEG,
    timeout: float = config.GOTO_TIMEOUT_S,
) -> bool:
    """
    Blocking move to a calibrated gripper angle.
    """

    target_ui_deg = max(
        ANGLE_MIN_DEG,
        min(
            ANGLE_MAX_DEG,
            target_ui_deg,
        ),
    )

    current = read_ui_output_position()

    delta_deg = (
        target_ui_deg - current
    )

    if abs(delta_deg) < 0.1:

        print(
            f"[MOTOR] Already at target "
            f"{target_ui_deg:.2f}°."
        )

        return True

    print(
        f"[MOTOR] Moving "
        f"{current:.2f}° → "
        f"{target_ui_deg:.2f}° "
        f"(Δ={delta_deg:+.2f}°)"
    )

    rel_ticks = _deg_to_rel_axis_ticks(
        delta_deg
    )

    _send_goto_rel_axis(
        rel_ticks,
        speed_rpm,
        accel,
    )

    start = time.monotonic()

    while (
        time.monotonic() - start
        < timeout
    ):

        time.sleep(0.05)

        current = read_ui_output_position()

        error = abs(
            target_ui_deg - current
        )

        if error <= tolerance_deg:

            stop_motor()

            print(
                f"[MOTOR] Target reached: "
                f"{current:.2f}° "
                f"(target={target_ui_deg:.2f}°, "
                f"err={error:.2f}°)"
            )

            return True

        if current >= ANGLE_MAX_DEG:

            stop_motor()

            print(
                "[MOTOR] Emergency: "
                "upper limit reached."
            )

            return False

        if current <= ANGLE_MIN_DEG:

            stop_motor()

            print(
                "[MOTOR] Emergency: "
                "lower limit reached."
            )

            return False

    stop_motor()

    print(
        f"[MOTOR] goto timeout after "
        f"{timeout:.1f}s. "
        f"Current={current:.2f}°, "
        f"target={target_ui_deg:.2f}°"
    )

    return False


# ============================================================
# GO TO OPENING
# ============================================================

def go_to_opening_mm(
    target_mm: float,
    **kwargs,
) -> bool:
    """Move to a desired gripper opening in mm."""

    target_deg = opening_mm_to_angle_deg(
        target_mm
    )

    return go_to_position_deg(
        target_deg,
        **kwargs,
    )


def is_jog_active() -> bool:
    """Return whether a continuous jog command is currently active."""
    return _jog_active
