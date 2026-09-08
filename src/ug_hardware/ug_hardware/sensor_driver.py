"""
Tactile sensor driver for the Universal Gripper.

XELA uSPa11 tactile sensors over CAN.

This module contains sensor-specific logic only.
All physical CAN communication is delegated to can_driver.py.

No ROS 2 logic is used here.

Threading model
---------------
can_driver.py owns the physical CAN bus and runs a single dispatcher
thread. Sensor frames are routed into a dedicated sensor queue.

This module reads only from can_driver.recv_sensor(), so motor reply
frames cannot be consumed by the sensor driver.

A threading.Lock protects the force arrays and history buffers.

A threading.Event prevents update_sensors() from draining the sensor
queue while calibration is running.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from . import can_driver
from . import config


# ============================================================
# SENSOR CONFIGURATION
# ============================================================

TEXEL_IDS: list[int] = config.TEXEL_IDS
NUM_TEXELS: int = config.NUM_TEXELS

_id_to_index: dict[int, int] = {
    texel_id: index
    for index, texel_id in enumerate(TEXEL_IDS)
}

RES_XY: float = config.SENSOR_RES_XY
RES_Z: float = config.SENSOR_RES_Z
ALPHA: float = config.SENSOR_ALPHA
WINDOW_SIZE: int = config.SENSOR_WINDOW


# Maximum number of sensor frames processed during one update.
#
# This prevents a high CAN sensor rate from monopolizing the
# thread calling update_sensors().
_MAX_DRAIN_PER_TICK: int = 60


# ============================================================
# FORCE VALUES
# ============================================================
#
# Filtered force values in Newtons.
#
# Access must be protected by _data_lock.
#

Fx: np.ndarray = np.zeros(NUM_TEXELS)
Fy: np.ndarray = np.zeros(NUM_TEXELS)
Fz: np.ndarray = np.zeros(NUM_TEXELS)


# ============================================================
# CALIBRATION OFFSETS
# ============================================================
#
# Raw sensor values measured with no load.
#

_x0: np.ndarray = np.zeros(NUM_TEXELS)
_y0: np.ndarray = np.zeros(NUM_TEXELS)
_z0: np.ndarray = np.zeros(NUM_TEXELS)


# ============================================================
# ROLLING HISTORY
# ============================================================
#
# Used later by the UI / plotting layer.
#

fx_hist: np.ndarray = np.zeros(
    (NUM_TEXELS, WINDOW_SIZE)
)

fy_hist: np.ndarray = np.zeros(
    (NUM_TEXELS, WINDOW_SIZE)
)

fz_hist: np.ndarray = np.zeros(
    (NUM_TEXELS, WINDOW_SIZE)
)


# ============================================================
# THREAD SYNCHRONIZATION
# ============================================================

_data_lock = threading.Lock()

# SET   -> calibration is running
# CLEAR -> normal operation
_calibrating = threading.Event()


# ============================================================
# DECODE HELPERS
# ============================================================

def _decode_axis(msb: int, lsb: int) -> int:
    """
    Convert two raw bytes to a signed sensor value.

    XELA encoding is offset-binary with the center at 32768.
    """

    return (
        ((msb << 8) | lsb)
        - 32768
    )


def _parse_sensor(data: bytes) -> tuple[int, int, int]:
    """
    Decode a 7-byte XELA sensor frame.

    Frame layout:

        byte 0 : sensor/status information
        byte 1 : X MSB
        byte 2 : X LSB
        byte 3 : Y MSB
        byte 4 : Y LSB
        byte 5 : Z MSB
        byte 6 : Z LSB
    """

    return (
        _decode_axis(data[1], data[2]),
        _decode_axis(data[3], data[4]),
        _decode_axis(data[5], data[6]),
    )


# ============================================================
# CALIBRATION
# ============================================================

def calibrate(
    num_samples: int = config.SENSOR_CALIB_SAMPLES,
    timeout_s: float = 30.0,
) -> bool:
    """
    Calibrate all tactile sensors.

    The sensors must be unloaded during calibration.

    For each Texel, num_samples frames are collected and the
    median X/Y/Z value is stored as the zero-load offset.

    Returns:
        True  -> calibration successful
        False -> bus unavailable or timeout
    """

    global _x0
    global _y0
    global _z0

    if can_driver.bus is None:

        print(
            "[SENSORS] Bus unavailable — "
            "calibration skipped."
        )

        return False

    # Prevent update_sensors() from consuming frames
    # while calibration is collecting them.
    _calibrating.set()

    try:

        print(
            "[SENSORS] Calibrating — "
            "keep sensors unloaded..."
        )

        print(
            f"[SENSORS] Waiting for "
            f"{num_samples} samples from "
            f"{NUM_TEXELS} sensors "
            f"(timeout: {timeout_s}s)"
        )

        samples: list[list[tuple[int, int, int]]] = [
            []
            for _ in range(NUM_TEXELS)
        ]

        start_time = time.monotonic()
        last_progress_time = start_time

        while min(
            len(sensor_samples)
            for sensor_samples in samples
        ) < num_samples:

            elapsed = (
                time.monotonic()
                - start_time
            )

            # ------------------------------------------------
            # TIMEOUT
            # ------------------------------------------------

            if elapsed > timeout_s:

                print(
                    f"\n[SENSORS] Calibration timeout "
                    f"after {elapsed:.1f}s!"
                )

                print(
                    "[SENSORS] Sensor status:"
                )

                for index, texel_id in enumerate(
                    TEXEL_IDS
                ):

                    count = len(
                        samples[index]
                    )

                    if count > 0:
                        status = (
                            f"{count}/{num_samples}"
                        )
                    else:
                        status = "NO FRAMES"

                    print(
                        f"  Sensor {texel_id} "
                        f"(idx {index}): "
                        f"{status}"
                    )

                print(
                    "\n[SENSORS] Troubleshooting:"
                )

                print(
                    "  1. Verify sensor power "
                    "and CAN connections"
                )

                print(
                    "  2. Check sensor CAN IDs "
                    "in config.py"
                )

                print(
                    "  3. Verify the CAN "
                    "interface configuration"
                )

                return False

            # ------------------------------------------------
            # PROGRESS
            # ------------------------------------------------

            if (
                time.monotonic()
                - last_progress_time
                > 5.0
            ):

                min_samples = min(
                    len(sensor_samples)
                    for sensor_samples in samples
                )

                print(
                    f"[SENSORS] Progress: "
                    f"{min_samples}/"
                    f"{num_samples} samples..."
                    f" ({elapsed:.1f}s)"
                )

                last_progress_time = (
                    time.monotonic()
                )

            # ------------------------------------------------
            # RECEIVE SENSOR FRAME
            # ------------------------------------------------

            msg = can_driver.recv_sensor(
                timeout=0.5
            )

            if msg is None:
                continue

            texel_id = msg.arbitration_id

            if texel_id not in _id_to_index:
                continue

            if len(msg.data) < 7:
                continue

            index = _id_to_index[
                texel_id
            ]

            samples[index].append(
                _parse_sensor(msg.data)
            )

        # ----------------------------------------------------
        # CALCULATE MEDIAN OFFSETS
        # ----------------------------------------------------

        with _data_lock:

            for index in range(NUM_TEXELS):

                array = np.array(
                    samples[index]
                )

                (
                    _x0[index],
                    _y0[index],
                    _z0[index],
                ) = np.median(
                    array,
                    axis=0,
                )

        print(
            "[SENSORS] Calibration complete."
        )

        return True

    finally:

        # Always re-enable normal sensor updates,
        # including when calibration fails or raises.
        _calibrating.clear()


# ============================================================
# SENSOR UPDATE
# ============================================================

def update_sensors() -> None:
    """
    Drain pending sensor frames and update filtered forces.

    Intended to be called periodically by the higher-level
    application layer.

    The function is non-blocking.

    At most _MAX_DRAIN_PER_TICK frames are processed per call.
    """

    global Fx
    global Fy
    global Fz

    # Do not consume sensor frames while calibration owns
    # the sensor queue.
    if _calibrating.is_set():
        return

    for _ in range(
        _MAX_DRAIN_PER_TICK
    ):

        # Non-blocking receive.
        msg = can_driver.recv_sensor(
            timeout=0
        )

        if msg is None:
            break

        texel_id = msg.arbitration_id

        if texel_id not in _id_to_index:
            continue

        if len(msg.data) < 7:
            continue

        index = _id_to_index[
            texel_id
        ]

        x, y, z = _parse_sensor(
            msg.data
        )

        # ----------------------------------------------------
        # RAW → FORCE
        # ----------------------------------------------------

        fx_raw = (
            x - _x0[index]
        ) * RES_XY

        fy_raw = (
            y - _y0[index]
        ) * RES_XY

        # Normal force cannot be negative.
        fz_raw = max(
            0.0,
            z - _z0[index],
        ) * RES_Z

        # ----------------------------------------------------
        # FILTER + HISTORY
        # ----------------------------------------------------

        with _data_lock:

            Fx[index] = (
                ALPHA * fx_raw
                + (1.0 - ALPHA) * Fx[index]
            )

            Fy[index] = (
                ALPHA * fy_raw
                + (1.0 - ALPHA) * Fy[index]
            )

            Fz[index] = (
                ALPHA * fz_raw
                + (1.0 - ALPHA) * Fz[index]
            )

            fx_hist[index] = np.roll(
                fx_hist[index],
                -1,
            )

            fx_hist[index, -1] = Fx[index]

            fy_hist[index] = np.roll(
                fy_hist[index],
                -1,
            )

            fy_hist[index, -1] = Fy[index]

            fz_hist[index] = np.roll(
                fz_hist[index],
                -1,
            )

            fz_hist[index, -1] = Fz[index]


# ============================================================
# FORCE ACCESSORS
# ============================================================

def get_avg_fz() -> float:
    """
    Return the average normal force across all Texels.

    Thread-safe.
    """

    with _data_lock:
        return float(
            np.mean(Fz)
        )


def get_max_fz() -> float:
    """
    Return the maximum normal force across all Texels.

    Thread-safe.
    """

    with _data_lock:
        return float(
            np.max(Fz)
        )


def get_forces() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Return a thread-safe snapshot of Fx, Fy and Fz.

    Copies are returned so callers cannot modify the internal
    driver state accidentally.
    """

    with _data_lock:

        return (
            Fx.copy(),
            Fy.copy(),
            Fz.copy(),
        )


def get_calibration_offsets() -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """
    Return a thread-safe snapshot of the calibration offsets.
    """

    with _data_lock:

        return (
            _x0.copy(),
            _y0.copy(),
            _z0.copy(),
        )
