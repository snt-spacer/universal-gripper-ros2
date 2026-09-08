from __future__ import annotations

import threading
import time

import numpy as np

from . import config
from . import can_driver


# =====================================================
# SENSOR CONFIG
# =====================================================

TEXEL_IDS: list = config.TEXEL_IDS
NUM_TEXELS: int = config.NUM_TEXELS

_id_to_index: dict = {
    tid: i for i, tid in enumerate(TEXEL_IDS)
}

RES_XY: float = config.SENSOR_RES_XY
RES_Z: float = config.SENSOR_RES_Z
ALPHA: float = config.SENSOR_ALPHA
WINDOW_SIZE: int = config.SENSOR_WINDOW

_MAX_DRAIN_PER_TICK: int = 60


# =====================================================
# FORCE VALUES
# =====================================================

Fx: np.ndarray = np.zeros(NUM_TEXELS)
Fy: np.ndarray = np.zeros(NUM_TEXELS)
Fz: np.ndarray = np.zeros(NUM_TEXELS)


# =====================================================
# CALIBRATION OFFSETS
# =====================================================

_x0: np.ndarray = np.zeros(NUM_TEXELS)
_y0: np.ndarray = np.zeros(NUM_TEXELS)
_z0: np.ndarray = np.zeros(NUM_TEXELS)


# =====================================================
# ROLLING HISTORY
# =====================================================

fx_hist: np.ndarray = np.zeros(
    (NUM_TEXELS, WINDOW_SIZE)
)

fy_hist: np.ndarray = np.zeros(
    (NUM_TEXELS, WINDOW_SIZE)
)

fz_hist: np.ndarray = np.zeros(
    (NUM_TEXELS, WINDOW_SIZE)
)


# =====================================================
# SYNCHRONISATION
# =====================================================

_data_lock = threading.Lock()

_calibrating = threading.Event()


# =====================================================
# DECODE
# =====================================================

def _decode_axis(msb: int, lsb: int) -> int:
    """
    Convert two raw bytes to signed offset-binary counts.
    Centre point = 32768.
    """
    return ((msb << 8) | lsb) - 32768


def _parse_sensor(data: bytes) -> tuple:
    """
    Decode a 7-byte XELA sensor frame.

    Returns:
        (x, y, z) raw counts
    """
    return (
        _decode_axis(data[1], data[2]),
        _decode_axis(data[3], data[4]),
        _decode_axis(data[5], data[6]),
    )


# =====================================================
# CALIBRATION
# =====================================================

def calibrate(
    num_samples: int = config.SENSOR_CALIB_SAMPLES,
    timeout_s: float = 30.0,
) -> bool:
    """
    Collect sensor samples with no load and calculate
    median zero offsets for every texel.

    Returns:
        True  -> calibration successful
        False -> bus unavailable or timeout
    """

    global _x0, _y0, _z0

    if can_driver.bus is None:
        print(
            "[SENSORS] Bus unavailable — "
            "calibration skipped, offsets zeroed."
        )
        return False

    _calibrating.set()

    try:
        print("[SENSORS] Calibrating — keep sensors unloaded...")
        print(
            f"[SENSORS] Waiting for {num_samples} samples "
            f"from {NUM_TEXELS} sensors "
            f"(timeout: {timeout_s}s)"
        )

        samples: list[list] = [
            [] for _ in range(NUM_TEXELS)
        ]

        start_time = time.monotonic()
        last_progress_time = start_time

        while min(len(s) for s in samples) < num_samples:

            elapsed = time.monotonic() - start_time

            if elapsed > timeout_s:
                print(
                    f"\n[SENSORS] Calibration timeout "
                    f"after {elapsed:.1f}s!"
                )

                print("[SENSORS] Sensor status:")

                for i, tid in enumerate(TEXEL_IDS):
                    count = len(samples[i])

                    status = (
                        f"{count}/{num_samples}"
                        if count > 0
                        else "NO FRAMES"
                    )

                    print(
                        f"  Sensor {tid} "
                        f"(idx {i}): {status}"
                    )

                print("\n[SENSORS] Troubleshooting:")
                print("  1. Verify sensor power and CAN connections")
                print(
                    "  2. Check that sensor CAN IDs in "
                    "config match hardware"
                )
                print(
                    "  3. Verify that the CAN interface "
                    "is configured correctly"
                )

                return False

            if (
                time.monotonic() - last_progress_time
                > 5.0
            ):
                min_samples = min(
                    len(s) for s in samples
                )

                print(
                    f"[SENSORS] Progress: "
                    f"{min_samples}/{num_samples} samples... "
                    f"({elapsed:.1f}s)"
                )

                last_progress_time = time.monotonic()

            msg = can_driver.recv_sensor(timeout=0.5)

            if msg is None:
                continue

            tid = msg.arbitration_id

            if tid not in _id_to_index:
                continue

            if len(msg.data) < 7:
                continue

            i = _id_to_index[tid]

            samples[i].append(
                _parse_sensor(msg.data)
            )

        with _data_lock:

            for i in range(NUM_TEXELS):

                arr = np.array(samples[i])

                _x0[i], _y0[i], _z0[i] = (
                    np.median(arr, axis=0)
                )

        print("[SENSORS] Calibration complete.")

        return True

    finally:
        _calibrating.clear()


# =====================================================
# UPDATE
# =====================================================

def update_sensors() -> None:
    """
    Drain pending sensor frames and update filtered
    force values.

    Intended to be called periodically by the ROS 2
    hardware interface.
    """

    global Fx, Fy, Fz

    if _calibrating.is_set():
        return

    for _ in range(_MAX_DRAIN_PER_TICK):

        msg = can_driver.recv_sensor(timeout=0)

        if msg is None:
            break

        tid = msg.arbitration_id

        if tid not in _id_to_index:
            continue

        if len(msg.data) < 7:
            continue

        i = _id_to_index[tid]

        x, y, z = _parse_sensor(msg.data)

        fx_raw = (
            (x - _x0[i])
            * RES_XY
        )

        fy_raw = (
            (y - _y0[i])
            * RES_XY
        )

        fz_raw = max(
            0.0,
            (z - _z0[i]) * RES_Z
        )

        with _data_lock:

            Fx[i] = (
                ALPHA * fx_raw
                + (1.0 - ALPHA) * Fx[i]
            )

            Fy[i] = (
                ALPHA * fy_raw
                + (1.0 - ALPHA) * Fy[i]
            )

            Fz[i] = (
                ALPHA * fz_raw
                + (1.0 - ALPHA) * Fz[i]
            )

            fx_hist[i] = np.roll(
                fx_hist[i], -1
            )
            fx_hist[i, -1] = Fx[i]

            fy_hist[i] = np.roll(
                fy_hist[i], -1
            )
            fy_hist[i, -1] = Fy[i]

            fz_hist[i] = np.roll(
                fz_hist[i], -1
            )
            fz_hist[i, -1] = Fz[i]


# =====================================================
# PUBLIC FORCE READERS
# =====================================================

def get_avg_fz() -> float:
    """
    Return average normal force across all texels.
    """

    with _data_lock:
        return float(np.mean(Fz))


def get_max_fz() -> float:
    """
    Return maximum normal force across all texels.
    """

    with _data_lock:
        return float(np.max(Fz))


def get_forces() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Return thread-safe copies of Fx, Fy and Fz.
    """

    with _data_lock:
        return (
            Fx.copy(),
            Fy.copy(),
            Fz.copy(),
        )


def get_calibration_offsets() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Return copies of the current calibration offsets.
    """

    with _data_lock:
        return (
            _x0.copy(),
            _y0.copy(),
            _z0.copy(),
        )
