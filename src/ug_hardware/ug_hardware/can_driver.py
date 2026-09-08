"""
CAN driver for the Universal Gripper.

Owns the physical CAN bus and provides:
- SLCAN connection through python-can
- thread-safe CAN transmission
- single receive dispatcher thread
- separate motor and tactile-sensor queues

ROS 2 is intentionally not used here.
"""

from __future__ import annotations

import queue
import threading
import time

import can

from . import config


# ============================================================
# PUBLIC STATE
# ============================================================

bus: can.BusABC | None = None


# ============================================================
# INTERNAL STATE
# ============================================================

_send_lock = threading.Lock()

_motor_queue: queue.Queue[can.Message] = queue.Queue()
_sensor_queue: queue.Queue[can.Message] = queue.Queue()

_dispatcher_thread: threading.Thread | None = None
_dispatcher_running = False


# ============================================================
# DISPATCHER
# ============================================================

def _dispatcher_loop() -> None:
    """
    Read CAN frames from the bus and dispatch them to the
    appropriate consumer queue.

    Only this thread calls bus.recv().
    """

    sensor_ids = set(config.TEXEL_IDS)
    motor_id = config.MOTOR_CAN_ID

    while _dispatcher_running:

        if bus is None:
            time.sleep(0.01)
            continue

        try:
            msg = bus.recv(timeout=0.05)

        except ValueError:
            # Invalid SLCAN frame.
            continue

        except Exception as exc:
            print(f"[CAN DISPATCHER] recv error: {exc}")
            continue

        if msg is None:
            continue

        arbitration_id = msg.arbitration_id

        if arbitration_id in sensor_ids:
            _sensor_queue.put_nowait(msg)

        elif arbitration_id == motor_id:
            _motor_queue.put_nowait(msg)

        # Unknown CAN IDs are intentionally ignored.


# ============================================================
# INITIALIZATION
# ============================================================

def init() -> bool:
    """
    Open the CAN interface and start the dispatcher thread.

    Returns:
        True if the CAN bus was opened successfully.
        False otherwise.
    """

    global bus
    global _dispatcher_thread
    global _dispatcher_running

    if bus is not None:
        print("[CAN] Bus already initialized.")
        return True

    try:
        bus = can.interface.Bus(
            interface=config.CAN_INTERFACE,
            channel=config.CAN_CHANNEL,
            bitrate=config.CAN_BITRATE,
        )

        print(
            f"[CAN] Bus connected on "
            f"{config.CAN_CHANNEL} @ "
            f"{config.CAN_BITRATE // 1000}K baud."
        )

    except Exception as exc:
        print(f"[CAN] Bus not available: {exc}")
        bus = None
        return False

    _dispatcher_running = True

    _dispatcher_thread = threading.Thread(
        target=_dispatcher_loop,
        daemon=True,
        name="CANDispatcherThread",
    )

    _dispatcher_thread.start()

    print("[CAN] Dispatcher thread started.")

    return True


# ============================================================
# SHUTDOWN
# ============================================================

def shutdown() -> None:
    """Stop the dispatcher and close the CAN bus."""

    global bus
    global _dispatcher_running

    _dispatcher_running = False

    if _dispatcher_thread is not None:
        _dispatcher_thread.join(timeout=1.0)

    if bus is not None:
        try:
            bus.shutdown()
        except Exception as exc:
            print(f"[CAN] Shutdown error: {exc}")

        bus = None

    print("[CAN] Bus shut down.")


# ============================================================
# SEND
# ============================================================

def send(can_id: int, data_bytes: list[int]) -> bool:
    """
    Send an MKS CAN frame.

    The MKS protocol requires:

        CRC = (CAN_ID + sum(data_bytes)) & 0xFF

    The CRC byte is appended automatically.

    Returns:
        True if transmission succeeded.
        False otherwise.
    """

    if bus is None:
        print("[CAN SEND] Bus is not initialized.")
        return False

    crc = (can_id + sum(data_bytes)) & 0xFF

    frame_data = data_bytes + [crc]

    message = can.Message(
        arbitration_id=can_id,
        data=frame_data,
        is_extended_id=False,
    )

    with _send_lock:

        try:
            bus.send(message)
            return True

        except Exception as exc:
            print(f"[CAN SEND] Error: {exc}")
            return False


# ============================================================
# MOTOR RECEIVE
# ============================================================

def recv_motor(
    timeout: float = config.CAN_RECV_TIMEOUT_S,
) -> can.Message | None:
    """
    Return the next motor response frame.

    Returns None if the timeout expires.
    """

    try:
        return _motor_queue.get(timeout=timeout)

    except queue.Empty:
        return None


# ============================================================
# SENSOR RECEIVE
# ============================================================

def recv_sensor(
    timeout: float = config.CAN_RECV_TIMEOUT_S,
) -> can.Message | None:
    """
    Return the next tactile sensor frame.

    timeout=0 performs a non-blocking read.
    """

    try:

        if timeout > 0:
            return _sensor_queue.get(timeout=timeout)

        return _sensor_queue.get_nowait()

    except queue.Empty:
        return None
