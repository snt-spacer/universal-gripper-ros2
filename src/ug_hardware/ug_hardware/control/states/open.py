"""
control/states/open.py
-----------------------
OPEN state: gripper opens to the maximum aperture at constant speed.

Transitions:
    OPEN → IDLE   when open limit reached or "cancel" command received
"""

from __future__ import annotations

from ... import config
from ... import motor_driver


class OpenState:
    NAME = "OPEN"

    def enter(self, context) -> None:
        current = motor_driver.read_ui_output_position()

        if current <= (
            motor_driver.ANGLE_MIN_DEG
            + config.JOG_STOP_MARGIN_DEG
        ):
            print(
                f"[STATE] OPEN: already at open limit "
                f"({current:.2f}°)."
            )
            # update() will detect this and return "IDLE" immediately
            return

        # Use speed mode (F6) for continuous opening
        speed_param = motor_driver._rpm_to_speed_param(
            config.SPEED_OPEN_RPM
        )

        motor_driver._send_speed(
            speed_param,
            config.MOTOR_DIR_OPEN,
            config.ACCEL_NORMAL,
        )

        print(
            f"[STATE] Entered {self.NAME} — "
            f"opening from {current:.2f}°"
        )

    def update(self, context) -> str | None:

        # ── Cancel command ─────────────────────────────────
        cmd = context.get_command()

        if cmd == "cancel":

            motor_driver.stop_motor()

            print(
                "[STATE] OPEN: cancelled."
            )

            return "IDLE"

        pos = motor_driver.read_ui_output_position()

        # ── Open limit reached ─────────────────────────────
        if pos <= (
            motor_driver.ANGLE_MIN_DEG
            + config.JOG_STOP_MARGIN_DEG
        ):

            motor_driver.stop_motor()

            print(
                f"[STATE] OPEN: open limit reached "
                f"({pos:.2f}°) → IDLE"
            )

            return "IDLE"

        return None

    def exit(self, context) -> None:

        motor_driver.stop_motor()

        print(
            f"[STATE] Exiting {self.NAME}"
        )
