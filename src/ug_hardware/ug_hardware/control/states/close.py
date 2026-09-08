"""
control/states/close.py
------------------------
CLOSE state: gripper closes slowly until contact is detected or
the physical limit is reached.

Transitions:
    CLOSE → CONTACT   when average Fz >= CONTACT_FZ_THRESHOLD_N
    CLOSE → IDLE      on cancel/open command or physical limit reached
"""

from __future__ import annotations

from ... import config
from ... import motor_driver
from ... import tactile_sensor_driver


class CloseState:
    NAME = "CLOSE"

    def __init__(self) -> None:
        self._cancelled: bool = False

    def enter(self, context) -> None:
        self._cancelled = False

        current = motor_driver.read_ui_output_position()

        if current >= (
            motor_driver.ANGLE_MAX_DEG
            - config.JOG_STOP_MARGIN_DEG
        ):
            print(
                f"[STATE] CLOSE: already at closed limit "
                f"({current:.2f}°) — going to IDLE."
            )
            self._cancelled = True
            return

        motor_driver.start_closing_slow()

        print(
            f"[STATE] Entered {self.NAME} — "
            f"closing from {current:.2f}°"
        )

    def update(self, context) -> str | None:

        # ── Cancelled at entry ─────────────────────────────
        if self._cancelled:
            return "IDLE"

        # ── External cancel ────────────────────────────────
        cmd = context.get_command()

        if cmd in ("cancel", "open"):
            motor_driver.stop_motor()

            print(
                f"[STATE] CLOSE: cancelled by command '{cmd}'."
            )

            return "IDLE"

        # ── Physical limit watchdog ────────────────────────
        pos = motor_driver.read_ui_output_position()

        if pos >= (
            motor_driver.ANGLE_MAX_DEG
            - config.JOG_STOP_MARGIN_DEG
        ):
            motor_driver.stop_motor()

            print(
                "[STATE] CLOSE: closed limit reached "
                "without contact → IDLE."
            )

            return "IDLE"

        # ── Contact detection ──────────────────────────────
        fz_avg = tactile_sensor_driver.get_avg_fz()

        if fz_avg >= config.CONTACT_FZ_THRESHOLD_N:

            motor_driver.stop_motor()

            print(
                f"[STATE] CLOSE: contact detected "
                f"(Fz_avg={fz_avg:.3f} N) "
                f"at pos={pos:.2f}° → CONTACT"
            )

            return "CONTACT"

        return None

    def exit(self, context) -> None:
        motor_driver.stop_motor()

        print(f"[STATE] Exiting {self.NAME}")
