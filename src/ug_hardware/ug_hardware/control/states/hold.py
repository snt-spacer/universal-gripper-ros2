"""
control/states/hold.py
-----------------------
HOLD state: gripper maintains position with motor stopped.
Monitors for object loss or excessive force.

Transitions:
    HOLD → OPEN      on "open"/"cancel" command or object lost
    HOLD → CONTACT   if force exceeds HOLD_MAX_FZ_N (re-adjust)
"""

from __future__ import annotations

from ... import config
from ... import tactile_sensor_driver
from ... import motor_driver


class HoldState:
    NAME = "HOLD"

    def enter(self, context) -> None:
        motor_driver.stop_motor()

        fz = tactile_sensor_driver.get_avg_fz()

        print(
            f"[STATE] Entered {self.NAME} — "
            f"Fz_avg={fz:.3f} N"
        )

    def update(self, context) -> str | None:

        # ── External command ───────────────────────────────
        cmd = context.get_command()

        if cmd in ("cancel", "open"):

            print(
                f"[STATE] HOLD: command '{cmd}' → OPEN"
            )

            return "OPEN"

        fz_avg = tactile_sensor_driver.get_avg_fz()

        # ── Object lost ────────────────────────────────────
        if fz_avg < config.HOLD_MIN_FZ_N:

            print(
                f"[STATE] HOLD: object lost "
                f"(Fz={fz_avg:.3f} N) → OPEN"
            )

            return "OPEN"

        # ── Excessive force: back to CONTACT ───────────────
        if fz_avg > config.HOLD_MAX_FZ_N:

            print(
                f"[STATE] HOLD: force too high "
                f"(Fz={fz_avg:.3f} N) → CONTACT"
            )

            return "CONTACT"

        return None

    def exit(self, context) -> None:

        motor_driver.stop_motor()

        print(
            f"[STATE] Exiting {self.NAME}"
        )
