"""
control/states/safety_state.py
-------------------------------
SAFETY state: entered when an unsafe condition is detected.
Motor is stopped. Requires explicit operator release to continue.

For now this is a simple latch — the gripper stays stopped until
the operator sends an "open" command to safely recover.

Transitions:
    SAFETY → OPEN   on "open" command
    SAFETY → IDLE   on "cancel" command
"""

from __future__ import annotations

from ... import motor_driver


class SafetyState:
    NAME = "SAFETY"

    def enter(self, context) -> None:
        motor_driver.stop_motor()

        print(
            f"[STATE] *** Entered {self.NAME} *** — "
            "motor stopped. "
            "Send 'open' to recover or 'cancel' to return to IDLE."
        )

    def update(self, context) -> str | None:

        cmd = context.get_command()

        if cmd == "open":
            print(
                "[STATE] SAFETY: operator released — opening."
            )
            return "OPEN"

        if cmd == "cancel":
            print(
                "[STATE] SAFETY: operator cancelled — "
                "going to IDLE."
            )
            return "IDLE"

        return None

    def exit(self, context) -> None:
        print(
            f"[STATE] Exiting {self.NAME}"
        )
