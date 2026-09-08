"""
control/states/idle.py
-----------------------
IDLE state: gripper is stationary, motor enabled but not moving.

Transitions:
    IDLE → CLOSE   on "close" command
    IDLE → OPEN    on "open" command
"""

from __future__ import annotations

from ... import motor_driver


class IdleState:
    NAME = "IDLE"

    def enter(self, context) -> None:
        motor_driver.stop_motor()
        print(f"[STATE] Entered {self.NAME}")

    def update(self, context) -> str | None:
        cmd = context.get_command()

        if cmd == "close":
            return "CLOSE"

        if cmd == "open":
            return "OPEN"

        return None

    def exit(self, context) -> None:
        print(f"[STATE] Exiting {self.NAME}")
