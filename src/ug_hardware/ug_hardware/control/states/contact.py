"""
control/states/contact.py
--------------------------
CONTACT state: gripper just touched the object. A simple PID controller
adjusts motor speed to reach HOLD_TARGET_FZ_N, then transitions to HOLD
once the force is stable.

PID input:  average Fz across all 4 sensors (N)
PID output: speed_param for the F6 speed command (closing or opening)

Transitions:
    CONTACT → HOLD    when Fz is stable within tolerance for HOLD_STABLE_TIME_S
    CONTACT → OPEN    when Fz drops below CONTACT_MIN_FZ_N (object lost)
    CONTACT → IDLE    on cancel/open command
"""

from __future__ import annotations

import time

from ... import config
from ... import tactile_sensor_driver
from ... import motor_driver


class _SimplePID:
    """
    Minimal discrete PID controller.

    Output is a signed float:
        positive = close more
        negative = open more
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        max_out: float,
    ) -> None:

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out

        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._prev_time: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def update(self, error: float) -> float:

        now = time.monotonic()

        if self._prev_time is None:
            dt = 1.0 / config.SM_LOOP_HZ
        else:
            dt = now - self._prev_time
            dt = max(0.001, dt)

        self._prev_time = now

        self._integral += error * dt

        # Anti-windup: clamp integral contribution
        max_i = self.max_out / max(self.ki, 1e-9)

        self._integral = max(
            -max_i,
            min(max_i, self._integral),
        )

        derivative = (
            error - self._prev_error
        ) / dt

        self._prev_error = error

        output = (
            self.kp * error
            + self.ki * self._integral
            + self.kd * derivative
        )

        return max(
            -self.max_out,
            min(self.max_out, output),
        )


class ContactState:
    NAME = "CONTACT"

    def __init__(self) -> None:

        self._pid = _SimplePID(
            kp=config.PID_KP,
            ki=config.PID_KI,
            kd=config.PID_KD,
            max_out=config.PID_MAX_OUTPUT,
        )

        self._stable_since: float | None = None

    def enter(self, context) -> None:

        self._pid.reset()
        self._stable_since = None

        fz = tactile_sensor_driver.get_avg_fz()

        print(
            f"[STATE] Entered {self.NAME} — "
            f"Fz_avg={fz:.3f} N, "
            f"target={config.HOLD_TARGET_FZ_N} N"
        )

    def update(self, context) -> str | None:

        # ── External cancel ────────────────────────────────
        cmd = context.get_command()

        if cmd in ("cancel", "open"):

            motor_driver.stop_motor()

            return "IDLE"

        fz_avg = tactile_sensor_driver.get_avg_fz()

        pos = motor_driver.read_ui_output_position()

        # ── Safety: excessive force ────────────────────────
        if fz_avg >= config.SAFETY_FZ_LIMIT_N:

            motor_driver.stop_motor()

            print(
                f"[STATE] CONTACT → SAFETY "
                f"(safety: Fz={fz_avg:.2f} N >= "
                f"{config.SAFETY_FZ_LIMIT_N} N)"
            )

            return "IDLE"

        # ── Object lost ────────────────────────────────────
        if fz_avg < config.CONTACT_MIN_FZ_N:

            motor_driver.stop_motor()

            self._stable_since = None

            print(
                f"[STATE] CONTACT: object lost "
                f"(Fz={fz_avg:.3f} N) → OPEN"
            )

            return "OPEN"

        # ── PID force control ──────────────────────────────
        error = (
            config.HOLD_TARGET_FZ_N
            - fz_avg
        )

        pid_out = self._pid.update(error)

        in_band = (
            abs(error)
            <= config.HOLD_TOLERANCE_N
        )

        if not in_band:

            self._stable_since = None

            if pid_out > 0:

                # Need more force → close more
                if pos < (
                    motor_driver.ANGLE_MAX_DEG
                    - config.JOG_STOP_MARGIN_DEG
                ):

                    sp = max(
                        config.PID_MIN_OUTPUT,
                        int(abs(pid_out)),
                    )

                    motor_driver.start_closing_with_speed_param(
                        sp
                    )

                else:

                    motor_driver.stop_motor()

            else:

                # Too much force → open slightly
                if pos > (
                    motor_driver.ANGLE_MIN_DEG
                    + config.JOG_STOP_MARGIN_DEG
                ):

                    sp = max(
                        config.PID_MIN_OUTPUT,
                        int(abs(pid_out)),
                    )

                    motor_driver.start_opening_with_speed_param(
                        sp
                    )

                else:

                    motor_driver.stop_motor()

        else:

            # ── Force in band — wait for stability ─────────
            motor_driver.stop_motor()

            now = time.monotonic()

            if self._stable_since is None:
                self._stable_since = now

            elapsed = (
                now - self._stable_since
            )

            if elapsed >= config.HOLD_STABLE_TIME_S:

                print(
                    f"[STATE] CONTACT → HOLD "
                    f"(Fz={fz_avg:.3f} N stable for "
                    f"{elapsed:.2f} s)"
                )

                return "HOLD"

        return None

    def exit(self, context) -> None:

        motor_driver.stop_motor()

        self._pid.reset()
        self._stable_since = None

        print(
            f"[STATE] Exiting {self.NAME}"
        )
