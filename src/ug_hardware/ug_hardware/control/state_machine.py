"""
control/state_machine.py
-------------------------
Gripper state machine.

Runs in its own daemon thread at SM_LOOP_HZ.

Normal flow:
    IDLE → CLOSE → CONTACT → HOLD → OPEN → IDLE

Public interface:
    sm.send_command(cmd)   — enqueue "close", "open", or "cancel"
    sm.current_state_name  — read-only, thread-safe property
"""

from __future__ import annotations

import queue
import threading
import time

from .. import config
from .states.idle import IdleState
from .states.close import CloseState
from .states.contact import ContactState
from .states.hold import HoldState
from .states.open import OpenState
from .states.safety_state import SafetyState

_LOOP_PERIOD: float = 1.0 / config.SM_LOOP_HZ


class StateMachine:
    """
    Manages state transitions for the gripper.

    Usage from higher-level application:

        sm = StateMachine()
        sm.start()
        sm.send_command("close")
        print(sm.current_state_name)
        sm.stop()
    """

    _STATE_MAP: dict = {
        "IDLE": IdleState,
        "CLOSE": CloseState,
        "CONTACT": ContactState,
        "HOLD": HoldState,
        "OPEN": OpenState,
        "SAFETY": SafetyState,
    }

    def __init__(self) -> None:

        # maxsize=1: if a command is already pending,
        # drop the new one.
        self._cmd_queue: queue.Queue = queue.Queue(
            maxsize=1
        )

        self._state_lock: threading.Lock = threading.Lock()

        self._current_state = IdleState()
        self._current_name: str = IdleState.NAME

        self._thread: threading.Thread | None = None
        self._running: bool = False

    # =========================================================
    # PUBLIC PROPERTIES
    # =========================================================

    @property
    def current_state_name(self) -> str:
        """
        Return the current state name.

        Thread-safe.
        """

        with self._state_lock:
            return self._current_name

    def wait_for_state(
        self,
        target_states: set[str],
        timeout: float = 30.0,
    ) -> str | None:
        """
        Wait until the state machine reaches one of target_states.

        Returns:
            The reached state name, or None if timeout expires.
        """

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            current = self.current_state_name

            if current in target_states:
                return current

            time.sleep(0.01)

        return None

    def wait_until_not_state(
        self,
        state: str,
        timeout: float = 5.0,
    ) -> str | None:
        """
        Wait until the state machine leaves the given state.

        Returns:
            The new state name, or None if timeout expires.
        """

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            current = self.current_state_name

            if current != state:
                return current

            time.sleep(0.01)

        return None

    # =========================================================
    # COMMAND INTERFACE
    # =========================================================

    def send_command(self, cmd: str) -> None:
        """
        Enqueue a command.

        Valid commands:
            close
            open
            cancel

        If the queue already contains a command,
        the new command is silently dropped.
        """

        try:
            self._cmd_queue.put_nowait(cmd)

        except queue.Full:
            pass

    def get_command(self) -> str | None:
        """
        Consume and return the pending command.

        Called by state objects from within the
        state-machine thread.
        """

        try:
            return self._cmd_queue.get_nowait()

        except queue.Empty:
            return None

    # =========================================================
    # LIFECYCLE
    # =========================================================

    def start(self) -> None:
        """
        Start the state-machine thread.

        Call once.
        """

        if self._running:
            return

        self._running = True

        self._current_state.enter(self)

        self._thread = threading.Thread(
            target=self._loop,
            daemon=True,
            name="StateMachineThread",
        )

        self._thread.start()

        print("[SM] State machine started.")

    def stop(self) -> None:
        """
        Stop the state-machine thread cleanly.
        """

        self._running = False

        if self._thread is not None:

            self._thread.join(
                timeout=2.0
            )

            self._thread = None

        print("[SM] State machine stopped.")

    # =========================================================
    # INTERNAL LOOP
    # =========================================================

    def _loop(self) -> None:

        while self._running:

            t_start = time.monotonic()

            try:

                next_state_name = (
                    self._current_state.update(self)
                )

            except Exception as exc:

                print(
                    f"[SM] Error in "
                    f"{self._current_name}.update(): "
                    f"{exc}"
                )

                next_state_name = None

            if next_state_name is not None:

                self._transition(
                    next_state_name
                )

            elapsed = (
                time.monotonic()
                - t_start
            )

            sleep_time = (
                _LOOP_PERIOD
                - elapsed
            )

            if sleep_time > 0:

                time.sleep(
                    sleep_time
                )

    # =========================================================
    # STATE TRANSITION
    # =========================================================

    def _transition(
        self,
        next_name: str,
    ) -> None:

        if next_name not in self._STATE_MAP:

            print(
                f"[SM] Unknown state "
                f"'{next_name}' — staying in "
                f"{self._current_name}."
            )

            return

        print(
            f"[SM] "
            f"{self._current_name} "
            f"→ {next_name}"
        )

        try:

            self._current_state.exit(
                self
            )

        except Exception as exc:

            print(
                f"[SM] Error in "
                f"{self._current_name}.exit(): "
                f"{exc}"
            )

        new_state = (
            self._STATE_MAP[next_name]()
        )

        try:

            new_state.enter(self)

        except Exception as exc:

            print(
                f"[SM] Error in "
                f"{next_name}.enter(): "
                f"{exc}"
            )

        with self._state_lock:

            self._current_state = new_state
            self._current_name = next_name
