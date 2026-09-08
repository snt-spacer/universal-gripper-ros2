from __future__ import annotations

import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer

from ug_interfaces.action import GripCommand
from ug_interfaces.msg import GripperStatus, TactileForces, TactileReading
from ug_interfaces.srv import CalibrateSensors, GoToPosition, JogMotor, StopMotor
from std_msgs.msg import Bool

from . import can_driver
from . import config
from . import motor_driver
from . import tactile_sensor_driver
from .control.state_machine import StateMachine


class HardwareInterface(Node):
    """
    ROS 2 hardware interface for the Universal Gripper.

    This node connects ROS 2 services/topics with the low-level
    CAN, motor and tactile-sensor drivers.
    """

    def __init__(self) -> None:
        super().__init__("hardware_interface")

        self._shutdown_lock = threading.Lock()
        self._shutdown_done = False
        self.goto_callback_group = (
            MutuallyExclusiveCallbackGroup()
        )

        # ---------------------------------------------------------
        # CAN
        # ---------------------------------------------------------

        if not can_driver.init():
            self.get_logger().error("CAN bus initialization failed.")
        else:
            self.get_logger().info("CAN bus initialized.")

        # ---------------------------------------------------------
        # Motor
        # ---------------------------------------------------------

        try:
            motor_driver.enable_motor()
            self.get_logger().info("Motor enabled.")
        except Exception as exc:
            self.get_logger().error(
                f"Failed to enable motor: {exc}"
            )

        # ---------------------------------------------------------
        # TACTILE SENSOR CALIBRATION
        # ---------------------------------------------------------

        self.get_logger().info(
            "Starting automatic tactile sensor calibration..."
        )

        try:
            calibration_success = tactile_sensor_driver.calibrate(
                num_samples=config.SENSOR_CALIB_SAMPLES,
                timeout_s=30.0,
            )

            if calibration_success:
                self.get_logger().info(
                    "Tactile sensors calibrated successfully."
                )
            else:
                self.get_logger().error(
                    "Tactile sensor calibration failed. "
                    "State machine will not start."
                )

                return

        except Exception as exc:
            self.get_logger().error(
                f"Tactile sensor calibration error: {exc}. "
                "State machine will not start."
            )

            return

        # ---------------------------------------------------------
        # STATE MACHINE
        # ---------------------------------------------------------
        self.get_logger().info(
            "Opening gripper to maximum aperture..."
        )

        try:
            opened = motor_driver.go_to_position_deg(
                config.ANGLE_OPEN_DEG
            )

            if not opened:
                self.get_logger().error(
                    "Failed to open gripper at startup. "
                    "UI will remain closed."
                )
                return

        except Exception as exc:
            self.get_logger().error(
                f"Startup opening error: {exc}. "
                "UI will remain closed."
            )
            return
        self.state_machine = StateMachine()
        self.state_machine.start()

        self.get_logger().info(
            "Gripper state machine started."
        )
        self.hardware_ready = True
        self.ready_pub = self.create_publisher(
            Bool,
            "/gripper/hardware_ready",
            10,
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

        self.status_pub = self.create_publisher(
            GripperStatus,
            "/gripper/status",
            10,
        )

        self.tactile_pub = self.create_publisher(
            TactileForces,
            "/gripper/tactile_forces",
            10,
        )

        # ---------------------------------------------------------
        # Action Server
        # ---------------------------------------------------------

        self.grip_action_server = ActionServer(
            self,
            GripCommand,
            "/gripper/grip",
            self._execute_grip,
        )
        # ---------------------------------------------------------
        # Services
        # ---------------------------------------------------------

        self.calibrate_srv = self.create_service(
            CalibrateSensors,
            "/gripper/calibrate_sensors",
            self._handle_calibrate,
        )

        self.jog_srv = self.create_service(
            JogMotor,
            "/gripper/jog",
            self._handle_jog,
        )

        self.stop_srv = self.create_service(
            StopMotor,
            "/gripper/stop",
            self._handle_stop,
        )

        self.goto_srv = self.create_service(
            GoToPosition,
            "/gripper/go_to",
            self._handle_go_to,
            callback_group=self.goto_callback_group,
        )

        # ---------------------------------------------------------
        # Timers
        # ---------------------------------------------------------

        self.ready_timer = self.create_timer(
            0.1,
            self._publish_ready,
        )

        # Sensor processing / tactile publication
        sensor_period = 1.0 / config.SM_LOOP_HZ

        self.sensor_timer = self.create_timer(
            sensor_period,
            self._update_hardware,
        )

        # Status publication
        self.status_timer = self.create_timer(
            0.1,
            self._publish_status,
        )

        self.get_logger().info(
            "Universal Gripper hardware interface started."
        )
    def _publish_ready(self) -> None:
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)

    # ============================================================
    # HARDWARE UPDATE
    # ============================================================

    def _update_hardware(self) -> None:
        """
        Process pending tactile CAN frames and publish tactile data.
        """

        try:
            motor_driver.check_jog_limits()
            tactile_sensor_driver.update_sensors()
            self._publish_tactile()
        except Exception as exc:
            self.get_logger().error(
                f"Hardware update error: {exc}"
            )

    # ============================================================
    # TACTILE PUBLISHING
    # ============================================================

    def _publish_tactile(self) -> None:
        """
        Publish current tactile forces.
        """

        fx, fy, fz = tactile_sensor_driver.get_forces()

        msg = TactileForces()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, texel_id in enumerate(config.TEXEL_IDS):

            reading = TactileReading()

            reading.texel_id = int(texel_id)
            reading.fx = float(fx[i])
            reading.fy = float(fy[i])
            reading.fz = float(fz[i])

            msg.texels.append(reading)

        msg.avg_fz = float(
            tactile_sensor_driver.get_avg_fz()
        )

        msg.max_fz = float(
            tactile_sensor_driver.get_max_fz()
        )

        self.tactile_pub.publish(msg)

    # ============================================================
    # STATUS PUBLISHING
    # ============================================================

    def _publish_status(self) -> None:
        """
        Publish current gripper status.
        """

        msg = GripperStatus()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.state = self.state_machine.current_state_name

        try:
            msg.angle_deg = float(
                motor_driver.read_ui_output_position()
            )
        except Exception:
            msg.angle_deg = 0.0

        try:
            msg.opening_mm = float(
                motor_driver.get_current_opening_mm()
            )
        except Exception:
            msg.opening_mm = 0.0

        try:
            msg.jog_active = bool(
                motor_driver.is_jog_active()
            )
        except Exception:
            msg.jog_active = False

        self.status_pub.publish(msg)

    # ============================================================
    # CALIBRATION SERVICE
    # ============================================================

    def _handle_calibrate(
        self,
        request: CalibrateSensors.Request,
        response: CalibrateSensors.Response,
    ) -> CalibrateSensors.Response:

        num_samples = (
            request.num_samples
            if request.num_samples > 0
            else config.SENSOR_CALIB_SAMPLES
        )

        timeout_s = (
            request.timeout_s
            if request.timeout_s > 0.0
            else 30.0
        )

        self.get_logger().info(
            f"Starting tactile calibration: "
            f"{num_samples} samples, "
            f"{timeout_s:.1f}s timeout."
        )

        try:
            success = tactile_sensor_driver.calibrate(
                num_samples=num_samples,
                timeout_s=timeout_s,
            )

            response.success = bool(success)

            if success:
                response.message = (
                    "Tactile sensor calibration completed successfully."
                )
            else:
                response.message = (
                    "Tactile sensor calibration failed."
                )

        except Exception as exc:

            response.success = False
            response.message = (
                f"Calibration error: {exc}"
            )

            self.get_logger().error(
                response.message
            )

        return response

    # ============================================================
    # JOG SERVICE
    # ============================================================

    def _handle_jog(
        self,
        request: JogMotor.Request,
        response: JogMotor.Response,
    ) -> JogMotor.Response:

        direction = request.direction.strip().upper()

        try:

            if direction == "OPEN":
                motor_driver.open_motor()

            elif direction == "CLOSE":
                motor_driver.close_motor()

            else:
                response.success = False
                response.message = (
                    "Invalid direction. Use OPEN or CLOSE."
                )
                return response

            response.success = True
            response.message = (
                f"Jog {direction} started."
            )

            self.get_logger().info(
                response.message
            )

        except Exception as exc:

            response.success = False
            response.message = (
                f"Jog error: {exc}"
            )

            self.get_logger().error(
                response.message
            )

        return response

    # ============================================================
    # STOP SERVICE
    # ============================================================

    def _handle_stop(
        self,
        request: StopMotor.Request,
        response: StopMotor.Response,
    ) -> StopMotor.Response:

        del request

        try:

            motor_driver.stop_motor()

            response.success = True

            self.get_logger().info(
                "Motor stopped."
            )

        except Exception as exc:

            response.success = False

            self.get_logger().error(
                f"Stop error: {exc}"
            )

        return response

    # ============================================================
    # GO TO POSITION SERVICE
    # ============================================================

    def _handle_go_to(
        self,
        request: GoToPosition.Request,
        response: GoToPosition.Response,
    ) -> GoToPosition.Response:

        mode = request.mode.strip().lower()
        target = float(request.target)

        if mode not in ("angle", "opening"):
            response.success = False
            response.message = (
                "Invalid mode. Use 'angle' or 'opening'."
            )
            response.final_angle_deg = float(
                motor_driver.read_ui_output_position()
            )
            response.final_opening_mm = float(
                motor_driver.get_gripper_opening_mm(
                    response.final_angle_deg
                )
            )

            self.get_logger().error(
                response.message
            )

            return response

        self.get_logger().info(
            f"Go-to command received: "
            f"mode={mode}, target={target:.2f}"
        )

        try:

            if mode == "angle":

                success = motor_driver.go_to_position_deg(
                    target,
                    speed_rpm=config.SPEED_GOTO_RPM,
                    accel=config.ACCEL_NORMAL,
                    timeout=config.GOTO_TIMEOUT_S,
                )

            else:

                success = motor_driver.go_to_opening_mm(
                    target,
                    speed_rpm=config.SPEED_GOTO_RPM,
                    accel=config.ACCEL_NORMAL,
                    timeout=config.GOTO_TIMEOUT_S,
                )

            final_angle = float(
                motor_driver.read_ui_output_position()
            )

            final_opening = float(
                motor_driver.get_gripper_opening_mm(
                    final_angle
                )
            )

            response.success = bool(success)
            response.final_angle_deg = final_angle
            response.final_opening_mm = final_opening

            if success:
                response.message = (
                    f"Reached target. "
                    f"Angle={final_angle:.2f} deg, "
                    f"Opening={final_opening / 10:.2f} cm."
                )
            else:
                response.message = (
                    f"Go-to failed or timed out. "
                    f"Angle={final_angle:.2f} deg, "
                    f"Opening={final_opening / 10:.2f} cm."
                )

            self.get_logger().info(
                response.message
            )

        except Exception as exc:

            response.success = False

            try:
                final_angle = float(
                    motor_driver.read_ui_output_position()
                )
            except Exception:
                final_angle = 0.0

            try:
                final_opening = float(
                    motor_driver.get_gripper_opening_mm(
                        final_angle
                    )
                )
            except Exception:
                final_opening = 0.0

            response.final_angle_deg = final_angle
            response.final_opening_mm = final_opening
            response.message = (
                f"Go-to error: {exc}"
            )

            self.get_logger().error(
                response.message
            )

        return response

    # ============================================================
    # SHUTDOWN
    # ============================================================

    # ============================================================
    # GRIP COMMAND ACTION
    # ============================================================

    def _execute_grip(self, goal_handle):
        """
        Execute a gripper command through the state machine.
        """

        command = goal_handle.request.command.strip().lower()

        # ---------------------------------------------------------
        # Validate command
        # ---------------------------------------------------------

        if command not in ("close", "open", "cancel"):

            goal_handle.abort()

            result = GripCommand.Result()

            result.success = False
            result.final_state = (
                self.state_machine.current_state_name
            )
            result.final_angle_deg = 0.0
            result.final_fz = 0.0

            self.get_logger().error(
                f"Invalid grip command: '{command}'"
            )

            return result

        self.get_logger().info(
            f"Received grip command: '{command}'"
        )

        # ---------------------------------------------------------
        # Send command to state machine
        # ---------------------------------------------------------

        self.state_machine.send_command(command)

        # ---------------------------------------------------------
        # Define terminal states
        # ---------------------------------------------------------

        if command == "close":
            target_states = {
                "HOLD",
                "SAFETY",
            }

        else:
            target_states = {
                "IDLE",
                "SAFETY",
            }

        # ---------------------------------------------------------
        # Wait for state machine + publish feedback
        # ---------------------------------------------------------

        deadline = time.monotonic() + 30.0
        final_state = None

        while time.monotonic() < deadline:

            current_state = (
                self.state_machine.current_state_name
            )

            try:
                angle_deg = float(
                    motor_driver.read_ui_output_position()
                )
            except Exception:
                angle_deg = 0.0

            try:
                fz = float(
                    tactile_sensor_driver.get_avg_fz()
                )
            except Exception:
                fz = 0.0

            # -----------------------------------------------------
            # Publish Action Feedback
            # -----------------------------------------------------

            feedback = GripCommand.Feedback()

            feedback.current_state = current_state
            feedback.angle_deg = angle_deg
            feedback.fz = fz

            goal_handle.publish_feedback(feedback)

            # -----------------------------------------------------
            # Check terminal state
            # -----------------------------------------------------

            if current_state in target_states:

                final_state = current_state
                break

            # -----------------------------------------------------
            # Keep ROS responsive
            # -----------------------------------------------------

            time.sleep(0.05)

        # ---------------------------------------------------------
        # Timeout
        # ---------------------------------------------------------

        if final_state is None:

            goal_handle.abort()

            result = GripCommand.Result()

            result.success = False
            result.final_state = (
                self.state_machine.current_state_name
            )
            result.final_angle_deg = 0.0
            result.final_fz = 0.0

            self.get_logger().error(
                f"Grip command '{command}' timed out."
            )

            return result

        # ---------------------------------------------------------
        # Action succeeded
        # ---------------------------------------------------------

        goal_handle.succeed()

        result = GripCommand.Result()

        result.success = True
        result.final_state = final_state

        try:
            result.final_angle_deg = float(
                motor_driver.read_ui_output_position()
            )
        except Exception:
            result.final_angle_deg = 0.0

        try:
            result.final_fz = float(
                tactile_sensor_driver.get_avg_fz()
            )
        except Exception:
            result.final_fz = 0.0

        self.get_logger().info(
            f"Grip command '{command}' completed "
            f"in state {final_state}."
        )

        return result

    def shutdown_hardware(self) -> None:
        """
        Safely stop the motor and close the CAN bus.
        """

        with self._shutdown_lock:

            if self._shutdown_done:
                return

            self._shutdown_done = True

        self.get_logger().info(
            "Shutting down hardware..."
        )

        try:
            self.state_machine.stop()
        except Exception as exc:
            self.get_logger().error(
                f"State machine shutdown error: {exc}"
            )


        try:
            motor_driver.stop_motor()
        except Exception as exc:
            self.get_logger().error(
                f"Motor shutdown error: {exc}"
            )

        try:
            can_driver.shutdown()
        except Exception as exc:
            self.get_logger().error(
                f"CAN shutdown error: {exc}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(
        num_threads=2
    )

    node = HardwareInterface()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.shutdown_hardware()
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
