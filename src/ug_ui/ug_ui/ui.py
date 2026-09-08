from __future__ import annotations

import math
import os
import sys

import pyqtgraph as pg
import rclpy

from ament_index_python.packages import get_package_share_directory
from PyQt5 import QtCore, QtWidgets, QtGui

from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

from ug_interfaces.msg import GripperStatus, TactileForces
from ug_interfaces.srv import JogMotor, StopMotor, GoToPosition
from ug_interfaces.action import GripCommand


pg.setConfigOptions(antialias=True)


# ============================================================================
# PALETTE
# ============================================================================

_STATE_COLORS = {
    "IDLE": "#aaaaaa",
    "CLOSE": "#FFD700",
    "CONTACT": "#FF8C00",
    "HOLD": "#00FF99",
    "OPEN": "#33aaff",
    "SAFETY": "#FF0000",
}


_FZ_GRADIENT = [
    (0.0, 0, 80, 255),
    (2.0, 0, 200, 80),
    (4.0, 220, 220, 0),
    (6.0, 255, 140, 0),
    (10.0, 255, 0, 0),
]


_SENSOR_POS = {
    0: (0.355, 0.63),
    1: (0.69, 0.70),
    2: (0.69, 0.30),
    3: (0.355, 0.231),
}

_RIGHT_SENSORS = {1, 2}

CIRCLE_R = 42
ARROW_SCALE = 55
FZ_TARGET_N = 1.5
FXY_COLOR_MAX = 2.0


# ============================================================================
# HELPERS
# ============================================================================

def _fz_to_rgb(fz: float):
    fz = max(0.0, fz)
    s = _FZ_GRADIENT

    if fz <= s[0][0]:
        return s[0][1], s[0][2], s[0][3]

    if fz >= s[-1][0]:
        return s[-1][1], s[-1][2], s[-1][3]

    for i in range(len(s) - 1):
        v0, r0, g0, b0 = s[i]
        v1, r1, g1, b1 = s[i + 1]

        if v0 <= fz <= v1:
            t = (fz - v0) / (v1 - v0)

            return (
                int(r0 + t * (r1 - r0)),
                int(g0 + t * (g1 - g0)),
                int(b0 + t * (b1 - b0)),
            )

    return 255, 0, 0


def _fxy_to_rgb(fx: float, fy: float):
    fxy_mag = max(abs(fx), abs(fy))

    clamped = max(
        0.0,
        min(fxy_mag, FXY_COLOR_MAX),
    )

    t = clamped / FXY_COLOR_MAX

    r = int(255 * t)
    g = int(255 * (1.0 - t))

    return r, g, 0


def _draw_arrow(p, x1, y1, x2, y2, color, lw=4):

    if x1 == x2 and y1 == y2:
        return

    p.setPen(
        QtGui.QPen(
            color,
            lw,
            QtCore.Qt.SolidLine,
            QtCore.Qt.RoundCap,
            QtCore.Qt.RoundJoin,
        )
    )

    p.drawLine(x1, y1, x2, y2)

    ang = math.atan2(
        y2 - y1,
        x2 - x1,
    )

    tl = 16
    ta = math.radians(28)

    pts = [
        QtCore.QPoint(x2, y2),
        QtCore.QPoint(
            int(x2 - tl * math.cos(ang - ta)),
            int(y2 - tl * math.sin(ang - ta)),
        ),
        QtCore.QPoint(
            int(x2 - tl * math.cos(ang + ta)),
            int(y2 - tl * math.sin(ang + ta)),
        ),
    ]

    p.setBrush(QtGui.QBrush(color))
    p.setPen(QtGui.QPen(color, 1))

    p.drawPolygon(QtGui.QPolygon(pts))


# ============================================================================
# GRIPPER OVERLAY
# ============================================================================

class GripperOverlay(QtWidgets.QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

        try:
            package_share = get_package_share_directory("ug_ui")

            image_path = os.path.join(
                package_share,
                "images",
                "Gripper.png",
            )

        except Exception:

            image_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "Gripper.png",
            )

        self._px = QtGui.QPixmap(image_path)

        self.setMinimumSize(500, 360)

        self.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )

        self.texels = [
            {
                "fx": 0.0,
                "fy": 0.0,
                "fz": 0.0,
            }
            for _ in range(4)
        ]

    def set_forces(self, texels):

        for i in range(min(4, len(texels))):

            self.texels[i]["fx"] = float(texels[i].fx)
            self.texels[i]["fy"] = float(texels[i].fy)
            self.texels[i]["fz"] = float(texels[i].fz)

        self.update()

    def paintEvent(self, _):

        p = QtGui.QPainter(self)

        p.setRenderHint(
            QtGui.QPainter.Antialiasing
        )

        W = self.width()
        H = self.height()

        if not self._px.isNull():

            sc = self._px.scaled(
                W,
                H,
                QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation,
            )

            ox = (W - sc.width()) // 2
            oy = (H - sc.height()) // 2

            p.drawPixmap(
                ox,
                oy,
                sc,
            )

            iw = sc.width()
            ih = sc.height()

        else:

            ox = 0
            oy = 0
            iw = W
            ih = H

        for i in range(4):

            nx, ny = _SENSOR_POS[i]

            cx = int(ox + nx * iw)
            cy = int(oy + ny * ih)

            fx = self.texels[i]["fx"]
            fy = self.texels[i]["fy"]
            fz = self.texels[i]["fz"]

            r, g, b = _fxy_to_rgb(
                fx,
                fy,
            )

            col = QtGui.QColor(
                r,
                g,
                b,
            )

            min_scale = 0.16
            max_scale = 1.20

            radius = int(
                CIRCLE_R
                * (
                    min_scale
                    + (
                        max_scale
                        - min_scale
                    )
                    * min(abs(fz), 4.0)
                    / 4.0
                )
            )

            radius = max(
                4,
                min(
                    radius,
                    int(CIRCLE_R * max_scale),
                ),
            )

            # Glow

            p.setPen(
                QtGui.QPen(
                    QtGui.QColor(
                        r,
                        g,
                        b,
                        55,
                    ),
                    8,
                )
            )

            p.setBrush(QtCore.Qt.NoBrush)

            p.drawEllipse(
                cx - radius - 8,
                cy - radius - 8,
                (radius + 8) * 2,
                (radius + 8) * 2,
            )

            # Sensor circle

            p.setPen(
                QtGui.QPen(
                    col,
                    3,
                )
            )

            p.setBrush(
                QtGui.QBrush(
                    QtGui.QColor(
                        r,
                        g,
                        b,
                        180,
                    )
                )
            )

            p.drawEllipse(
                cx - radius,
                cy - radius,
                radius * 2,
                radius * 2,
            )

            # Fx arrow

            fx_end_y = cy + int(
                fx * ARROW_SCALE
            )

            _draw_arrow(
                p,
                cx,
                cy,
                cx,
                fx_end_y,
                QtGui.QColor(
                    255,
                    220,
                    50,
                ),
            )

            # Fy arrow

            mag = int(
                abs(fy) * ARROW_SCALE
            )

            center_x = ox + iw // 2
            center_y = oy + ih // 2

            base_ang = math.atan2(
                center_y - cy,
                center_x - cx,
            )

            if i in _RIGHT_SENSORS:

                ang = (
                    base_ang + math.pi
                    if fy >= 0
                    else base_ang
                )

            else:

                ang = (
                    base_ang
                    if fy >= 0
                    else base_ang + math.pi
                )

            fy_end_x = cx + int(
                mag * math.cos(ang)
            )

            fy_end_y = cy + int(
                mag * math.sin(ang)
            )

            _draw_arrow(
                p,
                cx,
                cy,
                fy_end_x,
                fy_end_y,
                QtGui.QColor(
                    80,
                    210,
                    255,
                ),
            )

            # Labels

            p.setFont(
                QtGui.QFont(
                    "Consolas",
                    9,
                    QtGui.QFont.Bold,
                )
            )

            p.setPen(
                QtGui.QPen(
                    QtGui.QColor(
                        255,
                        220,
                        50,
                    )
                )
            )

            fx_label_x = cx + 6

            fx_label_y = (
                fx_end_y - 6
                if fx >= 0
                else fx_end_y + 16
            )

            p.drawText(
                fx_label_x,
                fx_label_y,
                "Fx",
            )

            p.setPen(
                QtGui.QPen(
                    QtGui.QColor(
                        80,
                        210,
                        255,
                    )
                )
            )

            lbl_off_x = (
                8
                if math.cos(ang) >= 0
                else -24
            )

            fy_label_x = (
                fy_end_x + lbl_off_x
            )

            fy_label_y = (
                fy_end_y - 6
            )

            p.drawText(
                fy_label_x,
                fy_label_y,
                "Fy",
            )

            # Numeric values

            p.setFont(
                QtGui.QFont(
                    "Consolas",
                    9,
                )
            )

            p.setPen(
                QtGui.QPen(
                    QtGui.QColor(
                        255,
                        255,
                        255,
                    )
                )
            )

            fx_text = f"Fx={fx:+.2f}"
            fy_text = f"Fy={fy:+.2f}"
            fz_text = f"Fz={fz:+.2f}"

            text_offset_x = 58
            text_offset_y = -20

            text_x = (
                cx + text_offset_x
                if i in _RIGHT_SENSORS
                else cx - text_offset_x - 90
            )

            text_y = cy + text_offset_y

            p.drawText(
                text_x,
                text_y,
                fx_text,
            )

            p.drawText(
                text_x,
                text_y + 14,
                fy_text,
            )

            p.drawText(
                text_x,
                text_y + 28,
                fz_text,
            )

        # Legend

        legend_w = 260
        legend_h = 70

        legend_x = (
            ox
            + (iw - legend_w) // 2
        )

        legend_y = (
            oy
            + ih
            - legend_h
            - 14
        )

        p.setPen(QtCore.Qt.NoPen)

        p.setBrush(
            QtGui.QBrush(
                QtGui.QColor(
                    0,
                    0,
                    0,
                    180,
                )
            )
        )

        p.drawRoundedRect(
            legend_x,
            legend_y,
            legend_w,
            legend_h,
            10,
            10,
        )

        p.setFont(
            QtGui.QFont(
                "Consolas",
                10,
                QtGui.QFont.Bold,
            )
        )

        p.setPen(
            QtGui.QPen(
                QtGui.QColor(
                    255,
                    255,
                    255,
                )
            )
        )

        p.drawText(
            QtCore.QRect(
                legend_x,
                legend_y + 12,
                legend_w,
                16,
            ),
            QtCore.Qt.AlignCenter,
            "Circle size = Fz",
        )

        p.drawText(
            QtCore.QRect(
                legend_x,
                legend_y + 30,
                legend_w,
                16,
            ),
            QtCore.Qt.AlignCenter,
            "Circle color = Avg Fx/Fy",
        )

        # Gradient

        grad_x = legend_x + 12
        grad_y = legend_y + 44
        grad_w = legend_w - 24
        grad_h = 14

        gradient = QtGui.QLinearGradient(
            grad_x,
            grad_y,
            grad_x + grad_w,
            grad_y,
        )

        gradient.setColorAt(
            0.0,
            QtGui.QColor(
                0,
                255,
                0,
            ),
        )

        gradient.setColorAt(
            1.0,
            QtGui.QColor(
                255,
                0,
                0,
            ),
        )

        p.setBrush(
            QtGui.QBrush(gradient)
        )

        p.drawRoundedRect(
            grad_x,
            grad_y,
            grad_w,
            grad_h,
            6,
            6,
        )

        p.setPen(
            QtGui.QPen(
                QtGui.QColor(
                    200,
                    200,
                    200,
                ),
                1,
            )
        )

        p.drawRect(
            grad_x,
            grad_y,
            grad_w,
            grad_h,
        )

        p.setFont(
            QtGui.QFont(
                "Consolas",
                8,
            )
        )

        p.drawText(
            grad_x,
            grad_y + grad_h + 12,
            "0 N",
        )

        p.drawText(
            grad_x + grad_w - 22,
            grad_y + grad_h + 12,
            "2 N",
        )

        p.end()


# ============================================================================
# GRIP QUALITY
# ============================================================================

class GripBar(QtWidgets.QWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        self._pct = 0.0

        self.setFixedHeight(44)
        self.setMinimumWidth(240)

    def set_pct(self, pct):

        self._pct = max(
            0.0,
            min(100.0, pct),
        )

        self.update()

    def paintEvent(self, _):

        p = QtGui.QPainter(self)

        p.setRenderHint(
            QtGui.QPainter.Antialiasing
        )

        W = self.width()
        H = self.height()
        r = 8

        p.setPen(QtCore.Qt.NoPen)

        p.setBrush(
            QtGui.QBrush(
                QtGui.QColor(
                    30,
                    30,
                    30,
                )
            )
        )

        p.drawRoundedRect(
            0,
            0,
            W,
            H,
            r,
            r,
        )

        fw = int(
            W * self._pct / 100.0
        )

        t = self._pct / 100.0

        p.setBrush(
            QtGui.QBrush(
                QtGui.QColor(
                    int(255 * (1 - t)),
                    int(200 * t),
                    0,
                )
            )
        )

        if fw > 0:

            p.drawRoundedRect(
                0,
                0,
                fw,
                H,
                r,
                r,
            )

        p.setPen(
            QtGui.QPen(
                QtGui.QColor(
                    255,
                    255,
                    255,
                )
            )
        )

        p.setFont(
            QtGui.QFont(
                "Consolas",
                14,
                QtGui.QFont.Bold,
            )
        )

        p.drawText(
            0,
            0,
            W,
            H,
            QtCore.Qt.AlignCenter,
            f"{self._pct:.0f}%",
        )

        p.end()


# ============================================================================
# ROS NODE
# ============================================================================

class UINode(Node):

    def __init__(self):

        super().__init__("ug_ui")

        self.latest_status = None
        self.latest_tactile = None
        self.hardware_ready = False

        # Grip action state
        self.grip_state = "IDLE"
        self.grip_angle_deg = 0.0
        self.grip_fz = 0.0
        self.grip_result = ""

        # Last service result
        self.last_service = ""
        self.last_service_success = False
        self.last_service_message = ""

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------

        self.status_sub = self.create_subscription(
            GripperStatus,
            "/gripper/status",
            self._status_callback,
            10,
        )

        self.tactile_sub = self.create_subscription(
            TactileForces,
            "/gripper/tactile_forces",
            self._tactile_callback,
            10,
        )
        self.ready_sub = self.create_subscription(
            Bool,
            "/gripper/hardware_ready",
            self._hardware_ready_callback,
            10,
        )

        # ------------------------------------------------------------
        # Services
        # ------------------------------------------------------------

        self.jog_client = self.create_client(
            JogMotor,
            "/gripper/jog",
        )

        self.stop_client = self.create_client(
            StopMotor,
            "/gripper/stop",
        )

        self.goto_client = self.create_client(
            GoToPosition,
            "/gripper/go_to",
        )

        # ------------------------------------------------------------
        # Action
        # ------------------------------------------------------------

        self.grip_client = ActionClient(
            self,
            GripCommand,
            "/gripper/grip",
        )

        self.get_logger().info(
            "UG UI ROS node started."
        )

    def _status_callback(self, msg):

        self.latest_status = msg

    def _tactile_callback(self, msg):

        self.latest_tactile = msg

    def _hardware_ready_callback(self, msg):

        if msg.data:
            self.hardware_ready = True

    # ----------------------------------------------------------------
    # STOP
    # ----------------------------------------------------------------

    def stop_motor(self):

        if not self.stop_client.service_is_ready():

            self.get_logger().warning(
                "STOP service not available."
            )

            return

        request = StopMotor.Request()

        future = self.stop_client.call_async(
            request
        )

        future.add_done_callback(
            self._stop_done
        )

    def _stop_done(self, future):

        try:

            response = future.result()

            self.last_service = "STOP"
            self.last_service_success = bool(response.success)
            self.last_service_message = (
                "Motor stopped"
                if response.success
                else "STOP failed"
            )

            self.get_logger().info(
                f"STOP: {response.success}"
            )

        except Exception as exc:

            self.last_service = "STOP"
            self.last_service_success = False
            self.last_service_message = str(exc)

            self.get_logger().error(
                f"STOP error: {exc}"
            )

    # ----------------------------------------------------------------
    # JOG
    # ----------------------------------------------------------------

    def jog(self, direction):

        if not self.jog_client.service_is_ready():

            self.get_logger().warning(
                "JOG service not available."
            )

            return

        request = JogMotor.Request()

        request.direction = direction

        future = self.jog_client.call_async(
            request
        )

        future.add_done_callback(
            self._jog_done
        )

    def _jog_done(self, future):

        try:

            response = future.result()

            self.last_service = "JOG"
            self.last_service_success = bool(response.success)
            self.last_service_message = response.message

            self.get_logger().info(
                f"JOG: {response.message}"
            )

        except Exception as exc:

            self.last_service = "JOG"
            self.last_service_success = False
            self.last_service_message = str(exc)

            self.get_logger().error(
                f"JOG error: {exc}"
            )

    # ----------------------------------------------------------------
    # GO TO
    # ----------------------------------------------------------------

    def go_to(self, mode, target):

        if not self.goto_client.service_is_ready():

            self.get_logger().warning(
                "GO TO service not available."
            )

            return

        request = GoToPosition.Request()

        request.mode = mode.lower()

        request.target = float(target)

        future = self.goto_client.call_async(
            request
        )

        future.add_done_callback(
            self._goto_done
        )

    def _goto_done(self, future):

        try:

            response = future.result()

            self.last_service = "GO TO"
            self.last_service_success = bool(response.success)
            self.last_service_message = (
                f"{response.message} | "
                f"angle={response.final_angle_deg:.2f}° | "
                f"opening={response.final_opening_mm / 10:.2f} cm"
            )

            self.get_logger().info(
                f"GO TO: {response.message}"
            )

        except Exception as exc:

            self.last_service = "GO TO"
            self.last_service_success = False
            self.last_service_message = str(exc)

            self.get_logger().error(
                f"GO TO error: {exc}"
            )

    # ----------------------------------------------------------------
    # GRIP ACTION
    # ----------------------------------------------------------------

    def grip(self, command):

        if not self.grip_client.server_is_ready():

            self.get_logger().warning(
                "Grip action server not available."
            )

            return

        goal = GripCommand.Goal()

        goal.command = command

        future = self.grip_client.send_goal_async(
            goal,
            feedback_callback=self._grip_feedback,
        )

        future.add_done_callback(
            self._grip_goal_response
        )

    def _grip_goal_response(self, future):

        try:

            goal_handle = future.result()

            if not goal_handle.accepted:

                self.get_logger().warning(
                    "Grip goal rejected."
                )

                return

            result_future = (
                goal_handle.get_result_async()
            )

            result_future.add_done_callback(
                self._grip_result
            )

        except Exception as exc:

            self.get_logger().error(
                f"Grip action error: {exc}"
            )

    def _grip_feedback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.grip_state = feedback.current_state
        self.grip_angle_deg = float(feedback.angle_deg)
        self.grip_fz = float(feedback.fz)

        self.get_logger().debug(
            f"Grip feedback: "
            f"{feedback.current_state} | "
            f"angle={feedback.angle_deg:.2f}° | "
            f"Fz={feedback.fz:.2f} N"
        )

    def _grip_result(self, future):

        try:

            result = future.result().result

            self.grip_state = result.final_state
            self.grip_angle_deg = float(result.final_angle_deg)
            self.grip_fz = float(result.final_fz)

            self.grip_result = (
                "SUCCESS"
                if result.success
                else "FAILED"
            )

            self.get_logger().info(
                f"Grip result: "
                f"{self.grip_result} | "
                f"{result.final_state} | "
                f"angle={result.final_angle_deg:.2f}° | "
                f"Fz={result.final_fz:.2f} N"
            )

        except Exception as exc:

            self.grip_result = "ERROR"

            self.get_logger().error(
                f"Grip result error: {exc}"
            )


# ============================================================================
# MAIN WINDOW
# ============================================================================

class MainWindow(QtWidgets.QWidget):

    _H_CTRL = 110

    def closeEvent(self, event):
        self.ros_node.get_logger().info(
            "UI closed. Shutting down ROS node..."
        )

        if rclpy.ok():
            rclpy.shutdown()

        event.accept()

    def __init__(self, ros_node):

        super().__init__()

        self.ros_node = ros_node

        self.setWindowTitle(
            "UG Force Monitor"
        )

        self.setFixedSize(
            1500,
            900,
        )

        self.setStyleSheet(
            "QWidget{"
            "background:#080808;"
            "color:white;"
            "}"
        )

        self._build_ui()

        self._timer = QtCore.QTimer()

        self._timer.timeout.connect(
            self._update_ui
        )

        self._timer.start(100)

    # ========================================================================
    # UI BUILD
    # ========================================================================

    def _build_ui(self):

        root = QtWidgets.QVBoxLayout(self)

        root.setSpacing(2)

        # --------------------------------------------------------------------
        # TITLE
        # --------------------------------------------------------------------

        top = QtWidgets.QVBoxLayout()

        top.setSpacing(8)
        top.setAlignment(
            QtCore.Qt.AlignCenter
        )

        title = QtWidgets.QLabel(
            "UG Force Monitor"
        )

        title.setAlignment(
            QtCore.Qt.AlignCenter
        )

        title.setStyleSheet(
            "font-size:28px;"
            "font-weight:bold;"
            "color:#00D8FF;"
            "padding:2px 14px;"
        )

        self._state_label = QtWidgets.QLabel(
            "State: IDLE"
        )

        self._state_label.setAlignment(
            QtCore.Qt.AlignCenter
        )

        self._state_label.setFixedSize(
            260,
            44,
        )

        self._state_label.setStyleSheet(
            "font-size:20px;"
            "font-weight:bold;"
            "color:#aaa;"
            "padding:4px 12px;"
            "border:2px solid #444;"
            "border-radius:7px;"
        )

        top.addWidget(
            title,
            alignment=QtCore.Qt.AlignCenter,
        )

        top.addWidget(
            self._state_label,
            alignment=QtCore.Qt.AlignCenter,
        )

        self._grip_action_label = QtWidgets.QLabel(
            "Grip Action: IDLE"
        )

        self._grip_action_label.setAlignment(
            QtCore.Qt.AlignCenter
        )

        self._grip_action_label.setFixedSize(
            320,
            36,
        )

        self._grip_action_label.setStyleSheet(
            "font-size:16px;"
            "font-weight:bold;"
            "color:#777;"
            "padding:2px 10px;"
            "border:1px solid #333;"
            "border-radius:6px;"
        )

        top.addWidget(
            self._grip_action_label,
            alignment=QtCore.Qt.AlignCenter,
        )

        root.addLayout(top)

        # --------------------------------------------------------------------
        # CENTER GRID
        # --------------------------------------------------------------------

        grid = QtWidgets.QGridLayout()

        grid.setSpacing(5)

        grid.setColumnStretch(0, 27)
        grid.setColumnStretch(1, 46)
        grid.setColumnStretch(2, 27)

        grid.setRowMinimumHeight(
            0,
            220,
        )

        grid.setRowMinimumHeight(
            1,
            220,
        )

        positions = {
            3: (0, 0),
            2: (0, 2),
            0: (1, 0),
            1: (1, 2),
        }

        self._curve_fx = []
        self._curve_fy = []
        self._curve_fz = []

        self._plot_widgets = []

        for i in range(4):

            pw = pg.PlotWidget(
                title=(
                    f"<span style='color:#cccccc;"
                    f"font-size:15px;"
                    f"font-weight:bold'>"
                    f"T{i + 1}"
                    f"</span>"
                )
            )

            pw.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Fixed,
            )

            pw.setMaximumHeight(400)

            pw.setBackground(
                "#0c0c0c"
            )

            pw.showGrid(
                x=True,
                y=True,
                alpha=0.10,
            )

            pw.setYRange(
                -3,
                9,
            )

            legend = pw.addLegend(
                offset=(8, 8)
            )

            legend.setLabelTextSize(
                "12pt"
            )

            self._curve_fx.append(
                pw.plot(
                    pen=pg.mkPen(
                        "#ff4d4d",
                        width=2,
                    ),
                    name="Fx",
                )
            )

            self._curve_fy.append(
                pw.plot(
                    pen=pg.mkPen(
                        "#00ff99",
                        width=2,
                    ),
                    name="Fy",
                )
            )

            self._curve_fz.append(
                pw.plot(
                    pen=pg.mkPen(
                        "#33aaff",
                        width=2,
                    ),
                    name="Fz",
                )
            )

            self._plot_widgets.append(
                pw
            )

            r, c = positions[i]

            grid.addWidget(
                pw,
                r,
                c,
            )

        self._gripper_overlay = (
            GripperOverlay()
        )

        grid.addWidget(
            self._gripper_overlay,
            0,
            1,
            2,
            1,
        )

        root.addLayout(
            grid,
            stretch=10,
        )

        # --------------------------------------------------------------------
        # STATUS
        # --------------------------------------------------------------------

        stat = QtWidgets.QHBoxLayout()

        stat.setSpacing(10)

        self._avg_fz_lbl = self._slbl(
            "Avg Fz:  0.00 N",
            "#33aaff",
        )

        self._avg_fx_lbl = self._slbl(
            "Avg Fx:  0.00 N",
            "#ff4d4d",
        )

        self._avg_fy_lbl = self._slbl(
            "Avg Fy:  0.00 N",
            "#00ff99",
        )

        self._angle_lbl = self._slbl(
            "Motor:  0.00°",
            "#FFD700",
        )

        self._opening_lbl = self._slbl(
            "Opening:  0.00 cm",
            "#00FF99",
        )

        grip_label = QtWidgets.QLabel(
            "Grip Quality:"
        )

        grip_label.setStyleSheet(
            "font-size:18px;"
            "font-weight:bold;"
            "color:#aaa;"
            "padding:0 8px;"
        )

        self._grip_bar = GripBar()

        for widget in (
            self._avg_fz_lbl,
            self._avg_fx_lbl,
            self._avg_fy_lbl,
            self._angle_lbl,
            self._opening_lbl,
            grip_label,
            self._grip_bar,
        ):

            stat.addWidget(widget)

        stat.addStretch()

        root.addLayout(stat)

        # --------------------------------------------------------------------
        # SERVICE RESULT
        # --------------------------------------------------------------------

        self._service_result_label = QtWidgets.QLabel(
            "Service: IDLE"
        )

        self._service_result_label.setAlignment(
            QtCore.Qt.AlignCenter
        )

        self._service_result_label.setFixedHeight(
            34
        )

        self._service_result_label.setStyleSheet(
            "font-size:15px;"
            "font-weight:bold;"
            "color:#777;"
            "padding:2px 10px;"
            "border:1px solid #333;"
            "border-radius:6px;"
        )

        root.addWidget(
            self._service_result_label
        )

        # --------------------------------------------------------------------
        # CONTROLS
        # --------------------------------------------------------------------

        ctrl = QtWidgets.QHBoxLayout()

        ctrl.setSpacing(16)

        ctrl.addWidget(
            self._build_manual()
        )

        ctrl.addWidget(
            self._build_goto()
        )

        ctrl.addWidget(
            self._build_operation()
        )

        ctrl.addStretch()

        root.addLayout(ctrl)

    # ========================================================================
    # MANUAL
    # ========================================================================

    def _build_manual(self):

        box = QtWidgets.QGroupBox(
            "Control Manual"
        )

        box.setStyleSheet(
            self._grp()
        )

        box.setFixedHeight(
            self._H_CTRL
        )

        layout = QtWidgets.QHBoxLayout(box)

        layout.setSpacing(12)

        layout.setContentsMargins(
            14,
            22,
            14,
            14,
        )

        self._btn_open = self._jbtn(
            "OPEN"
        )

        self._btn_close = self._jbtn(
            "CLOSE"
        )

        btn_stop = self._jbtn(
            "STOP"
        )

        self._btn_open.clicked.connect(
            lambda: self.ros_node.jog(
                "open"
            )
        )

        self._btn_close.clicked.connect(
            lambda: self.ros_node.jog(
                "close"
            )
        )

        btn_stop.clicked.connect(
            self.ros_node.stop_motor
        )

        for button in (
            self._btn_open,
            self._btn_close,
            btn_stop,
        ):

            layout.addWidget(button)

        return box

    # ========================================================================
    # GO TO
    # ========================================================================

    def _build_goto(self):

        box = QtWidgets.QGroupBox(
            "Go to:"
        )

        box.setStyleSheet(
            self._grp()
        )

        box.setFixedHeight(
            self._H_CTRL
        )

        layout = QtWidgets.QHBoxLayout(box)

        layout.setSpacing(12)

        layout.setContentsMargins(
            14,
            22,
            14,
            14,
        )

        self._goto_mode = (
            QtWidgets.QComboBox()
        )

        self._goto_mode.addItems(
            [
                "Angle",
                "Opening",
            ]
        )

        self._goto_mode.setFixedSize(
            130,
            52,
        )

        self._goto_mode.setStyleSheet(
            "font-size:16px;"
            "background:#181818;"
            "color:#ccc;"
            "border:1px solid #3a3a3a;"
            "border-radius:6px;"
            "padding:4px;"
        )

        self._goto_mode.currentTextChanged.connect(
            self._update_goto_mode
        )

        self._goto_spin = (
            QtWidgets.QDoubleSpinBox()
        )

        self._goto_spin.setFixedSize(
            150,
            52,
        )

        self._goto_spin.setStyleSheet(
            "font-size:16px;"
            "background:#181818;"
            "color:#ccc;"
            "border:1px solid #3a3a3a;"
            "border-radius:6px;"
            "padding:4px;"
        )

        self._goto_spin.setRange(
            -360.0,
            360.0,
        )

        self._goto_spin.setSuffix(
            " °"
        )

        btn_go = self._jbtn(
            "GO"
        )

        btn_cancel = self._jbtn(
            "✕"
        )

        btn_go.clicked.connect(
            self._on_goto_clicked
        )

        btn_cancel.clicked.connect(
            self.ros_node.stop_motor
        )

        self._goto_status = QtWidgets.QLabel(
            "Idle"
        )

        self._goto_status.setStyleSheet(
            "font-size:16px;"
            "color:#777;"
            "padding:0 8px;"
        )

        for widget in (
            self._goto_mode,
            self._goto_spin,
            btn_go,
            btn_cancel,
            self._goto_status,
        ):

            layout.addWidget(widget)

        return box

    # ========================================================================
    # OPERATION
    # ========================================================================

    def _build_operation(self):

        box = QtWidgets.QGroupBox(
            "Gripper Operation"
        )

        box.setStyleSheet(
            self._grp()
        )

        box.setFixedHeight(
            self._H_CTRL
        )

        layout = QtWidgets.QHBoxLayout(box)

        layout.setSpacing(14)

        layout.setContentsMargins(
            14,
            22,
            14,
            14,
        )

        grip = self._cbtn(
            "GRIP",
            "#FFD700",
        )

        release = self._cbtn(
            "RELEASE",
            "#00C875",
        )

        stop = self._cbtn(
            "STOP",
            "#cc2200",
        )

        grip.clicked.connect(
            lambda: self.ros_node.grip(
                "close"
            )
        )

        release.clicked.connect(
            lambda: self.ros_node.grip(
                "open"
            )
        )

        stop.clicked.connect(
            lambda: self.ros_node.grip(
                "cancel"
            )
        )

        for button in (
            grip,
            release,
            stop,
        ):

            layout.addWidget(button)

        return box

    # ========================================================================
    # HELPERS
    # ========================================================================

    @staticmethod
    def _slbl(
        text,
        color,
        border_color="#ffffff",
    ):

        label = QtWidgets.QLabel(
            text
        )

        label.setFixedHeight(
            44
        )

        label.setStyleSheet(
            f"font-size:17px;"
            f"font-weight:bold;"
            f"color:{color};"
            f"padding:4px 14px;"
            f"border:1px solid {border_color};"
            f"border-radius:6px;"
        )

        return label

    @staticmethod
    def _cbtn(text, color):

        button = QtWidgets.QPushButton(
            text
        )

        button.setFixedSize(
            120,
            68,
        )

        button.setStyleSheet(
            f"QPushButton{{"
            f"font-size:18px;"
            f"font-weight:bold;"
            f"background:{color};"
            f"color:#000;"
            f"border-radius:9px;"
            f"}}"
            f"QPushButton:hover{{"
            f"background:#ffffff;"
            f"}}"
        )

        return button

    @staticmethod
    def _jbtn(text):

        button = QtWidgets.QPushButton(
            text
        )

        button.setFixedSize(
            110,
            56,
        )

        button.setStyleSheet(
            """
            QPushButton{
                font-size:15px;
                font-weight:bold;
                background:#181818;
                color:#cccccc;
                border:1px solid #383838;
                border-radius:7px;
            }

            QPushButton:hover{
                border:1px solid #00D8FF;
                color:#ffffff;
            }

            QPushButton:disabled{
                color:#404040;
                border-color:#202020;
            }
            """
        )

        return button

    @staticmethod
    def _grp():

        return (
            "QGroupBox{"
            "font-size:16px;"
            "font-weight:bold;"
            "color:#888;"
            "border:1px solid #2a2a2a;"
            "border-radius:9px;"
            "margin-top:12px;"
            "padding-top:4px;"
            "}"
            "QGroupBox::title{"
            "subcontrol-origin:margin;"
            "left:14px;"
            "padding:0 8px;"
            "}"
        )

    # ========================================================================
    # GO TO MODE
    # ========================================================================

    def _update_goto_mode(self, mode):

        if mode == "Angle":

            self._goto_spin.setRange(
                -360.0,
                360.0,
            )

            self._goto_spin.setSuffix(
                " °"
            )

        else:

            self._goto_spin.setRange(
                0.0,
                8.5,
            )

            self._goto_spin.setSuffix(
                " cm"
            )

    def _on_goto_clicked(self):

        mode = (
            self._goto_mode.currentText()
        )

        target = (
            self._goto_spin.value()
        )

        self._goto_status.setText(
            f"→ {target:.1f}"
            + (
                "°"
                if mode == "Angle"
                else " cm"
            )
        )

        target_for_service = (
            target * 10.0
            if mode == "Opening"
            else target
        )


        self.ros_node.go_to(
            mode,
            target_for_service,
        )

    # ========================================================================
    # UI UPDATE
    # ========================================================================

    def _update_ui(self):

        status = (
            self.ros_node.latest_status
        )

        if status is not None:

            state = status.state

            color = _STATE_COLORS.get(
                state,
                "#ffffff",
            )

            self._state_label.setText(
                f"State: {state}"
            )

            self._state_label.setStyleSheet(
                f"font-size:20px;"
                f"font-weight:bold;"
                f"color:{color};"
                f"padding:4px 12px;"
                f"border:2px solid {color};"
                f"border-radius:7px;"
            )

            self._angle_lbl.setText(
                f"Motor: "
                f"{status.angle_deg:.2f}°"
            )

            self._opening_lbl.setText(
                f"Opening: "
                f"{status.opening_mm / 10:.2f} cm"
            )

        # ------------------------------------------------------------
        # GRIP ACTION STATUS
        # ------------------------------------------------------------

        grip_state = self.ros_node.grip_state
        grip_result = self.ros_node.grip_result
        grip_angle = self.ros_node.grip_angle_deg
        grip_fz = self.ros_node.grip_fz

        if grip_result == "SUCCESS":
            grip_color = "#00FF99"
            grip_text = "SUCCESS"

        elif grip_result == "FAILED":
            grip_color = "#ff4d4d"
            grip_text = "FAILED"

        elif grip_result == "ERROR":
            grip_color = "#ff4d4d"
            grip_text = "ERROR"

        elif grip_state != "IDLE":
            grip_color = "#FFD700"
            grip_text = grip_state

        else:
            grip_color = "#777"
            grip_text = "IDLE"

        self._grip_action_label.setText(
            f"Grip Action: {grip_text} | "
            f"{grip_angle:.1f}° | "
            f"Fz {grip_fz:.2f} N"
        )

        self._grip_action_label.setStyleSheet(
            f"font-size:16px;"
            f"font-weight:bold;"
            f"color:{grip_color};"
            f"padding:2px 10px;"
            f"border:1px solid {grip_color};"
            f"border-radius:6px;"
        )

        # ------------------------------------------------------------
        # SERVICE RESULT STATUS
        # ------------------------------------------------------------

        service = self.ros_node.last_service
        service_success = self.ros_node.last_service_success
        service_message = self.ros_node.last_service_message

        if service:

            service_color = (
                "#00FF99"
                if service_success
                else "#ff4d4d"
            )

            self._service_result_label.setText(
                f"{service}: {service_message}"
            )

            self._service_result_label.setStyleSheet(
                f"font-size:15px;"
                f"font-weight:bold;"
                f"color:{service_color};"
                f"padding:2px 10px;"
                f"border:1px solid {service_color};"
                f"border-radius:6px;"
            )

        # ------------------------------------------------------------
        # TACTILE
        # ------------------------------------------------------------

        tactile = (
            self.ros_node.latest_tactile
        )

        if tactile is None:
            return

        texels = tactile.texels

        # ------------------------------------------------------------
        # Convert ROS tactile message into fixed T1-T4 order
        # ------------------------------------------------------------

        ordered = list(texels)

        # Current hardware interface publishes
        # config.TEXEL_IDS in the same order as the original UI.
        while len(ordered) < 4:

            class Empty:
                fx = 0.0
                fy = 0.0
                fz = 0.0

            ordered.append(
                Empty()
            )

        # ------------------------------------------------------------
        # Graphs
        # ------------------------------------------------------------

        for i in range(4):

            fx = float(
                ordered[i].fx
            )

            fy = float(
                ordered[i].fy
            )

            fz = float(
                ordered[i].fz
            )

            # Store short history locally
            if not hasattr(
                self,
                "_history_fx",
            ):

                self._history_fx = [
                    []
                    for _ in range(4)
                ]

                self._history_fy = [
                    []
                    for _ in range(4)
                ]

                self._history_fz = [
                    []
                    for _ in range(4)
                ]

            self._history_fx[i].append(fx)
            self._history_fy[i].append(fy)
            self._history_fz[i].append(fz)

            max_points = 100

            self._history_fx[i] = (
                self._history_fx[i][-max_points:]
            )

            self._history_fy[i] = (
                self._history_fy[i][-max_points:]
            )

            self._history_fz[i] = (
                self._history_fz[i][-max_points:]
            )

            self._curve_fx[i].setData(
                self._history_fx[i]
            )

            self._curve_fy[i].setData(
                self._history_fy[i]
            )

            self._curve_fz[i].setData(
                self._history_fz[i]
            )

            texel_id = int(
                ordered[i].texel_id
            )

            self._plot_widgets[i].setTitle(
                f"<span style='color:#cccccc;"
                f"font-size:15px;"
                f"font-weight:bold'>"
                f"T{i + 1} | ID {texel_id} | "
                f"Fx={fx:+.2f} "
                f"Fy={fy:+.2f} "
                f"Fz={fz:.2f}N"
                f"</span>"
            )

        # ------------------------------------------------------------
        # Averages
        # ------------------------------------------------------------

        n = len(ordered)

        avg_fx = sum(
            float(x.fx)
            for x in ordered
        ) / n

        avg_fy = sum(
            float(x.fy)
            for x in ordered
        ) / n

        self._avg_fz_lbl.setText(
            f"Avg Fz: "
            f"{tactile.avg_fz:+.2f} N"
        )

        self._avg_fx_lbl.setText(
            f"Avg Fx: "
            f"{avg_fx:+.2f} N"
        )

        self._avg_fy_lbl.setText(
            f"Avg Fy: "
            f"{avg_fy:+.2f} N"
        )

        # ------------------------------------------------------------
        # Grip quality
        # ------------------------------------------------------------

        quality = sum(
            min(
                max(
                    0.0,
                    float(x.fz)
                ) / FZ_TARGET_N,
                1.0,
            )
            for x in ordered
        )

        quality = (
            quality / n
        ) * 100.0

        self._grip_bar.set_pct(
            quality
        )

        # ------------------------------------------------------------
        # Gripper image overlay
        # ------------------------------------------------------------

        self._gripper_overlay.set_forces(
            ordered
        )


# ============================================================================
# MAIN
# ============================================================================

def main(args=None):

    app = QtWidgets.QApplication(
        sys.argv
    )

    rclpy.init(
        args=args
    )

    ros_node = UINode()

    window = MainWindow(
        ros_node
    )

    window.show()

    # ROS 2 spin integrated into Qt
    ros_timer = QtCore.QTimer()

    def spin_ros():

        rclpy.spin_once(
            ros_node,
            timeout_sec=0.0,
        )

    ros_timer.timeout.connect(
        spin_ros
    )

    ros_timer.start(10)

    exit_code = app.exec_()

    ros_node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(
        exit_code
    )

if __name__ == "__main__":
    main()
