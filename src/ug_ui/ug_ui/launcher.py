import sys

from PyQt5 import QtWidgets


def main():
    app = QtWidgets.QApplication(sys.argv)

    import rclpy
    from . import ui

    rclpy.init()

    ros_node = ui.UINode()

    window = ui.MainWindow(ros_node)

    ros_timer = ui.QtCore.QTimer()

    def spin_ros():
        try:
            if rclpy.ok():
                rclpy.spin_once(
                    ros_node,
                    timeout_sec=0.0,
                )
        except (KeyboardInterrupt, rclpy.RCLError):
            pass

    ros_timer.timeout.connect(spin_ros)
    ros_timer.start(10)

    ready_timer = ui.QtCore.QTimer()

    def show_when_hardware_ready():
        if ros_node.hardware_ready:
            ready_timer.stop()
            window.show()

    ready_timer.timeout.connect(
        show_when_hardware_ready
    )
    ready_timer.start(50)

    import signal

    def handle_sigint(signum, frame):
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    exit_code = app.exec_()

    ros_node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
