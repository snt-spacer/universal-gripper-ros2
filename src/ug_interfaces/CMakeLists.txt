cmake_minimum_required(VERSION 3.8)
project(ug_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TactileReading.msg"
  "msg/TactileForces.msg"
  "msg/GripperStatus.msg"
  "srv/CalibrateSensors.srv"
  "srv/StopMotor.srv"
  "srv/JogMotor.srv"
  "srv/GoToPosition.srv"
  "action/GripCommand.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_package()
