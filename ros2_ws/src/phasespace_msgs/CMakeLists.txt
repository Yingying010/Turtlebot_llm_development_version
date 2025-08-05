cmake_minimum_required(VERSION 3.5)
project(phasespace_msgs)
 
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # 如果你某些msg依赖它就加上
 
set(msg_files
  "msg/Camera.msg"
  "msg/Cameras.msg"
  "msg/Marker.msg"
  "msg/Markers.msg"
  "msg/Rigid.msg"
  "msg/Rigids.msg"
)
 
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)
 
ament_export_dependencies(rosidl_default_runtime)
ament_package()
