cmake_minimum_required(VERSION 3.5)
project(phasespace_client)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(include)

# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(phasespace_msgs REQUIRED)

# 导入 OWL SDK 的动态库
add_library(owlsock SHARED IMPORTED)
set_target_properties(owlsock PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/bin/libowlsock.so
)

# psnode 可执行文件
add_executable(psnode src/psnode.cpp)
target_link_libraries(psnode owlsock)
ament_target_dependencies(psnode rclcpp std_msgs phasespace_msgs)

# listener 可执行文件
add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs phasespace_msgs)

# 安装目标
install(TARGETS psnode listener
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件（如有）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()




