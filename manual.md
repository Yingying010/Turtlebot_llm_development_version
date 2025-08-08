ros2 launch turtlebot3_bringup robot.launch.py namespace:=robot1

ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/robot1/cmd_vel