ros2 launch turtlebot3_bringup robot.launch.py namespace:=robot1

ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/robot1/cmd_vel

git config lfs.https://github.com/Yingying010/Turtlebot_llm_development_version.git/info/lfs.locksverify false