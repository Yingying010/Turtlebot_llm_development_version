ros2 launch turtlebot3_bringup robot.launch.py namespace:=robot1

ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/robot1/cmd_vel

git config lfs.https://github.com/Yingying010/Turtlebot_llm_development_version.git/info/lfs.locksverify false

CUDA_VISIBLE_DEVICES=0 GRADIO_SHARE_PORT=7860 llamafactory-cli webui

export PULSE_SERVER=tcp:192.168.0.208
python3 ttsRepo/stream_tts_test.py