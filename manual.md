ros2 launch turtlebot3_bringup robot.launch.py namespace:=robot1

ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/robot1/cmd_vel

git config lfs.https://github.com/Yingying010/Turtlebot_llm_development_version.git/info/lfs.locksverify false

CUDA_VISIBLE_DEVICES=0 GRADIO_SHARE_PORT=7860 llamafactory-cli webui

export PULSE_SERVER=tcp:192.168.0.208
python3 ttsRepo/stream_tts_test.py

export YOLO_SOURCE="udp://@:8888?fifo_size=1000000&overrun_nonfatal=1&buffer_size=1000000&probesize=32&analyzeduration=0"

rpicam-vid -t 0 --width 640 --height 480 --framerate 30 \
  --codec h264 --profile baseline --inline --intra 10 --bitrate 2000000 \
  -o udp://<你的电脑IP>:8888

