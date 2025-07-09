export CAM_TYPE=usb
exec ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py &
exec ros2 run falldown_alert falldown_alert &