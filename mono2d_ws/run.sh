export CAM_TYPE=usb
exec ros2 launch mono2d_body_detection mono2d_body_detection.launch.py device:=/dev/video2 &
exec ros2 run body_follow body_follow &