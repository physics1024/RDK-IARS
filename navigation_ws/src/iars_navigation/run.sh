source /home/sunrise/jianye_dev/rplidar_ws/install/setup.bash
exec ros2 launch rplidar_ros rplidar_c1_launch.py &
exec ros2 launch iars_base iars_bringup.launch.py &
exec ros2 launch iars_navigation nav_bringup.launch.py &
exec ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 1 &