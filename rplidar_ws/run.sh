source /home/sunrise/jianye_dev/rplidar_ws/install/setup.bash
exec ros2 launch rplidar_ros rplidar_c1_launch.py &
exec ros2 launch iars_base iars_bringup.launch.py &
exec ros2 launch iars_navigation cartographer.launch.py &
exec ros2 run nav2_map_server map_saver_cli -t map -f real &