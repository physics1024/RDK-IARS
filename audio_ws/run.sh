exec ros2 launch sensevoice_ros2 sensevoice_ros2.launch.py micphone_name:="plughw:1,0" &
exec ros2 run hobot_llamacpp hobot_llamacpp --ros-args -p feed_type:=2 -p system_prompt:="config/system_prompt.txt" --ros-args --log-level warn &
exec ros2 run tts_receive tts_receive
