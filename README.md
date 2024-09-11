# foldy

## Mapping commands

### Visualisation (remote):

ros2 run rviz2 rviz2 -d /opt/ros/humble/share/cartographer_ros/configuration_files/demo_2d.rviz
ros2 run teleop_twist_keyboard teleop_twist_keyboard

### Save map:
ros2 run nav2_map_server map_saver_cli --free 0.196 --ros-args -p save_map_timeout:=5000.0