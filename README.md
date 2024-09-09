# foldy

## Cartographer commands

### Installation

sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros

### Map

ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0

ros2 run cartographer_ros cartographer_node -configuration_directory ${PWD} -configuration_basename config.lua

ros2 run teleop_twist_keyboard teleop_twist_keyboard

### Save map:
ros2 run nav2_map_server map_saver_cli --free 0.196 --ros-args -p save_map_timeout:=5000.0

### Visualisation (remote):

ros2 run rviz2 rviz2 -d /opt/ros/humble/share/cartographer_ros/configuration_files/demo_2d.rviz