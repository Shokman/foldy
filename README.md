# foldy

Bring up Neato docker images:

## Start the base
docker compose -f docker-compose-neato.yaml up -d --force-recreate neato-base ros2-base

## Mapping

docker compose -f docker-compose-neato.yaml up -d --force-recreate neato-base ros2-base neato-cartographer

### Save map:
ros2 run nav2_map_server map_saver_cli --free 0.196 --ros-args -p save_map_timeout:=5000.0