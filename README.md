# foldy

## Installation

### RK1 computer setup:

1) Clone the directory:
```
$ git clone 
```

2) Go to the folder of the projectand build the project.
```
$ cd foldy
$ docker compose -f docker-compose-neato.yaml build
```

## Bring up Neato docker images:

### Start the base
```
$ docker compose -f docker-compose-neato.yaml up -d --force-recreate neato-base ros2-base
```

### Mapping
```
$ docker compose -f docker-compose-neato.yaml up -d --force-recreate neato-base ros2-base neato-cartographer
```

#### Save map:
```
$ ros2 run nav2_map_server map_saver_cli --free 0.196 --ros-args -p save_map_timeout:=5000.0
```

### Visualise the map:
```
$ xhost + && docker compose -f docker-compose-desktop.yaml up -d
```

## Navigation

On RK1:
```
docker compose -f docker-compose-neato.yaml up -d --force-recreate neato-base ros2-base
```

On the Desktop PC:
```
xhost + && docker compose -f docker-compose-desktop.yaml up rviz2-nav -d --force-recreate
```