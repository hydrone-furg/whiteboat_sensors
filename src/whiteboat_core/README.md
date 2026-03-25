# whiteboat_core

## To clone with SSH:
### [SSH Tutorial](https://phoenixnap.com/kb/git-clone-ssh)

## To use SITL:
### [Rover SITL](https://ardupilot.org/dev/docs/rover-sitlmavproxy-tutorial.html)
```
git clone --recurse-submodules git@github.com:ArduPilot/ardupilot.git
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
cd ~/ardupilot/Rover
sim_vehicle.py --map --console
```
### To use with QGroundControl: set <ins>comm link</ins> to UDP -> Port: 14551
```
sim_vehicle.py -v Rover -f sailboat-motor --map --console -l -32.0782,-52.1501,0,0 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552
```
## Dependencies:
### [Ubuntu 20.04.6 LTS (Focal Fossa)](https://releases.ubuntu.com/focal/)
### [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
### [whiteboat-tools](https://github.com/hydrone-furg/whiteboat-tools/tree/develop)

## Creating the workspace:
```
mkdir -p ~/<your_ws>/src
cd ~/<your_ws>/
catkin_make
```

## Creating the package and building:
```
cd ~/<your_ws>/src
git clone git@github.com:hydrone-furg/whiteboat_core.git --recursive
cd ~/<your_ws>/
catkin_make
. ~/<your_ws>/devel/setup.bash
source devel/setup.bash
```
Replace **<your_ws>** with the name of your workspace.

## Terminal 1:
```
roscore
```

## Terminal 2:
```
rosrun whiteboat_core whiteboat_node.py
```

## Quick Commands:
### Roscore:
```
catkin_make && source devel/setup.bash && roscore
```
### Whiteboat Node:
```
catkin_make && source devel/setup.bash && rosrun whiteboat_core whiteboat_node.py
```
### Data stream server:
```
catkin_make && source devel/setup.bash && rosrun whiteboat_core data_stream_server.py
```
### GPS Topic:
```
catkin_make && source devel/setup.bash && rostopic echo /whiteboat/gps/fix
```
### IMU Topic:
```
catkin_make && source devel/setup.bash && rostopic echo /whiteboat/imu/data
```