# Control package and emulate sensors for Heron simulation
Water drone traverses a given area and get data about temperature from temperature map.
Gazebo simulation was given from https://github.com/heron

## Installation
Install heron repositories:
```bash
cd catkin_ws/src
git clone https://github.com/heron/heron_simulator
git clone https://github.com/heron/heron.git
git clone https://github.com/heron/heron_controller.git
git clone https://github.com/heron/heron_desktop.git
git clone https://github.com/heron/heron_worlds.git
git clone https://github.com/ros-visualization/interactive_marker_twist_server.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=melodic -yr
```
Clone control package:
```bash
git clone https://github.com/LoSk-p/heron_sensors
```
Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Create way
In `utils/borders` write coordinates of your border's points.
Run `way_lodka.py`:
```
rosrun heron_sensors way_lodka.py
```
## Run
Run Gazebo simulation:
```bash
roslaunch heron_gazebo heron_lake_world.launch
```
Run control package:
```bash
rosrun heron_sensors one_heron.py
```
