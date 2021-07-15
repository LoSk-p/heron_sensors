# Control package and emulate sensors for Heron simulation
Water drone traverses a given area and get data about temperature from temperature map.
Gazebo simulation was given from https://github.com/heron

## Repo structure

scripts/
* drone_control.py -- main script for drone control 

utils/
   - ways/ -- trajectories for meandr
   *  map/ -- parsed data for simulation 
   * experimental_data/ -- unparsed data from experiments
   * devide_area.py -- devide the area for several parts (use for several  drones)
   * map_creation.py -- create a simulation map from experimental data
   * trajectory_creation.py -- create points for meandr 


## Installation

Required dependencies:

```bash
sudo apt-get install ros-melodic-gazebo-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller ros-melodic-imu-tools ros-melodic-uuv-simulator ros-melodic-lms1xx
pip3 install matplotlib
sudo apt-get install python3-empy
```

Install heron repositories:
```bash
cd catkin_ws/src
git clone https://github.com/heron/heron_simulator.git
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
cd ~/catkin_ws/src
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
Run `trajectory_creation`:
```
rosrun heron_sensors trajectory_creation.py
```
## Run
Run Gazebo simulation:
```bash
roslaunch heron_gazebo heron_lake_world.launch
```
Run control package:
```bash
rosrun heron_sensors drone_control.py
```
