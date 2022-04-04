# ROS Scenario Simulation

This project is designed to test the openplanner algorithms, based on "common road" [1] and "openScenario"[2] files.
Currently commonroad scenario files are not supported out of the box. 
The commonroad data structure is used to describe the scenarios. 
Therefore a commonroad interface should be easy to implement.

## Scenarios
Currently the folowing scenarios are provided:
Simple swerve around object own our own lane without opposing traffic.

![Alt text](docs/overtake.png?raw=true "Overtake")

Left turn inside an intersection without stoplines or tarfficlights.

![Alt text](docs/left_turn.png?raw=true "left_turn")

## Simulation/ROS Node Architecture
![Alt text](docs/node_graph.png?raw=true "node_graph")



## Installation
This repository can be built using catkin build in the scenario_sim_ws directory.

### Prerequisites

```bash
sudo apt-get install python-catkin-tools
pip install -U rospkg
pip3 install commonroad-io
pip3 install rospkg numpy matplotlib

```

get depedencies -> execute in /ros_scenario_simulation

```bash
sudo rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

add ad-xolib as git submodule
```bash
git submodule add https://github.com/javedulu/ad-xolib.git src/open_scenario_helper/include/ad-xolib
```

### Dependencies
- ad-xolib
Install and build the ad-xolib from source. Additionally, install and build "emscripten" from source. This is needed to build "ad-xolib" or edit the CMakeLists of "ad-xolib" and set emscripten support OFF. At Line 13.

#### Build library ad-xolib
```bash
cd src/open_scenario_helper/include/ad-xolib 
git submodule update --init --recursive 
mkdir build
cd build
cmake .. -DBUILD_EMBED_TARGETS=OFF
make
```

### Build ROS Workspace 
If it fails, just try again.
```bash
catkin build
```

## Run
roslaunch sim_run run.launch 

## Info
### Citation
Citation Elements
Author: Armin Straller, Mathias Pechinger, Carsten Markgraf Title: ROS Scenario Simulation Publisher: University of Applied Sciences Augsburg Place of publication: Germany Date of publication: 2022 URL: https://github.com/arminstr/ros_scenario_simulation

Bibtex
```
@online{2022_ros_scenario_simulation, author = {Armin Straller and Mathias Pechinger and Hatem Darweesh and Carsten Markgraf}, title = {ROS Scenario Simulation}, publisher = {University of Applied Sciences Augsburg}, address = {Germany}, year = {2022}, url = {https://github.com/arminstr/ros_scenario_simulation} }
```

### References
[1]: Commonroad Scenario Format Definition https://commonroad.in.tum.de

[2]: OpenSCENARIO  Format Definition https://www.asam.net/standards/detail/openscenario/

