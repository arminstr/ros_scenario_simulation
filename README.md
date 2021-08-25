# ROS Scenario Simulation

This project is designed to test the openplanner algorithms, based on "common road" [1] and "openScenario"[2] files.

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

get depedencies -> execute in /scenario_sim_ws

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

### Build library ad-xolib

In CMakeLists of ad-xolib set emscrption support to OFF

```bash
cd src/open_scenario_helper/include/ad-xolib 
git submodule update --init --recursive 
mkdir build
cd build
cmake ..
make
```

### Build ROS Workspace with catkin build

### References
[1]: Commonroad Scenario Format Definition https://commonroad.in.tum.de

[2]: OpenSCENARIO  Format Definition https://www.asam.net/standards/detail/openscenario/