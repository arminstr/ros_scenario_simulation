# ROS Scenario Simulation
Simulation Framework for ROS based path planning algorithms. Uses Scenarios and is intended for CI use.

***This Framework is based on TUM Commonroad and Asam OpenScenario. Currently the licensing is not marked properly. Please notice this and do not copy or redistribute the code without marking it properly.***

## Prerequisites

```bash
sudo apt-get install python-catkin-tools
pip install -U rospkg
pip3 install commonroad-io
pip3 install rospkg numpy matplotlib

```

## installation

```bash
sudo rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

## building

```bash
catkin build
```

