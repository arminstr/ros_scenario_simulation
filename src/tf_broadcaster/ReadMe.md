# tf broadcaster
This ros node broadcasts the tf tree for the HIL simulation.

## Usage:
Sensor positions and tf tree dependencies can be setup insde the source code. The ros node subscribes to the current_pose message and publishes the tf tree.

## Publications: 
 * /tf [tf2_msgs/TFMessage]

## Subscriptions: 
 * /current_pose [geometry_msgs/PoseStamped]

