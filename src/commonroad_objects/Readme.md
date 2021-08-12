# Publishing Objects since April 2021

Add these lines to any launchfile to start the node.
```xml
<arg name="pathToCommonRoad" default="../commonroadFiles/UAS_Augsburg_CampusBrunnenlech_signalized_obstacle.xml"/>
<include file="$(find commonroad_objects)/launch/commonroad_objects.launch"/>
```
Built using catkin build.
