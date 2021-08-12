/* Copyright (C) University of Applied Sciences Augsburg - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Contributers: 
 * Mathias Pechinger <mathias.pechinger@hs-augsburg.de>, September 2020
 */

#ifndef UDP_ROS_UDP_ROS_NODE_H
#define UDP_ROS_UDP_ROS_NODE_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>


class tfBroadcaster
{
private:
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &data);
    ros::Subscriber puseSub;
    

public:
    tfBroadcaster(ros::NodeHandle nh, ros::NodeHandle Private_nh);
    void staticTransforms(void);
    

};




#endif //UDP_ROS_UDP_ROS_NODE_H