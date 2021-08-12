/* Copyright (C) University of Applied Sciences Augsburg - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Contributers: 
 * Mathias Pechinger <mathias.pechinger@hs-augsburg.de>, September 2020
 */

#include "tf_broadcaster_node.hpp"

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"tfBroadcaster");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    tfBroadcaster tfBroadcaster(nh,priv_nh);
    ros::Rate ros_rate(100);

    while(ros::ok())
    {
        tfBroadcaster.staticTransforms();
        ros::spinOnce();
        ros_rate.sleep();
        
    }
    return 0;
}

tfBroadcaster::tfBroadcaster(ros::NodeHandle nh, ros::NodeHandle Private_nh)
{
    puseSub = nh.subscribe("/current_pose",1,&tfBroadcaster::poseCallback,this,ros::TransportHints().tcpNoDelay(true));
}

void tfBroadcaster::staticTransforms(void){

    //base_link
    static tf::TransformBroadcaster tf_boradcaster;
    tf::Transform tf;
    
    tf.setOrigin( tf::Vector3(0, 0.284, -0.38) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    tf.setRotation(q);
    tf_boradcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "gnss_link", "base_rot"));

    //sensor_mount top
    tf.setOrigin( tf::Vector3(0.5, 0 ,1.6) );
    q.setRPY(0, 0, 0);
    tf.setRotation(q);
    tf_boradcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "base_link", "sensor_mount"));

    //velodyne
    tf.setOrigin( tf::Vector3(0.0, 0 ,0) );
    q.setRPY(0, 0, 0);
    tf.setRotation(q);
    tf_boradcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "sensor_mount", "velodyne"));

    //camera
    tf.setOrigin( tf::Vector3(1, 0 ,0) );
    q.setRPY(0, 0, 0);
    tf.setRotation(q);
    tf_boradcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "sensor_mount", "camera"));

    //world
    tf.setOrigin( tf::Vector3(0.0, 0 ,0) );
    q.setRPY(0, 0, 0);
    tf.setRotation(q);
    tf_boradcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world", "map"));

}

void tfBroadcaster::poseCallback(const geometry_msgs::PoseStampedConstPtr &data){
    static tf::TransformBroadcaster tf_boradcaster;
    
    tf::Transform tf;
    tf.setOrigin( tf::Vector3(data->pose.position.x, data->pose.position.y, 0.0) );

    double qx= data->pose.orientation.x;
    double qy= data->pose.orientation.y;
    double qz= data->pose.orientation.z;
    double qw= data->pose.orientation.w;

    tf::Quaternion q(qx, qy, qz, qw);

    tf.setRotation(q);
    tf_boradcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "map", "base_link"));

}