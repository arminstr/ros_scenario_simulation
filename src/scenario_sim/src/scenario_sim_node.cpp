
#include "commonroad/CommonRoad.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include "autoware_msgs/VehicleCmd.h"
#include <tf/tf.h>

#include <cmath>
#include <vector>

#include "openScenarioHelper.hpp"



geometry_msgs::PoseStamped getCurrentPoseVehicleModel(commonroad::CommonRoadData &cR, int timeStep);
visualization_msgs::Marker getMarkerFromStampedPose(geometry_msgs::PoseStamped gP);
void modelStep(commonroad::CommonRoadData &cR, int timeStep);
geometry_msgs::TwistStamped getCurrentVelocityVehicleModel(commonroad::CommonRoadData &cR, geometry_msgs::PoseStamped cP, geometry_msgs::PoseStamped lP, int timeStep);
void callbackVehicleCommand(const autoware_msgs::VehicleCmd &msg);
int getStopStateFromScenario(commonroad::CommonRoadData &cR);

double max_accel = 5;   // m / s^2
double max_brake = -10; // m / s^2
double max_steering_angle = 60.0 / 180.0 * M_PI;
autoware_msgs::VehicleCmd current_cmd;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::TwistStamped current_velocity_base_link;
geometry_msgs::TwistStamped current_velocity_world;

double dT = 0.1;
double lengthToRearAxle = 0.1;
double slipAngle = 0.0;
double steeringAngle = 0.0;
double steeringAngleRate = 0.0;
double vehicleLength = 2.8;

double lastSpeed = 0.0;

int maxTimeSteps = 600;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "commonroad_sim");
  ros::NodeHandle n;
  ros::Publisher current_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
  ros::Publisher current_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/current_velocity", 10);
  ros::Publisher current_timestep_pub = n.advertise<std_msgs::Int32>("/sim_timestep", 10);

  ros::Publisher current_pose_marker_pub = n.advertise<visualization_msgs::Marker>("/sim_visu/current_pose_marker", 10);
  ros::Publisher end_state_pub = n.advertise<std_msgs::String>("/sim/end_state", 10);
  ros::Subscriber sub_vehicle_cmd = n.subscribe("/op_controller_cmd", 10, callbackVehicleCommand);
  ros::Rate r(1 / dT);

  // create commonroad data structure
  commonroad::CommonRoadData cr;
  bool bSuccess;

  // get path to scenario
  std::string pathToScenario;
  n.getParam("/pathToScenario", pathToScenario);

  // check scenario type and load
  if (openScenarioHelper::checkFileTypeXOSC(pathToScenario))
  {
    bSuccess = openScenarioHelper::Load(pathToScenario, cr);
  }
  else
  {
    bSuccess = commonroad::Load(pathToScenario, cr);
  }

  if (!bSuccess)
  {
    ROS_FATAL("Unable to load scenario file!");
    exit(0);
  }

  int timeCounter = -50;
  geometry_msgs::PoseStamped lastPose;

  int endTimeStep = getStopStateFromScenario(cr);

  while (ros::ok())
  {
    modelStep(cr, timeCounter);

    current_pose_pub.publish(current_pose);
    current_velocity_pub.publish(current_velocity_base_link);

    std_msgs::Int32 timeMsg;
    timeMsg.data = timeCounter;
    current_timestep_pub.publish(timeMsg);

    visualization_msgs::Marker currentPoseMarker;
    currentPoseMarker = getMarkerFromStampedPose(current_pose);
    current_pose_marker_pub.publish(currentPoseMarker);

    if (timeCounter > endTimeStep)
    {
      std_msgs::String end_msg;
      end_msg.data = "StopTrigger";
      end_state_pub.publish(end_msg);
    }

    timeCounter++;
    ros::spinOnce();

    r.sleep();
  }
}

void callbackVehicleCommand(const autoware_msgs::VehicleCmd &msg)
{
  current_cmd = msg;
}

void modelStep(commonroad::CommonRoadData &cR, int timeStep)
{
  if (timeStep <= 0)
  {
    commonroad::ProblemState state = cR.planningProblem.initialState;

    current_pose.header.frame_id = "world";
    current_pose.header.stamp = ros::Time::now();

    current_pose.pose.position.x = state.position.point.x;
    current_pose.pose.position.y = state.position.point.y;
    current_pose.pose.position.z = 0.0;

    slipAngle = state.slipAngle.exact;
    steeringAngle = 0.0;
    steeringAngleRate = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state.orientation.exact);
    current_pose.pose.orientation.x = q[0];
    current_pose.pose.orientation.y = q[1];
    current_pose.pose.orientation.z = q[2];
    current_pose.pose.orientation.w = q[3];

    current_velocity_base_link.header.frame_id = "base_link";
    current_velocity_base_link.header.stamp = ros::Time::now();

    current_velocity_base_link.twist.linear.x = state.velocity.exact;
    current_velocity_base_link.twist.linear.y = 0.0;
    current_velocity_base_link.twist.linear.z = 0.0;

    current_velocity_base_link.twist.angular.x = 0;
    current_velocity_base_link.twist.angular.y = 0;
    current_velocity_base_link.twist.angular.z = state.yawRate.exact;
  }
  else if (timeStep > 0)
  {
    geometry_msgs::PoseStamped lP = current_pose;

    current_pose.header.frame_id = "world";
    current_pose.header.stamp = ros::Time::now();

    current_velocity_base_link.header.frame_id = "base_link";
    current_velocity_base_link.header.stamp = ros::Time::now();

    current_velocity_world.header.frame_id = "world";
    current_velocity_world.header.stamp = ros::Time::now();

    tf::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    // add current accel brake and steer values to the base link twist
    if(current_cmd.accel_cmd.accel > 100)
      current_cmd.accel_cmd.accel = 100
    if(current_cmd.accel_cmd.accel <= 0)
      current_cmd.accel_cmd.accel = 0
    if(current_cmd.brake_cmd.brake > 100)
      current_cmd.brake_cmd.brake = 100
    if(current_cmd.brake_cmd.brake <= 0)
      current_cmd.brake_cmd.brake = 0
    if(current_cmd.steer_cmd.steer > 100)
      current_cmd.steer_cmd.steer = 100
    if(current_cmd.steer_cmd.steer <= 0)
      current_cmd.steer_cmd.steer = 0
    current_velocity_base_link.twist.linear.x = current_velocity_base_link.twist.linear.x + (max_accel * (double)current_cmd.accel_cmd.accel / 100.0) * dT + (max_brake * (double)current_cmd.brake_cmd.brake / 100.0) * dT;

    current_velocity_base_link.twist.linear.y = (current_velocity_base_link.twist.linear.y - lastSpeed) / dT;
    lastSpeed = current_velocity_base_link.twist.linear.y;
    steeringAngle = max_steering_angle * (double)current_cmd.steer_cmd.steer / 100.0;

    if (current_velocity_base_link.twist.linear.x < 0.0)
    {
      current_velocity_base_link.twist.linear.x = 0.0;
    }

    slipAngle = atan2((lengthToRearAxle * tan(steeringAngle)), vehicleLength);

    current_velocity_base_link.twist.angular.z = (current_velocity_base_link.twist.linear.x * cos(slipAngle) * tan(steeringAngle)) / vehicleLength;

    yaw += current_velocity_base_link.twist.angular.z * dT;

    current_velocity_world.twist.linear.x = current_velocity_base_link.twist.linear.x * cos(yaw + slipAngle);
    current_velocity_world.twist.linear.y = current_velocity_base_link.twist.linear.x * sin(yaw + slipAngle);

    current_pose.pose.position.x += current_velocity_world.twist.linear.x * dT;
    current_pose.pose.position.y += current_velocity_world.twist.linear.y * dT;
    current_pose.pose.position.z = 0.0;

    q.setRPY(0.0, 0.0, yaw);
    current_pose.pose.orientation.x = q[0];
    current_pose.pose.orientation.y = q[1];
    current_pose.pose.orientation.z = q[2];
    current_pose.pose.orientation.w = q[3];
  }
}

int getStopStateFromScenario(commonroad::CommonRoadData &cR)
{
  int finalTimeStep = 0;
  if(cR.planningProblem.goalState.time.intervalEnd != 0)
  {
    finalTimeStep = cR.planningProblem.goalState.time.intervalEnd; 
  }
  else 
  {
    finalTimeStep = maxTimeSteps;
  }
  
  return finalTimeStep;
}

visualization_msgs::Marker getMarkerFromStampedPose(geometry_msgs::PoseStamped gP)
{
  visualization_msgs::Marker gM;
  gM.header.frame_id = "map";
  gM.header.stamp = ros::Time::now();
  gM.ns = "goal_visualize";
  gM.action = visualization_msgs::Marker::ADD;
  gM.id = 1;
  gM.type = visualization_msgs::Marker::CUBE;

  gM.pose.position = gP.pose.position;
  gM.pose.orientation = gP.pose.orientation;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  gM.scale.x = 5.0;
  gM.scale.y = 2.0;
  gM.scale.z = 2.0;

  // Set the color -- be sure to set alpha to something non-zero!
  gM.color.r = 0.0f;
  gM.color.g = 1.0f;
  gM.color.b = 0.0f;
  gM.color.a = 1.0;

  gM.lifetime = ros::Duration();

  return gM;
}
