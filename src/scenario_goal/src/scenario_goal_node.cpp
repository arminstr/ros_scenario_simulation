#include <ros/ros.h>
#include "commonroad/CommonRoad.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "opendrive/OpenDrive.hpp"

#include <cmath>
#include <vector>

#include "openScenarioHelper.hpp"

geometry_msgs::PoseStamped getGoalPoseFromCommonRoad(commonroad::CommonRoadData& cR, opendrive::OpenDriveData& oDR);
visualization_msgs::Marker getGoalMarkerFromStampedPose(geometry_msgs::PoseStamped gP);
bool bNoGoalFound = false;
bool bGoalPublished = false;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "commonroad_goal");
  ros::NodeHandle n;
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  ros::Publisher goal_marker_pub = n.advertise<visualization_msgs::Marker>("/sim_visu/goal_marker", 10);
  ros::Publisher state_pub = n.advertise<visualization_msgs::MarkerArray>("/behavior_state", 10);
  
  ros::Rate r(10);
  ros::NodeHandle nh;

  // create commonroad data structure
  commonroad::CommonRoadData cr;
  bool bSuccess;

  // get path to scenario
  std::string pathToScenario;
  n.getParam("/pathToScenario", pathToScenario);

  // check scenario type and load
  if (openScenarioHelper::checkFileTypeXOSC(pathToScenario))
  {
    // ToDo: Implement objects in scenario loader
    bSuccess = openScenarioHelper::Load(pathToScenario, cr);
  }
  else
  {
    bSuccess = commonroad::Load(pathToScenario, cr);
  }


  if(!bSuccess)
  {
    ROS_FATAL("Unable to load CommonRoad Benchmark file!");
    exit(0);
  }

  // get parameters from params.yaml
  std::string pathOD;
  nh.getParam("/pathToOpenDrive", pathOD);

  std::cout << "OPENDRIVE" << pathOD << std::endl;

  // create opendrive data structure and load info from OpenDrive File
  opendrive::OpenDriveData odr;
  bSuccess = opendrive::Load(pathOD, odr);

  // check if the file was loaded successfully
  if(!bSuccess)
  {
    ROS_FATAL("Unable to load Map file!");
    exit(0);
  }

  while (ros::ok())
  {

    if (!bGoalPublished){
      
      ros::Duration(1.0).sleep();

      geometry_msgs::PoseStamped goalPose;
      goalPose = getGoalPoseFromCommonRoad(cr, odr);
      goal_pub.publish(goalPose);
      
      visualization_msgs::Marker goalMarker;
      goalMarker = getGoalMarkerFromStampedPose(goalPose);
      goal_marker_pub.publish(goalMarker);

      bGoalPublished = true;
    }
    


    if(bNoGoalFound)
    {
      visualization_msgs::MarkerArray stateMarkerArray;
      visualization_msgs::Marker stateMarker;
      stateMarker.text = "End";
      stateMarkerArray.markers.push_back(stateMarker);
      state_pub.publish(stateMarkerArray);
      stateMarkerArray.markers.clear();

      ROS_INFO("*** No Goal Position found ***");
      ROS_FATAL("*** Stopping Simulation ***");
    }
    
    
    ros::spinOnce();
    
    r.sleep();
  }
}

geometry_msgs::PoseStamped getGoalPoseFromCommonRoad(commonroad::CommonRoadData& cR, opendrive::OpenDriveData& oDR)
{
  /* initialize goal Position geometry_msgs::PoseStamped data structure */
  geometry_msgs::PoseStamped gP;
  {
    gP.header.frame_id = "map";
    gP.header.stamp = ros::Time::now();
    gP.pose.position.x = 0;
    gP.pose.position.y = 0;
    gP.pose.position.z = 0;

    double qx= 0;
    double qy= 0;
    double qz= 0;
    double qw= 1;

    tf2::Quaternion q;
    q.setRPY( qx, qy, qz );
    gP.pose.orientation.x = 0;
    gP.pose.orientation.y = 0;
    gP.pose.orientation.z = 0;
    gP.pose.orientation.w = 1.0;
  }
  
  /* Get Goal Position based on carthesion coordinates */
  if(cR.planningProblem.goalState.position.pos.point.x)
  {
    gP.pose.position.x = cR.planningProblem.goalState.position.pos.point.x;
    gP.pose.position.y = cR.planningProblem.goalState.position.pos.point.y;

    tf2::Quaternion q;
    q.setRPY( 0, 0, cR.planningProblem.goalState.position.orientation.exact );
    gP.pose.orientation.x = q[0];
    gP.pose.orientation.y = q[1];
    gP.pose.orientation.z = q[2];
    gP.pose.orientation.w = q[3];

    return gP;
  }

  if(gP.pose.position.x == 0 && gP.pose.position.y == 0)
  {
    bNoGoalFound = true;
  }
  
}

visualization_msgs::Marker getGoalMarkerFromStampedPose(geometry_msgs::PoseStamped gP)
{
  visualization_msgs::Marker gM;
  gM.header.frame_id = "map";
  gM.header.stamp = ros::Time::now();
  gM.ns = "goal_visualize";
  gM.action = visualization_msgs::Marker::ADD;
  gM.id = 1;
  gM.type = visualization_msgs::Marker::ARROW;

  gM.pose.position = gP.pose.position;
  gM.pose.orientation = gP.pose.orientation;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  gM.scale.x = 1.0;
  gM.scale.y = 0.2;
  gM.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  gM.color.r = 0.0f;
  gM.color.g = 1.0f;
  gM.color.b = 0.0f;
  gM.color.a = 1.0;

  gM.lifetime = ros::Duration();
  
  return gM;
}