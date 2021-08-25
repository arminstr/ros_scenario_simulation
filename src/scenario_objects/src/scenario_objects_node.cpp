/*
  Armin Straller - University of Applied Sciences Augsburg
  Licensed according to this repository.
*/

#include <ros/ros.h>
#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "commonroad/CommonRoad.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>
#include <vector>

#include "openScenarioHelper.hpp"

geometry_msgs::PoseStamped getGoalPoseFromLaneletId(commonroad::CommonRoadData& cR);
void getAutowareObjectsFromCommonRoad(autoware_msgs::DetectedObjectArray& objArray, commonroad::CommonRoadData& cR, int tC);
geometry_msgs::Point32 rotate_point(float cx,float cy,float angle,geometry_msgs::Point32 p);
autoware_msgs::DetectedObject get_object_from_state(const commonroad::ObstacleInformation* obstacle, commonroad::ObstacleState* state, int id);

double noiseMargin = 0.1; // 0.1 Meters max

int main( int argc, char** argv )
{
  ros::init(argc, argv, "commonroad_objects");
  ros::NodeHandle n;
  ros::Publisher autoware_object_pub = n.advertise<autoware_msgs::DetectedObjectArray>("/simulated/objects", 10);
  ros::Rate r(10);
  ros::NodeHandle nh;

  // create commonroad data structure
  commonroad::CommonRoadData cr;
  bool bSuccess;

  // get path to scenario
  std::string pathToScenario;
  n.getParam("/sim_objects/pathToScenario", pathToScenario);

  // check scenario type and load
  if (openScenarioHelper::checkFileTypeXOSC(pathToScenario))
  {
    bSuccess = openScenarioHelper::Load(pathToScenario, cr);
  }
  else if (commonroad::checkFileTypeXML(pathToScenario))
  {
    bSuccess = commonroad::Load(pathToScenario, cr);
  }

  // checks if the file was loaded
  if(!bSuccess)
  {
    ROS_FATAL("Unable to load Scenario file!");
    exit(0);
  }

  // storage array for the objects
  autoware_msgs::DetectedObjectArray object_list;

  object_list.header.frame_id = "map";

  int timeCounter = -50;

  while (ros::ok())
  {
    getAutowareObjectsFromCommonRoad(object_list, cr, timeCounter);
    timeCounter ++;
    // if (object_list.objects.size() >0) std::cout << "OBJ_LIST: " << object_list.objects[0].pose.position.x << std::endl;
    autoware_object_pub.publish(object_list);
    object_list.objects.clear();
    
    ros::spinOnce();
    
    r.sleep();
  }
}

void getAutowareObjectsFromCommonRoad(autoware_msgs::DetectedObjectArray& objArray, commonroad::CommonRoadData& cR, int tC)
{
  bool bNoise = true;
  int id = 0;
  for(const commonroad::ObstacleInformation & obstacle : cR.obstacles)
  {
    commonroad::ObstacleState state;
    /* code for publishing autoware object */
    autoware_msgs::DetectedObject obj;
    
    if(obstacle.trajectory.size() > 0) 
    {
      // check if it's the first frame -> get initial state
      if(tC <= 0)
      {
        state = obstacle.initialState;
      }
      if(tC > 0)
      {
        if(tC < obstacle.trajectory.size())
        {
          state = obstacle.trajectory[tC];
        }
        if(tC >= obstacle.trajectory.size())
        {
          state = obstacle.trajectory[obstacle.trajectory.size()-1];
        }
      }
      obj = get_object_from_state(&obstacle, &state, id);      
      if (obj.pose.position.x >= 0.0001 || obj.pose.position.x <= -0.0001){
          objArray.objects.push_back(obj);
      }
    } 
    if(obstacle.trajectory.size() == 0) 
    {
      state = obstacle.initialState;
      obj = get_object_from_state(&obstacle, &state, id);
      if (obj.pose.position.x >= 0.0001 || obj.pose.position.x <= -0.0001){
          objArray.objects.push_back(obj);
      }
    }

    id++;
  }
}

autoware_msgs::DetectedObject get_object_from_state(const commonroad::ObstacleInformation* obstacle, commonroad::ObstacleState* state, int id)
{
  double noiseX = (double)(std::rand() % 200 - 100)/100.0;
  double noiseY = (double)(std::rand() % 200 - 100)/100.0;
  autoware_msgs::DetectedObject obj;
  obj.id = id;
  obj.score = 100;
  obj.valid = true;
  obj.pose_reliable = true;
  obj.velocity_reliable = true;
  obj.acceleration_reliable = false;
  obj.space_frame = "map";

  obj.pose.position.x = state->position.point.x + noiseMargin * noiseX;
  obj.pose.position.y = state->position.point.y + noiseMargin * noiseY;
  obj.pose.position.z = 0;

  obj.acceleration.linear.x = 0;
  obj.acceleration.linear.y = 0;
  obj.acceleration.linear.z = 0;

  obj.velocity.linear.x = 0;
  obj.velocity.linear.y = 0;
  obj.velocity.linear.z = 0;

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY( 0, 0, state->orientation.exact);  // Create this quaternion from roll/pitch/yaw (in radians)
  obj.pose.orientation.x = myQuaternion.x();
  obj.pose.orientation.y = myQuaternion.y();
  obj.pose.orientation.z = myQuaternion.z();
  obj.pose.orientation.w = myQuaternion.w();
  
  obj.dimensions.x = obstacle->shape.length;
  obj.dimensions.y = obstacle->shape.width;
  obj.dimensions.z = 2;

  obj.indicator_state = 0; // none
  obj.behavior_state = 0; 

  geometry_msgs::Point32 temp_point;
  float yaw = state->orientation.exact;
  obj.convex_hull.polygon.points.clear();
  
  // Hull boundingbox definition
  temp_point.x = obj.pose.position.x+(obj.dimensions.x/2);
  temp_point.y = obj.pose.position.y+(obj.dimensions.y/2);
  temp_point.z = 2;
  temp_point = rotate_point(obj.pose.position.x,obj.pose.position.y,yaw,temp_point);
  obj.convex_hull.polygon.points.push_back(temp_point);
  temp_point.x = obj.pose.position.x+(obj.dimensions.x/2);
  temp_point.y = obj.pose.position.y+(-obj.dimensions.y/2);
  temp_point.z = 2;
  temp_point = rotate_point(obj.pose.position.x,obj.pose.position.y,yaw,temp_point);
  obj.convex_hull.polygon.points.push_back(temp_point);
  temp_point.x = obj.pose.position.x+(-obj.dimensions.x/2);
  temp_point.y = obj.pose.position.y+(-obj.dimensions.y/2);
  temp_point.z = 2;
  temp_point = rotate_point(obj.pose.position.x,obj.pose.position.y,yaw,temp_point);
  obj.convex_hull.polygon.points.push_back(temp_point);
  temp_point.x = obj.pose.position.x+(-obj.dimensions.x/2);
  temp_point.y = obj.pose.position.y+(obj.dimensions.y/2);
  temp_point.z = 2;
  temp_point = rotate_point(obj.pose.position.x,obj.pose.position.y,yaw,temp_point);
  obj.convex_hull.polygon.points.push_back(temp_point);

  obj.convex_hull.header.frame_id = "map";

  return obj;
}

geometry_msgs::Point32 rotate_point(float cx,float cy,float angle,geometry_msgs::Point32 p)
{
  float s = sin(angle);
  float c = cos(angle);

  // translate point back to origin:
  p.x -= cx;
  p.y -= cy;

  // rotate point
  float xnew = p.x * c - p.y * s;
  float ynew = p.x * s + p.y * c;

  // translate point back:
  p.x = xnew + cx;
  p.y = ynew + cy;
  return p;
}