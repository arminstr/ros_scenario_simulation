#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/Waypoint.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>


std::uint8_t behaviorState;
double currentVelocityX;

void stop_simulation(void);
std::string get_pid(std::string str_cmd);
void run_cmd(std::string str_cmd);
void callbackBehaviorState(const autoware_msgs::Waypoint::ConstPtr& msg);
void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);

ros::Subscriber sub_behavior_state, sub_current_velocity;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "sim_start");
    ros::NodeHandle nh;

    ros::Rate ros_rate(10);

    sub_behavior_state = nh.subscribe("/op_current_behavior", 1, callbackBehaviorState);
    sub_current_velocity = nh.subscribe("/current_velocity", 1, callbackCurrentVelocity);
    
    // get parameters from launchfile
    std::string pathCR;
    nh.getParam("/pathToScenario", pathCR);

    std::string pathReport;
    nh.getParam("/pathForReport", pathReport);

    // std::string fileXODR;
    // fileXODR = fileCR.substr(0, fileCR.find_last_of(".")) + ".xodr";
    
    int timeCounter = 0;

    while (ros::ok())
    {
        std::string str_cmd;
        const char *command;
        
        switch(timeCounter)
        {
            case 0:
                // str_cmd = "cd " + pathCR + "&& python3 lanelet2opendrive.py ./commonRoadScenarios/" + fileCR + " --force-overwrite &";  
                // // Convert string to const char * as system requires 
                // command = str_cmd.c_str(); 
                // ROS_INFO("*** Starting Scenario Conversion ***");
                // system(command);
                break;

            case 20:
                str_cmd = "roslaunch scenario_sim scenario_sim.launch pathToScenario:=" + pathCR + " pathForReport:=" + pathReport + " &";  
                // Convert string to const char * as system requires 
                command = str_cmd.c_str(); 
                ROS_INFO("*** Starting Simulation ***");
                ROS_INFO(command);
                system(command);
                break;
        }

        timeCounter ++;
        ros::spinOnce();
        
        ros_rate.sleep();
    }
    return 0;
}

void callbackBehaviorState(const autoware_msgs::Waypoint::ConstPtr& msg)
{   
    behaviorState = msg->wpstate.event_state;
}


void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    currentVelocityX = msg->twist.linear.x;
}