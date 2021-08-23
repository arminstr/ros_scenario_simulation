#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import numpy as np
import os
import pathlib
import matplotlib.pyplot as plt
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from visualization_msgs.msg import MarkerArray
from autoware_msgs.msg import Waypoint
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory,State
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.visualization.draw_dispatch_cr import draw_object
from vehiclemodels import parameters_vehicle3

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionWriter, Solution, PlanningProblemSolution, VehicleModel, VehicleType, CostFunction


# generate state list of the ego vehicle's trajectory
state_list = []
file_path = ''

currentTimeStep = -50
lastTimeStep = 0
currentBehavior = ''
currentVelocityX = 0.0
currentVelocityY = 0.0
currentYaw = 0.0
bSimulationFinished = True
simWaitTimer = 0
simWaitLimit = 5
globalGoal = 0



def calcDist(x1,y1,x2,y2):
     distance = math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1))
     return distance

def simFinished():
    global bSimulationFinished
    bSimulationFinished = True
    global simWaitTimer
    simWaitTimer = 0

def timeCallback(timeStep):
    global currentTimeStep
    currentTimeStep = int(timeStep.data)


def goalCallback(goal):
    global globalGoal
    globalGoal = goal.pose

def poseCallback(currentPose):
    if globalGoal == 0: return
    pose = currentPose.pose
    # Check distance to goal
    distance = calcDist(pose.position.x, pose.position.y,globalGoal.position.x, globalGoal.position.y)
    if distance != 0 and distance < 5.0:
        simFinished()

def sim_run():
    global lastTimeStep
    global file_path
    global bSimulationFinished
    global simWaitTimer
    global simWaitLimit
    rospy.init_node('sim_run', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/current_pose", PoseStamped, poseCallback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalCallback)
    rospy.Subscriber("/sim_timestep", Int32, timeCallback)

    file_path = rospy.get_param("/pathToScenario")

    fileList = []
    for root, dirs, files in os.walk(file_path):
        for file in files:
            if file.endswith('.xml') and not file.startswith('solution'):
                    rospy.loginfo("%s", file)
                    fileList.append(file)
            if file.endswith('.xosc'):
                    rospy.loginfo("%s", file)
                    fileList.append(file)
    
    while not rospy.is_shutdown():
        if bSimulationFinished:
            simWaitTimer += 1
        if bSimulationFinished and simWaitTimer < simWaitLimit:
            topicname = os.popen("rosnode list | grep sim_core").readlines()
            print(topicname)
            if topicname == ['/sim_core\n']:
                print("killing simulation")
                os.system("rosnode kill sim_core")
                rospy.loginfo("*** Simulation Files in Queue: %d", len(fileList))
        if bSimulationFinished and len(fileList) > 0 and simWaitTimer >= simWaitLimit:
            strCommand = 'roslaunch sim_start sim.launch pathToScenario:=' + file_path + '/' + fileList[0] + ' &'
            rospy.loginfo("launching: %s", strCommand)
            fileList.pop(0)
            bSimulationFinished = False
            os.system(strCommand)
            
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        sim_run()
    except rospy.ROSInterruptException:
        pass