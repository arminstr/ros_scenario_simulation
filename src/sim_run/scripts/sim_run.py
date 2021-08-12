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
simWaitLimit = 20

def behaviorCallback(behavior):
    global currentBehavior
    global bSimulationFinished
    global simWaitTimer
    currentBehavior = behavior.wpstate.event_state
    if 12 == currentBehavior:
        bSimulationFinished = True
        simWaitTimer = 0
        rospy.loginfo("***************** End Detected in Sim Master ********************")
        rospy.loginfo("event state: %d", currentBehavior)

def timeCallback(timeStep):
    global currentTimeStep
    currentTimeStep = int(timeStep.data)

def sim_run():
    global lastTimeStep
    global file_path
    global bSimulationFinished
    global simWaitTimer
    global simWaitLimit
    rospy.init_node('sim_run', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/op_current_behavior", Waypoint, behaviorCallback)
    rospy.Subscriber("/sim_timestep", Int32, timeCallback)

    file_path = rospy.get_param("/pathToCommonRoad")

    fileList = []
    for root, dirs, files in os.walk(file_path):
        for file in files:
            if file.endswith('.xml') and not file.startswith('solution'):
                    rospy.loginfo("%s", file)
                    fileList.append(file)
    
    while not rospy.is_shutdown():
        if bSimulationFinished:
            simWaitTimer += 1
        # if bSimulationFinished and simWaitTimer < simWaitLimit:
            # rospy.loginfo("*** Simulation Files in Queue: %d", len(fileList))
        if bSimulationFinished and len(fileList) > 0 and simWaitTimer >= simWaitLimit:
            strCommand = 'roslaunch sim_start sim.launch pathToCommonRoad:=' + file_path + '/' + fileList[0] + ' &'
            rospy.loginfo("launching: %s", strCommand)
            fileList.pop(0)
            bSimulationFinished = False
            os.system(strCommand)
        if bSimulationFinished and len(fileList) == 0:
            rospy.signal_shutdown()
            
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        sim_run()
    except rospy.ROSInterruptException:
        pass