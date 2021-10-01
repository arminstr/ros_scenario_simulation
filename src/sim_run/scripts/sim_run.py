#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import os
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped


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
simWaitLimit = 10
globalGoal = 0

def calcDist(x1,y1,x2,y2):
    distance = math.sqrt(math.pow((x2 - x1),2) + math.pow((y2 - y1),2))
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
    # Check distance to goal
    distance = calcDist(currentPose.pose.position.x, currentPose.pose.position.y, globalGoal.position.x, globalGoal.position.y)
    if distance != 0 and distance < 5.0:
        simFinished()

def endCallback(endState):
    if endState.data == "StopTrigger":
        simFinished()

def sim_run():
    global lastTimeStep
    global file_path
    global bSimulationFinished
    global simWaitTimer
    global simWaitLimit
    rospy.init_node('sim_run', anonymous=True, disable_signals=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/current_pose", PoseStamped, poseCallback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalCallback)
    rospy.Subscriber("/sim_timestep", Int32, timeCallback)
    rospy.Subscriber("/sim/end_state", String, endCallback)

    file_path = rospy.get_param("/pathToScenario")
    report_path = rospy.get_param("/pathForReport")

    fileList = []
    for _, _, files in os.walk(file_path):
        for file in files:
            if file.endswith('.xml') and not file.startswith('solution'):
                fileList.append(file)
            if file.endswith('.xosc'):
                fileList.append(file)
    
    while not rospy.is_shutdown():
        if bSimulationFinished:
            simWaitTimer += 1
        if bSimulationFinished and len(fileList) > 0 and simWaitTimer >= simWaitLimit:
            topicname = os.popen("rosnode list | grep sim_core").readlines()
            if topicname == ['/sim_core\n']:
                print("killing simulation")
                os.system("rosnode kill sim_core")
                rospy.loginfo("*** Simulation Files in Queue: %d", len(fileList))
        if bSimulationFinished and len(fileList) > 0 and simWaitTimer >= simWaitLimit*2:
            strCommand = 'roslaunch sim_start sim.launch pathToScenario:=' + file_path + '/' + fileList[0] + ' pathForReport:=' + report_path +  ' &'
            rospy.loginfo("launching: %s", strCommand)
            fileList.pop(0)
            bSimulationFinished = False
            os.system(strCommand)
        if bSimulationFinished and len(fileList) == 0 :
            rospy.signal_shutdown("Simulation finished!")
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        sim_run()
    except rospy.ROSInterruptException:
        pass
