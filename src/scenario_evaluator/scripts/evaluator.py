#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import numpy as np
import os
import pathlib
from reportGenerator import generateReport
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from autoware_msgs.msg import VehicleCmd, DetectedObjectArray
from visualization_msgs.msg import MarkerArray
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory,State
from commonroad.prediction.prediction import TrajectoryPrediction
from vehiclemodels import parameters_vehicle3

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionWriter, Solution, PlanningProblemSolution, VehicleModel, VehicleType, CostFunction

# constant parameters
max_steering_angle_deg = 60.0 

# generate state list of the ego vehicle's trajectory
state_list = []
objects_lists = []
file_path = ''
report_path = ''

currentTimeStep = -50
lastTimeStep = 0
currentPosition = np.array([0,0])
currentVelocityX = 0.0
currentVelocityY = 0.0
currentYaw = 0.0
currentSteeringAngle = 0.0
globalGoal = 0
detectedObjectArray = 0
centerLinesMarkerArray = 0


def yaw_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into yaw
        yaw is rotation around z in radians (counterclockwise)
        """
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians

def goalCallback(goal):
    global globalGoal
    globalGoal = goal.pose

def calcDist(x1,y1,x2,y2):
    distance = math.sqrt(math.pow((x2 - x1),2) + math.pow((y2 - y1),2))
    return distance

def poseCallback(poseS):
    global currentPosition
    currentPosition = np.array([poseS.pose.position.x,poseS.pose.position.y])
    q = poseS.pose.orientation
    yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
    global currentYaw
    currentYaw = yaw
    # Check distance to goal
    if globalGoal == 0: return
    distance = calcDist(poseS.pose.position.x, poseS.pose.position.y, globalGoal.position.x, globalGoal.position.y)
    if distance < 5.0:
        simFinished(False)

def simFinished(stopTrigger):
    generateReport(state_list, objects_lists, centerLinesMarkerArray, report_path, file_path, stopTrigger)
    rospy.signal_shutdown("Evaluation Finsihed")

def velocityCallback(twistS):
    global currentVelocityX
    currentVelocityX = twistS.twist.linear.x
    global currentVelocityY
    currentVelocityY = twistS.twist.linear.y

def timeCallback(timeStep):
    global currentTimeStep
    currentTimeStep = int(timeStep.data)

def vehicleCmdCallback(cmd):
    global currentSteeringAngle
    currentSteeringAngle = max_steering_angle_deg * cmd.steer_cmd.steer / 100.0

def endCallback(endState):
    if "StopTrigger" in endState.data:
        simFinished(True)

def objectListCallback(msg):
    global detectedObjectArray
    detectedObjectArray = msg

def centerLinesCallback(msg):
    global centerLinesMarkerArray
    centerLinesMarkerArray = msg

def finish():
    global state_list
    global file_path

    # load the CommonRoad scenario
    scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
    # create the planned trajectory starting at time step 1
    ego_vehicle_trajectory = Trajectory(initial_time_step=1, state_list=state_list[1:])
    # create the prediction using the planned trajectory and the shape of the ego vehicle

    vehicle3 = parameters_vehicle3.parameters_vehicle3()
    ego_vehicle_shape = Rectangle(length=vehicle3.l, width=vehicle3.w)
    ego_vehicle_prediction = TrajectoryPrediction(trajectory=ego_vehicle_trajectory,
                                                shape=ego_vehicle_shape)

    # the ego vehicle can be visualized by converting it into a DynamicObstacle
    ego_vehicle_type = ObstacleType.CAR
    ego_vehicle = DynamicObstacle(obstacle_id=100, obstacle_type=ego_vehicle_type,
                                obstacle_shape=ego_vehicle_shape, initial_state=state_list[0],
                                prediction=ego_vehicle_prediction)

    # plot the scenario and the ego vehicle for each time step
    # plt.figure(figsize=(25, 10))
    # for i in range(0, len(state_list)):
    #     plt.clf()
    #     draw_object(scenario, draw_params={'time_begin': i})
    #     draw_object(ego_vehicle, draw_params={'time_begin': i, 'facecolor': 'g'})
    #     draw_object(planning_problem_set)
    #     plt.gca().set_aspect('equal')
    #     plt.pause(0.001)
    # plt.show()
    pps = PlanningProblemSolution(planning_problem_id=100,
                                vehicle_type=VehicleType.VW_VANAGON,
                                vehicle_model=VehicleModel.PM,
                                cost_function=CostFunction.WX1,
                                trajectory=ego_vehicle_prediction.trajectory)

    # define the object with necessary attributes.
    solution = Solution(scenario.scenario_id, [pps], computation_time=100)

    # write solution to a xml file
    rospy.loginfo("creating file")

    csw = CommonRoadSolutionWriter(solution)
    rospy.loginfo(csw)
    solution_output_path = pathlib.Path(file_path).parent.absolute()
    solution_output_path = os.path.join(solution_output_path, "solutions")
    rospy.loginfo(solution_output_path)
    csw.write_to_file(output_path=solution_output_path, overwrite=True)

    rospy.signal_shutdown("Evaluation Finsihed")

def evaluator():
    global lastTimeStep
    global file_path
    global report_path
    global detectedObjectArray
    rospy.init_node('evaluator', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/current_pose", PoseStamped, poseCallback)
    rospy.Subscriber("/current_velocity", TwistStamped, velocityCallback)
    rospy.Subscriber("/sim_timestep", Int32, timeCallback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalCallback)
    rospy.Subscriber("/op_controller_cmd", VehicleCmd, vehicleCmdCallback)
    rospy.Subscriber("/sim/end_state", String, endCallback)
    rospy.Subscriber("/simulated/objects", DetectedObjectArray, objectListCallback)
    rospy.Subscriber("/vector_map_center_lines_rviz", MarkerArray, centerLinesCallback)

    file_path = rospy.get_param("/pathToScenario")
    report_path = rospy.get_param("/pathForReport")
    rospy.loginfo("%s", file_path)
    
    while not rospy.is_shutdown():
        if lastTimeStep != currentTimeStep:
            if currentTimeStep >= 0:
                s = State(position=currentPosition, velocity=math.sqrt(math.pow(currentVelocityX,2) + math.pow(currentVelocityY,2)), orientation=currentYaw, steering_angle=currentSteeringAngle, time_step=currentTimeStep)
                state_list.append(s)

                objects = []
                for msg_obj in detectedObjectArray.objects:
                    obj_dict = {
                        "position": [msg_obj.pose.position.x, msg_obj.pose.position.y],
                        "dimension": [msg_obj.dimensions.x, msg_obj.dimensions.y],
                        "orientation": [msg_obj.pose.orientation.x, msg_obj.pose.orientation.y, msg_obj.pose.orientation.z, msg_obj.pose.orientation.w]
                    }
                    objects.append(obj_dict)
                objects_lists.append(objects)

            lastTimeStep = currentTimeStep
        
        rate.sleep()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        evaluator()
    except rospy.ROSInterruptException:
        pass