#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import numpy as np
import os
import pathlib
import matplotlib.pyplot as plt
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, TwistStamped
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
currentPosition = np.array([0,0])
currentVelocityX = 0.0
currentVelocityY = 0.0
currentYaw = 0.0

bSimulationFinished = False

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def poseCallback(poseS):
    global currentPosition
    currentPosition = np.array([poseS.pose.position.x,poseS.pose.position.y])
    q = poseS.pose.orientation
    roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
    global currentYaw
    currentYaw = yaw

def velocityCallback(twistS):
    global currentVelocityX
    currentVelocityX = twistS.twist.linear.x
    global currentVelocityY
    currentVelocityY = twistS.twist.linear.y

def timeCallback(timeStep):
    global currentTimeStep
    currentTimeStep = int(timeStep.data)

def behaviorCallback(behavior):
    global currentBehavior
    global bSimulationFinished
    global simWaitTimer
    currentBehavior = behavior.markers[0].text
    if "End" in currentBehavior:
        bSimulationFinished = True
        simWaitTimer = 0
        rospy.loginfo("***************** End Detected in Evaluator ********************")
        rospy.loginfo(" %s", currentBehavior)

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
    global bSimulationFinished
    rospy.init_node('evaluator', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/current_pose", PoseStamped, poseCallback)
    rospy.Subscriber("/current_velocity", TwistStamped, velocityCallback)
    rospy.Subscriber("/sim_timestep", Int32, timeCallback)
    rospy.Subscriber("/behavior_state", MarkerArray, behaviorCallback)

    file_path = rospy.get_param("/pathToCommonRoad")
    rospy.loginfo("%s", file_path)
    
    while not rospy.is_shutdown():
        if lastTimeStep == currentTimeStep:
            if currentTimeStep >= 0 and bSimulationFinished:
                bSimulationFinished = True
                finish()
        
        if lastTimeStep != currentTimeStep:
            if currentTimeStep >= 0:
                s = State(position=currentPosition, velocity=currentVelocityX, velocity_y=currentVelocityY, orientation=currentYaw, time_step=currentTimeStep)
                state_list.append(s)
                # rospy.loginfo("*** Summary of current state ***")
                # rospy.loginfo("%s", state_list[currentTimeStep])
            lastTimeStep = currentTimeStep
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        evaluator()
    except rospy.ROSInterruptException:
        pass