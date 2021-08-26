import rospy
import time
import os
import errno
from commonroad.scenario.trajectory import State
import json

def generateReport(stateList, reportPath, scenarioName):
    rospy.loginfo("Starting Report Generation...")
    rospy.loginfo("Report will be stored at: %s", reportPath)
    rospy.loginfo("Scenario name: %s", scenarioName)
    rospy.loginfo("Received State List with %d states.", len(stateList))

    scenarioResult = {
        "name": "",
        "pathLength": 0.0,
        "time": 0.0,
        "velocity":[],
        "acceleration": [],
        "accelerationCost": 0.0,
        "jerk": [],
        "jerkCost": 0.0,
        "steeringAngle": [],
        "steeringAngleCost": 0.0,
        "steeringRate": [],
        "steeringRateCost": 0.0,
        "yawRate": [],
        "yawRateCost": 0.0,
        "distanceToObstaclesCost": 0.0
    }
    for i, state in enumerate(stateList):
        scenarioResult["time"]  += 0.1
        scenarioResult["pathLength"]  += state.velocity * 0.1
        scenarioResult["velocity"].append(state.velocity)
        if i < len(stateList)-1:
            scenarioResult["acceleration"].append( (stateList[i+1].velocity - state.velocity) / 0.1)
            scenarioResult["accelerationCost"] += pow( (stateList[i+1].velocity - state.velocity) / 0.1, 2.0)
        scenarioResult["steeringAngle"].append(state.steering_angle)
        scenarioResult["steeringAngleCost"] += pow(state.steering_angle, 2.0)
        if i < len(stateList)-1:
            scenarioResult["steeringRate"].append((stateList[i+1].steering_angle - state.steering_angle) / 0.1 )
            scenarioResult["steeringRateCost"] += pow((stateList[i+1].steering_angle - state.steering_angle) / 0.1, 2.0)
        if i < len(stateList)-1:
            scenarioResult["yawRate"].append((stateList[i+1].orientation - state.orientation) / 0.1 )
            scenarioResult["yawRateCost"] += pow((stateList[i+1].orientation - state.orientation) / 0.1, 2.0)
    for i, accel in enumerate(scenarioResult["acceleration"]):
        if i < len(scenarioResult["acceleration"])-1:
            scenarioResult["jerk"].append( ( scenarioResult["acceleration"][i+1] - accel) / 0.1)
            scenarioResult["jerkCost"] += pow( ( scenarioResult["acceleration"][i+1] - accel) / 0.1, 2.0)

    # JSON output
    timestr = time.strftime("%Y%m%d-%H%M%S")
    json_filename = reportPath + "/" + timestr + "-" + scenarioName + ".json"

    json_object = json.dumps(scenarioResult, indent = 4)

    if not os.path.exists(os.path.dirname(json_filename)):
        try:
            os.makedirs(os.path.dirname(json_filename))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    with open(json_filename, "w") as outfile:
        outfile.write(json_object)