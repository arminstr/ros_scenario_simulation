import rospy
import time
import os
import errno
import json

def generateReport(stateList, objectsLists, centerLinesMarkerArray, reportPath, file_path):
    _, scenarioName = os.path.split(file_path)
    scenarioName, _ = os.path.splitext(scenarioName)
    rospy.loginfo("Starting Report Generation...")
    rospy.loginfo("Report will be stored at: %s", reportPath)
    rospy.loginfo("Scenario name: %s", scenarioName)
    rospy.loginfo("Received State List with %d states.", len(stateList))

    scenarioResult = {
        "name": "",
        "pathLength": 0.0,
        "time": 0.0,
        "dimension": [],
        "timeSteps": [],
        "position": [],
        "orientation": [],
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
        "distanceToObstaclesCost": 0.0,
        "obstacles": []
    }

    scenarioResult["dimension"].append(5.0)
    scenarioResult["dimension"].append(2.0)

    for i, state in enumerate(stateList):
        scenarioResult["time"]  += 0.1
        scenarioResult["timeSteps"].append(state.time_step * 0.1)
        scenarioResult["pathLength"]  += state.velocity * 0.1
        scenarioResult["position"].append(state.position.tolist())
        scenarioResult["orientation"].append(state.orientation)
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

    for object_list in objectsLists:
        scenarioResult["obstacles"].append(object_list)

    timestr = time.strftime("%Y%m%d-%H%M%S")
    
    # JSON output
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

    # JSON overview generation
    json_overview_filename = reportPath + "/scenario.json"

    
    if os.path.exists(json_overview_filename):
        f = open(json_overview_filename, "r")
        existing_scenario = json.load(f)
        f.close()

        existing_scenario["scenarios"].append(scenarioName)
        existing_scenario["scenario_json_paths"].append(json_filename)

        f = open(json_overview_filename, "w")
        f.write(json.dumps(existing_scenario, indent = 4))
        f.close()
    else:
        existing_scenario = {
        "scenarios": [],
        "scenario_json_paths": []
        }
        existing_scenario["scenarios"].append(scenarioName)
        existing_scenario["scenario_json_paths"].append(json_filename)

        f = open(json_overview_filename, "w")
        f.write(json.dumps(existing_scenario, indent = 4))
        f.close()

    mapStructure = {
        "markers": []
    }

    for marker in centerLinesMarkerArray.markers:
        markerSerializable = {
            "type": marker.type,
            "points": []
        }
        for point in marker.points:
            pointSerializable = [point.x, point.y]
            markerSerializable["points"].append(pointSerializable)
        mapStructure["markers"].append(markerSerializable)

    # Map storage
    map_filename = reportPath + "/map.js"
    map_f = open(map_filename, "w")
    map_f.write("scenario_map = ")
    map_f.write(json.dumps(mapStructure, indent = 4))
    map_f.close()