import rospy
import time
import os
import errno
import json
import math
from shapely import geometry

# decreasing this factor increases the influence of close obstacles
additionalObstacleWeight = 0.1 

# weights for individual cost metrics
timeWeight = 1.0
collisionWeight = 10000.0
stopTriggerWeight = 1000.0
pathLengthWeight = 1.0
accelerationWeight = 1.0
jerkWeight = 0.1
steeringAngleWeight = 1.0
steeringRateWeight = 1.0
yawRateWeight = 1.0
distanceToObstaclesWeight = 10.0
distanceToCenterLinesWeight = 1.0

def calcDistance(p0, p1):
    return math.sqrt(math.pow(p0[0]-p1[0], 2.0)+math.pow(p0[1]-p1[1], 2.0))
    
# intersection of two polygons from accepted answer of
# https://stackoverflow.com/questions/10962379/how-to-check-intersection-between-2-rotated-rectangles
def checkpolygonIntersection(polygonA, polygonB):
    return geometry.Polygon(polygonA).intersects(geometry.Polygon(polygonB))
    # polygons = [polygonA, polygonB]
    # for polygon in polygons:
    #     for i1, p1 in enumerate(polygon):
    #         i2 = (i1 + 1) % len(polygon)
    #         p2 = polygon[i2]

    #         normal = [p2[1] - p1[1], p1[0] - p2[0]]
    #         minA = None
    #         maxA = None
    #         for point in polygonA:
    #             projected = normal[0] * point[0] + normal[1] * point[1]
    #             if (minA == None or projected < minA):
    #                 minA = projected
    #             if (maxA == None or projected > maxA):
    #                 maxA = projected

    #         minB = None
    #         maxB = None
    #         for point in polygonB:
    #             projected = normal[0] * point[0] + normal[1] * point[1]
    #             if (minB == None or projected < minB):
    #                 minB = projected
    #             if (maxB == None or projected > maxB):
    #                 maxB = projected

    #         if (maxA < minB or maxB < minA):
    #             return False
    # return True


def generateReport(stateList, objectsLists, centerLinesMarkerArray, reportPath, file_path, stopTrigger):
    _, scenarioName = os.path.split(file_path)
    scenarioName, _ = os.path.splitext(scenarioName)
    rospy.loginfo("Starting Report Generation...")
    rospy.loginfo("Report will be stored at: %s", reportPath)
    rospy.loginfo("Scenario name: %s", scenarioName)
    rospy.loginfo("Received State List with %d states.", len(stateList))

    scenarioResult = {
        "name": "",
        "pathLength": 0.0,
        "pathLengthWeighted": 0.0,
        "time": 0.0,
        "timeWeighted": 0.0,
        "collision": 0.0,
        "collisionWeighted": 0.0,
        "stopTrigger": 0.0,
        "stopTriggerWeighted": 0.0,
        "dimension": [],
        "timeSteps": [],
        "position": [],
        "orientation": [],
        "velocity":[],
        "acceleration": [],
        "accelerationCost": 0.0,
        "accelerationCostWeighted": 0.0,
        "jerk": [],
        "jerkCost": 0.0,
        "jerkCostWeighted": 0.0,
        "steeringAngle": [],
        "steeringAngleCost": 0.0,
        "steeringAngleCostWeighted": 0.0,
        "steeringRate": [],
        "steeringRateCost": 0.0,
        "steeringRateCostWeighted": 0.0,
        "yawRate": [],
        "yawRateCost": 0.0,
        "yawRateCostWeighted": 0.0,
        "distanceToObstaclesCost": 0.0,
        "distanceToObstaclesCostWeighted": 0.0,
        "distanceToCenterLinesCost": 0.0,
        "distanceToCenterLinesCostWeighted": 0.0,
        "obstacles": [],
        "costSum": 0.0,
        "costSumWeighted": 0.0
    }

    scenarioResult["dimension"].append(5.0)
    scenarioResult["dimension"].append(2.0)

    if stopTrigger:
        scenarioResult["stopTrigger"] = 1.0
    else:
        scenarioResult["stopTrigger"] = 0.0

    # iterate trough states and append the information to the dictionary for storage
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
        scenarioResult["steeringAngleCost"] += pow(state.steering_angle/180.0 * math.pi, 2.0)
        if i < len(stateList)-1:
            scenarioResult["steeringRate"].append((stateList[i+1].steering_angle - state.steering_angle) / 0.1 )
            scenarioResult["steeringRateCost"] += pow(((stateList[i+1].steering_angle - state.steering_angle)/180.0 * math.pi) / 0.1, 2.0)
        if i < len(stateList)-1:
            scenarioResult["yawRate"].append((stateList[i+1].orientation - state.orientation) / 0.1 )
            scenarioResult["yawRateCost"] += pow((stateList[i+1].orientation - state.orientation) / 0.1, 2.0)
    
    # iterate one element less due to integration
    for i, accel in enumerate(scenarioResult["acceleration"]):
        if i < len(scenarioResult["acceleration"])-1:
            scenarioResult["jerk"].append( ( scenarioResult["acceleration"][i+1] - accel) / 0.1)
            scenarioResult["jerkCost"] += pow( ( scenarioResult["acceleration"][i+1] - accel) / 0.1, 2.0)

    # apped the objects in the object list
    for object_list in objectsLists:
        scenarioResult["obstacles"].append(object_list)

    # calculating the distance to obstacles cost
    for position_index, position in enumerate(scenarioResult["position"]):
        obstacleDistances = []
        for obstacle in scenarioResult["obstacles"][position_index]:
            distance = math.sqrt(pow(position[0]-obstacle["position"][0], 2.0) + pow(position[1]-obstacle["position"][1], 2.0))
            obstacleDistances.append(pow(math.e, -1 * additionalObstacleWeight * distance))
        maxDistance = 0.0
        if len(obstacleDistances) > 0:
            maxDistance = max(obstacleDistances)
            if type(maxDistance) is not float:
                maxDistance = maxDistance[0]
        scenarioResult["distanceToObstaclesCost"] += maxDistance

    # checking for collision
    for position_index, position in enumerate(scenarioResult["position"]):
        # delta x and y in length axis
        ego_dl = [
            math.cos(scenarioResult["orientation"][position_index])*scenarioResult["dimension"][0]/2, 
            math.sin(scenarioResult["orientation"][position_index])*scenarioResult["dimension"][0]/2
            ]
        # delta x and y in with axis
        ego_dw = [
            math.cos(scenarioResult["orientation"][position_index] - math.pi/2)*scenarioResult["dimension"][1]/2, 
            math.sin(scenarioResult["orientation"][position_index] - math.pi/2)*scenarioResult["dimension"][1]/2
            ]
        boundingBoxEgo = [
            [position[0] + ego_dl[0] + ego_dw[0], position[1] + ego_dl[1] + ego_dw[1]],
            [position[0] + ego_dl[0] - ego_dw[0], position[1] + ego_dl[1] - ego_dw[1]],
            [position[0] - ego_dl[0] + ego_dw[0], position[1] - ego_dl[1] + ego_dw[1]],
            [position[0] - ego_dl[0] - ego_dw[0], position[1] - ego_dl[1] - ego_dw[1]]
        ]

        for obstacle in scenarioResult["obstacles"][position_index]:
            # delta x and y in length axis
            obstacle_dl = [
                math.cos(obstacle["orientation"])*obstacle["dimension"][0]/2, 
                math.sin(obstacle["orientation"])*obstacle["dimension"][0]/2
                ]
            # delta x and y in with axis
            obstacle_dw = [
                math.cos(obstacle["orientation"] - math.pi/2)*obstacle["dimension"][1]/2, 
                math.sin(obstacle["orientation"] - math.pi/2)*obstacle["dimension"][1]/2
                ]
            boundingBoxObstacle = [
                [obstacle["position"][0] + obstacle_dl[0] + obstacle_dw[0], obstacle["position"][1] + obstacle_dl[1] + obstacle_dw[1]],
                [obstacle["position"][0] + obstacle_dl[0] - obstacle_dw[0], obstacle["position"][1] + obstacle_dl[1] - obstacle_dw[1]],
                [obstacle["position"][0] - obstacle_dl[0] + obstacle_dw[0], obstacle["position"][1] - obstacle_dl[1] + obstacle_dw[1]],
                [obstacle["position"][0] - obstacle_dl[0] - obstacle_dw[0], obstacle["position"][1] - obstacle_dl[1] - obstacle_dw[1]]
            ]
            
            if checkpolygonIntersection(boundingBoxObstacle, boundingBoxEgo):
                scenarioResult["collision"] = 1.0

    mapStructure = {
        "markers": []
    }

    for marker in centerLinesMarkerArray.markers:
        if marker.ns == "road_network_vector_map" and marker.type == 4:
            markerSerializable = {
                "points": []
            }
            for point in marker.points:
                pointSerializable = [point.x, point.y]
                markerSerializable["points"].append(pointSerializable)
            mapStructure["markers"].append(markerSerializable)


    # calculating the distance to center Lanes
    for position in scenarioResult["position"]:
        distances = []
        for centerLineMarker in mapStructure["markers"]:
            for point in centerLineMarker["points"]:
                #calculate Distance between Ego position and centerLinePoint
                distances.append(calcDistance(position, point))
        if len(distances) > 1:
            minDistance = min(distances)
            if type(minDistance) is not float:
                minDistance = minDistance[0]
        else:
            minDistance = 1000.0
        scenarioResult["distanceToCenterLinesCost"] += minDistance

    scenarioResult["costSum"] = scenarioResult["time"] + \
        scenarioResult["collision"] + \
        scenarioResult["stopTrigger"] + \
        scenarioResult["pathLength"] + \
        scenarioResult["accelerationCost"] + \
        scenarioResult["jerkCost"] + \
        scenarioResult["steeringAngleCost"] + \
        scenarioResult["steeringRateCost"] + \
        scenarioResult["yawRateCost"] + \
        scenarioResult["distanceToCenterLinesCost"] + \
        scenarioResult["distanceToObstaclesCost"]

    scenarioResult["timeWeighted"] = scenarioResult["time"] * timeWeight
    scenarioResult["collisionWeighted"] = scenarioResult["collision"] * collisionWeight
    scenarioResult["stopTriggerWeighted"] = scenarioResult["stopTrigger"] * stopTriggerWeight
    scenarioResult["pathLengthWeighted"] = scenarioResult["pathLength"] * pathLengthWeight
    scenarioResult["accelerationCostWeighted"] = scenarioResult["accelerationCost"] * accelerationWeight
    scenarioResult["jerkCostWeighted"] = scenarioResult["jerkCost"] * jerkWeight
    scenarioResult["steeringAngleCostWeighted"] = scenarioResult["steeringAngleCost"] * steeringAngleWeight
    scenarioResult["steeringRateCostWeighted"] = scenarioResult["steeringRateCost"] * steeringRateWeight
    scenarioResult["yawRateCostWeighted"] = scenarioResult["yawRateCost"] * yawRateWeight
    scenarioResult["distanceToCenterLinesCostWeighted"] = scenarioResult["distanceToCenterLinesCost"] * distanceToCenterLinesWeight
    scenarioResult["distanceToObstaclesCostWeighted"] = scenarioResult["distanceToObstaclesCost"] * distanceToObstaclesWeight
    
    scenarioResult["costSumWeighted"] = scenarioResult["timeWeighted"] + \
        scenarioResult["collisionWeighted"] + \
        scenarioResult["stopTriggerWeighted"] + \
        scenarioResult["pathLengthWeighted"] + \
        scenarioResult["accelerationCostWeighted"] + \
        scenarioResult["jerkCostWeighted"] + \
        scenarioResult["steeringAngleCostWeighted"] + \
        scenarioResult["steeringRateCostWeighted"] + \
        scenarioResult["yawRateCostWeighted"] + \
        scenarioResult["distanceToCenterLinesCostWeighted"] + \
        scenarioResult["distanceToObstaclesCostWeighted"]

    
    # FILE OUTPUT SECTION 
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
    json_overview_filename = reportPath + "/scenarios.json"
    existing_scenario = {
        "scenario_json_paths": []
    }

    json_filename = timestr + "-" + scenarioName + ".json"
    if os.path.exists(json_overview_filename):
        f = open(json_overview_filename, "r")
        existing_scenario = json.load(f)
        f.close()

        existing_scenario["scenario_json_paths"].append(json_filename)

        f = open(json_overview_filename, "w")
        f.write(json.dumps(existing_scenario, indent = 4))
        f.close()
    else:
        existing_scenario["scenario_json_paths"].append(json_filename)

        f = open(json_overview_filename, "w")
        f.write(json.dumps(existing_scenario, indent = 4))
        f.close()

    # Map storage
    map_filename = reportPath + "/map.js"
    map_f = open(map_filename, "w")
    map_f.write("scenario_map = ")
    map_f.write(json.dumps(mapStructure, indent = 4))
    map_f.close()