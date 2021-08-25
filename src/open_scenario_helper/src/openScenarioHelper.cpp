#include "openScenarioHelper.hpp"

bool openScenarioHelper::Load(const std::string &path, commonroad::CommonRoadData &cr)
{
    bool bSuccess = false;
    // ToDo: rewrite success condition

    xosc xoscScenario;
    try
    {
        xoscScenario.load(path);
        xoscScenario.parse();
    }
    catch (...)
    {
        return bSuccess;
    }

    ScenarioObject ego;
    std::shared_ptr<Entities> entities;
    Storyboard story;
    int numberObjects = 0;

    story = *xoscScenario.m_OpenSCENARIO->m_OpenScenarioCategory->m_ScenarioDefinition->m_Storyboard;
    entities = xoscScenario.m_OpenSCENARIO->m_OpenScenarioCategory->m_ScenarioDefinition->m_Entities;

    std::string egoRef;
    std::vector<std::string> obstacleRefs;
    obstacleRefs.clear();

    //TODO: Add some parameter that can be used to set the ego and obstacle vehicle class tags. 
    // It should be possible to use one of the parameter fields of xosc

    // Retreive Ego Vehicle Scenario Object Name and Obstacle Names from OpenSCENARIO File
    for (int i = 0; i < entities->m_ScenarioObjects.size(); i++)
    {
        if(entities->m_ScenarioObjects[i]->m_EntityObject->m_Vehicle->name.m_string == "ego")
        {
            egoRef = entities->m_ScenarioObjects[i]->name.m_string;
        }
        if(entities->m_ScenarioObjects[i]->m_EntityObject->m_Vehicle->name.m_string == "obstacle")
        {
            obstacleRefs.push_back(entities->m_ScenarioObjects[i]->name.m_string);
        }
    }

    // Retreive Planning Problem start position
    for (int i = 0; i < story.m_Init->m_Actions->m_Privates.size(); i++)
    {
        if (story.m_Init->m_Actions->m_Privates[i]->entityRef.m_string == egoRef)
        {
            bSuccess = true;
            // Position
            cr.planningProblem.initialState.position.point.x = story.m_Init->m_Actions->m_Privates[i]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->x.m_double;
            cr.planningProblem.initialState.position.point.y = story.m_Init->m_Actions->m_Privates[i]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->y.m_double;
            // Heading
            cr.planningProblem.initialState.orientation.exact = story.m_Init->m_Actions->m_Privates[i]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->h.m_double;
            break;
        }
    }

    //Parse Ego Goal Position
    for (int i = 0; i < story.m_Storys.size(); i++)
    {
        for (int j = 0; j < story.m_Storys[i]->m_Acts.size(); j++)
        {
            if( story.m_Storys[i]->m_Acts[j]->m_ManeuverGroups[0]->m_Actors->m_EntityRefs[0]->entityRef.m_string == egoRef)
            {
                bSuccess = true;
                // Position
                cr.planningProblem.goalState.position.pos.point.x = story.m_Storys[i]->m_Acts[j]->m_ManeuverGroups[0]->m_Maneuvers[0]->m_Events[0]->m_Actions[0]->m_Action->m_PrivateAction->m_PrivateAction->m_RoutingAction->m_RoutingAction->m_AcquirePositionAction->m_Position->m_Position->m_WorldPosition->x.m_double;
                cr.planningProblem.goalState.position.pos.point.y = story.m_Storys[i]->m_Acts[j]->m_ManeuverGroups[0]->m_Maneuvers[0]->m_Events[0]->m_Actions[0]->m_Action->m_PrivateAction->m_PrivateAction->m_RoutingAction->m_RoutingAction->m_AcquirePositionAction->m_Position->m_Position->m_WorldPosition->y.m_double;
                // Heading
                cr.planningProblem.goalState.position.orientation.exact = story.m_Storys[i]->m_Acts[j]->m_ManeuverGroups[0]->m_Maneuvers[0]->m_Events[0]->m_Actions[0]->m_Action->m_PrivateAction->m_PrivateAction->m_RoutingAction->m_RoutingAction->m_AcquirePositionAction->m_Position->m_Position->m_WorldPosition->h.m_double;
                break;
            }
        }
    }

    cr.obstacles.clear();
    // Parse Obstacles
    for (int i = 0; i < story.m_Init->m_Actions->m_Privates.size(); i++)
    {
        for(int j = 0; j < obstacleRefs.size(); j++)
        {
            if (story.m_Init->m_Actions->m_Privates[i]->entityRef.m_string == obstacleRefs[j])
            {
                commonroad::ObstacleInformation temp_obstacle;
                bSuccess = true;
                // Position
                temp_obstacle.initialState.position.point.x = story.m_Init->m_Actions->m_Privates[i]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->x.m_double;
                temp_obstacle.initialState.position.point.y = story.m_Init->m_Actions->m_Privates[i]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->y.m_double;
                // Heading
                temp_obstacle.initialState.orientation.exact = story.m_Init->m_Actions->m_Privates[i]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->h.m_double;
                
                // Shape
                for (int k = 0; k < entities->m_ScenarioObjects.size(); k++){
                    if(entities->m_ScenarioObjects[k]->name.m_string == obstacleRefs[j])
                    {
                        temp_obstacle.shape.width = entities->m_ScenarioObjects[k]->m_EntityObject->m_Vehicle->m_BoundingBox->m_Dimensions->width.m_double;
                        temp_obstacle.shape.length = entities->m_ScenarioObjects[k]->m_EntityObject->m_Vehicle->m_BoundingBox->m_Dimensions->length.m_double;   
                        break;
                    }
                }

                // // now get the objects trajectory
                std::vector<std::shared_ptr<Vertex>> vertices;
                for (int l = 0; l < story.m_Storys.size(); l++)
                {
                    for (int m = 0; m < story.m_Storys[l]->m_Acts.size(); m++)
                    {
                        if (story.m_Storys[l]->m_Acts[m]->m_ManeuverGroups[0]->m_Actors->m_EntityRefs[0]->entityRef.m_string == obstacleRefs[j])
                        {
                            vertices = story.m_Storys[l]->m_Acts[m]->m_ManeuverGroups[0]->m_Maneuvers[0]->m_Events[0]->m_Actions[0]->m_Action->m_PrivateAction->m_PrivateAction->m_RoutingAction->m_RoutingAction->m_FollowTrajectoryAction->m_Trajectory->m_Shape->m_Shape->m_Polyline->m_Vertexs;
                            if(vertices.size() > 0)
                            {
                                // first calculate trajectory length
                                double trajectoryLength = 0.0;
                                for (int o = 0; o < vertices.size()-1; o ++)
                                {
                                    double segmentLength = std::sqrt(
                                        std::pow(vertices[o+1]->m_Position->m_Position->m_WorldPosition->x.m_double - 
                                        vertices[o]->m_Position->m_Position->m_WorldPosition->x.m_double, 2.0) + 
                                        std::pow(vertices[o+1]->m_Position->m_Position->m_WorldPosition->y.m_double - 
                                        vertices[o]->m_Position->m_Position->m_WorldPosition->y.m_double, 2.0));
                                    trajectoryLength += segmentLength;
                                }

                                // std::cout << ">>>> TRAJECTORY LENGTH: " << trajectoryLength << std::endl;

                                // then read obstacle speed
                                double speed = 0.0; // m/s
                                for (int p = 0; p < story.m_Init->m_Actions->m_Privates.size(); p++)
                                {
                                    if (story.m_Init->m_Actions->m_Privates[p]->entityRef.m_string == obstacleRefs[j])
                                    {
                                        speed = story.m_Init->m_Actions->m_Privates[p]->m_PrivateActions[1]->m_PrivateAction->m_LongitudinalAction->m_LongitudinalAction->m_SpeedAction->m_SpeedActionTarget->m_SpeedActionTarget->m_AbsoluteTargetSpeed->value.m_double;
                                    }
                                }

                                // calculate the step size at 10 Hz
                                double stepSize = speed / 10;
                                double numberOfSteps = trajectoryLength / stepSize;
                                double numberOfInterpolationStepsBetweenVertices = numberOfSteps / vertices.size();
                                std::cout << "numberOfInterpolationStepsBetweenVertices: " << numberOfInterpolationStepsBetweenVertices << std::endl;
                                for (int q = 0; q < vertices.size()-1; q ++)
                                {
                                    double dx = (vertices[q+1]->m_Position->m_Position->m_WorldPosition->x.m_double - vertices[q]->m_Position->m_Position->m_WorldPosition->x.m_double) / numberOfInterpolationStepsBetweenVertices;
                                    double dy = (vertices[q+1]->m_Position->m_Position->m_WorldPosition->y.m_double - vertices[q]->m_Position->m_Position->m_WorldPosition->y.m_double) / numberOfInterpolationStepsBetweenVertices;
                                    
                                    for(int r = 0; r < numberOfInterpolationStepsBetweenVertices; r ++)
                                    {
                                        commonroad::ObstacleState state;
                                        state.position.point.x = vertices[q]->m_Position->m_Position->m_WorldPosition->x.m_double + r * dx;
                                        state.position.point.y = vertices[q]->m_Position->m_Position->m_WorldPosition->y.m_double + r * dy;
                                        state.orientation.exact = vertices[q]->m_Position->m_Position->m_WorldPosition->h.m_double;
                                        temp_obstacle.trajectory.push_back(state);
                                    }
                                }
                            }
                        }
                    }
                }

                cr.obstacles.push_back(temp_obstacle); 
            }
        }
    }
    return bSuccess;
}

bool openScenarioHelper::checkFileTypeXOSC(std::string file){
    std::string fileEnding;
    try{
        for (int i = 4; i>0; i--){
            fileEnding.push_back(file[file.size()-i]);
        }
        if (fileEnding == "xosc") return true;

    } catch(...){
        std::cout << "ERROR IN: checkFileTypeXOSC" << std::endl;
    }
    return false;
}