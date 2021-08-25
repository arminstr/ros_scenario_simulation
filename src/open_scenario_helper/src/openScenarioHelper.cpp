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
    for (int i = 0; i < entities->m_ScenarioObjects.size(); i++){
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
    // TODO: Fix Open Scenario compatibility issues
    for (int i = 0; i < story.m_Storys.size(); i++)
    {
        for (int j = 0; j < story.m_Storys[i]->m_Acts.size(); j++)
        {
            if( story.m_Storys[i]->m_Acts[j]->m_StopTrigger->m_ConditionGroups[0]->m_Conditions[0]->m_Condition->m_ByEntityCondition->m_TriggeringEntities->m_EntityRefs[0]->entityRef.m_string == egoRef)
            {
                std::cout << "Found Ego Goal Point" << std::endl;
                bSuccess = true;
                // Position
                cr.planningProblem.goalState.position.pos.point.x = story.m_Storys[i]->m_Acts[j]->m_StopTrigger->m_ConditionGroups[0]->m_Conditions[0]->m_Condition->m_ByEntityCondition->m_EntityCondition->m_EntityCondition->m_ReachPositionCondition->m_Position->m_Position->m_WorldPosition->x.m_double;
                cr.planningProblem.goalState.position.pos.point.y = story.m_Storys[i]->m_Acts[j]->m_StopTrigger->m_ConditionGroups[0]->m_Conditions[0]->m_Condition->m_ByEntityCondition->m_EntityCondition->m_EntityCondition->m_ReachPositionCondition->m_Position->m_Position->m_WorldPosition->y.m_double;
                // Heading
                cr.planningProblem.goalState.position.orientation.exact = story.m_Storys[i]->m_Acts[j]->m_StopTrigger->m_ConditionGroups[0]->m_Conditions[0]->m_Condition->m_ByEntityCondition->m_EntityCondition->m_EntityCondition->m_ReachPositionCondition->m_Position->m_Position->m_WorldPosition->h.m_double;
                break;

            }
        }
    }

    cr.obstacles.clear();
    int object_cnt = 0;

    // Parse Obstacles
    for (int i = 0; i < story.m_Init->m_Actions->m_Privates.size(); i++)
    {
        // std::cout << "loop start " << count << std::endl;
        for(int j = 0; j < obstacleRefs.size(); j++)
        {
            if (story.m_Init->m_Actions->m_Privates[i]->entityRef.m_string == obstacleRefs[j])
            {
                object_cnt++;
                commonroad::ObstacleInformation temp_obstacle;
                std::cout << "Found Obstacle " << object_cnt << std::endl;
                bSuccess = true;
                // Position
                temp_obstacle.initialState.position.point.x = story.m_Init->m_Actions->m_Privates[i]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->x.m_double;
                temp_obstacle.initialState.position.point.y = story.m_Init->m_Actions->m_Privates[i]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->y.m_double;
                // Heading
                temp_obstacle.initialState.orientation.exact = story.m_Init->m_Actions->m_Privates[i]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->h.m_double;
                
                // Shape
                temp_obstacle.shape.width = 1.5;
                temp_obstacle.shape.length = 2.5;      

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