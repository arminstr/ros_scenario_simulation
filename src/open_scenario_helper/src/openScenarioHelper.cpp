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
    Storyboard story;
    int numberObjects = 0;

    story = *xoscScenario.m_OpenSCENARIO->m_OpenScenarioCategory->m_ScenarioDefinition->m_Storyboard;

    // Parse Start Position

    for (int count = 0; count < story.m_Init->m_Actions->m_Privates.size(); count++)
    {
        if (story.m_Init->m_Actions->m_Privates[count]->entityRef.m_string == "Ego")
        {
            std::cout << "Found Ego" << std::endl;
            bSuccess = true;
            // Position
            cr.planningProblem.initialState.position.point.x = story.m_Init->m_Actions->m_Privates[count]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->x.m_double;
            cr.planningProblem.initialState.position.point.y = story.m_Init->m_Actions->m_Privates[count]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->y.m_double;
            // Heading
            cr.planningProblem.initialState.orientation.exact = story.m_Init->m_Actions->m_Privates[count]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->h.m_double;
            break;
        }
    }

    //Parse Goal Position
    for (int count = 0; count < story.m_Storys.size() ; count++)
    {
        if (story.m_Storys[count]->name.m_string == "drive-to-point")
        {
            std::cout << "Found Goal Point" << std::endl;
            bSuccess = true;
            // Position
            cr.planningProblem.goalState.position.pos.point.x = story.m_Storys[count]->m_Acts[0]->m_StopTrigger->m_ConditionGroups[0]->m_Conditions[0]->m_Condition->m_ByEntityCondition->m_EntityCondition->m_EntityCondition->m_ReachPositionCondition->m_Position->m_Position->m_WorldPosition->x.m_double;
            cr.planningProblem.goalState.position.pos.point.y = story.m_Storys[count]->m_Acts[0]->m_StopTrigger->m_ConditionGroups[0]->m_Conditions[0]->m_Condition->m_ByEntityCondition->m_EntityCondition->m_EntityCondition->m_ReachPositionCondition->m_Position->m_Position->m_WorldPosition->y.m_double;
            // Heading
            cr.planningProblem.goalState.position.orientation.exact = story.m_Storys[count]->m_Acts[0]->m_StopTrigger->m_ConditionGroups[0]->m_Conditions[0]->m_Condition->m_ByEntityCondition->m_EntityCondition->m_EntityCondition->m_ReachPositionCondition->m_Position->m_Position->m_WorldPosition->h.m_double;
            break;
        }
    }

    cr.obstacles.clear();
    int object_cnt = 0;

    // Parse Obstacles
    for (int count = 0; count < story.m_Init->m_Actions->m_Privates.size(); count++)
    {
        // std::cout << "loop start " << count << std::endl;
        if (story.m_Init->m_Actions->m_Privates[count]->entityRef.m_string == "Obstacle")
        {
            object_cnt++;
            commonroad::ObstacleInformation temp_obstacle;
            std::cout << "Found Obstacle " << object_cnt << std::endl;
            bSuccess = true;
            // Position
            temp_obstacle.initialState.position.point.x = story.m_Init->m_Actions->m_Privates[count]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->x.m_double;
            temp_obstacle.initialState.position.point.y = story.m_Init->m_Actions->m_Privates[count]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->y.m_double;
            // Heading
            temp_obstacle.initialState.orientation.exact = story.m_Init->m_Actions->m_Privates[count]->m_PrivateActions[0]->m_PrivateAction->m_TeleportAction->m_Position->m_Position->m_WorldPosition->h.m_double;
            
            // Shape
            temp_obstacle.shape.width = 1.5;
            temp_obstacle.shape.length = 2.5;            

            cr.obstacles.push_back(temp_obstacle); 
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