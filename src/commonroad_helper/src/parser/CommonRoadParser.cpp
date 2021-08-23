#include <commonroad/types.hpp>

#include "commonroad/parser/CommonRoadParser.hpp"

#include "pugi/pugixml.hpp"

namespace commonroad {
namespace parser {

bool CommonRoadParser::Parse(const char *xml,
                            commonroad::CommonRoadData &out_common_road_data,
                            XmlInputType inputType,
                            std::string *out_error)
{
  namespace crp = commonroad::parser;

  pugi::xml_document xmlDoc;
  pugi::xml_parse_result pugiParseResult;

  unsigned int const parse_options_ensure_no_doctype = pugi::parse_default & ~pugi::parse_doctype;
  switch (inputType)
  {
    case XmlInputType::FILE:
    {
      pugiParseResult = xmlDoc.load_file(xml, parse_options_ensure_no_doctype);
    }
    break;

    case XmlInputType::CONTENT:
    {
      pugiParseResult = xmlDoc.load_string(xml, parse_options_ensure_no_doctype);
    }
    break;

    default:
    {
      std::cout << "Did not find XMLInputType!" << std::endl;
      return false;
    }
    break;
  }

  if(pugiParseResult == false)
  {
    if (out_error != nullptr)
    {
      *out_error = pugiParseResult.description();
    }
    std::cout << pugiParseResult.description() << std::endl;
    return false;
  }

  for (pugi::xml_node obstacle = xmlDoc.child("commonRoad").child("obstacle"); obstacle; obstacle = obstacle.next_sibling("obstacle"))
  {
    ObstacleInformation commonRoadObstacleInformation;
    commonRoadObstacleInformation.role = (ObstacleRole)obstacle.child("role").text().as_uint();
    commonRoadObstacleInformation.type = (ObstacleType)obstacle.child("type").text().as_uint();
    
    commonRoadObstacleInformation.shape.length = obstacle.child("shape").child("rectangle").child("length").text().as_double();
    if (!commonRoadObstacleInformation.shape.length)
    {
      std::cout << "Shape is not a rectangle!" << std::endl;
      return false;
    }
    else
    {
      ObstacleShape shape;
      ObstacleGeometry geometry = ObstacleGeometry::RECTANGLE;
      commonRoadObstacleInformation.shape.geometry = geometry;
      commonRoadObstacleInformation.shape.width = obstacle.child("shape").child("rectangle").child("width").text().as_double();
    }

    commonRoadObstacleInformation.initialState.position.point.x = obstacle.child("initialState").child("position").child("point").child("x").text().as_double();
    commonRoadObstacleInformation.initialState.position.point.y = obstacle.child("initialState").child("position").child("point").child("y").text().as_double();

    commonRoadObstacleInformation.initialState.orientation.exact = obstacle.child("initialState").child("orientation").child("exact").text().as_double();
    commonRoadObstacleInformation.initialState.time.exact = obstacle.child("initialState").child("time").child("exact").text().as_int();
    commonRoadObstacleInformation.initialState.velocity.exact = obstacle.child("initialState").child("velocity").child("exact").text().as_double();
    commonRoadObstacleInformation.initialState.acceleration.exact = obstacle.child("initialState").child("acceleration").child("exact").text().as_double();

    for (pugi::xml_node state = obstacle.child("trajectory").child("state"); state; state = state.next_sibling("state"))
    {
      ObstacleState obstacleState;
      obstacleState.position.point.x = state.child("position").child("point").child("x").text().as_double();
      obstacleState.position.point.y = state.child("position").child("point").child("y").text().as_double();

      obstacleState.orientation.exact = state.child("orientation").child("exact").text().as_double();
      obstacleState.time.exact = state.child("time").child("exact").text().as_int();
      obstacleState.velocity.exact = state.child("velocity").child("exact").text().as_double();
      obstacleState.acceleration.exact = state.child("acceleration").child("exact").text().as_double();

      commonRoadObstacleInformation.trajectory.push_back(obstacleState);
    }
    out_common_road_data.obstacles.push_back(commonRoadObstacleInformation);
  }

  pugi::xml_node planningProblem = xmlDoc.child("commonRoad").child("planningProblem");

  out_common_road_data.planningProblem.id = planningProblem.attribute("id").as_int();
  out_common_road_data.planningProblem.initialState.position.point.x = planningProblem.child("initialState").child("position").child("point").child("x").text().as_double();
  out_common_road_data.planningProblem.initialState.position.point.y = planningProblem.child("initialState").child("position").child("point").child("y").text().as_double();
  out_common_road_data.planningProblem.initialState.orientation.exact = planningProblem.child("initialState").child("orientation").child("exact").text().as_double();
  out_common_road_data.planningProblem.initialState.time.exact = planningProblem.child("initialState").child("time").child("exact").text().as_int();
  out_common_road_data.planningProblem.initialState.velocity.exact = planningProblem.child("initialState").child("velocity").child("exact").text().as_double();
  out_common_road_data.planningProblem.initialState.yawRate.exact = planningProblem.child("initialState").child("yawRate").child("exact").text().as_double();
  out_common_road_data.planningProblem.initialState.slipAngle.exact = planningProblem.child("initialState").child("slipAngle").child("exact").text().as_double();

  if(planningProblem.child("goalState").child("position").child("lanelet").attribute("ref").as_int())
  {
    out_common_road_data.planningProblem.goalState.position.laneletRef = planningProblem.child("goalState").child("position").child("lanelet").attribute("ref").as_int();
  }

  if(planningProblem.child("goalState").child("position").child("rectangle").child("center").child("x").text().as_double())
  {
    out_common_road_data.planningProblem.goalState.position.pos.point.x = planningProblem.child("goalState").child("position").child("rectangle").child("center").child("x").text().as_double();
    out_common_road_data.planningProblem.goalState.position.pos.point.y = planningProblem.child("goalState").child("position").child("rectangle").child("center").child("y").text().as_double();
    out_common_road_data.planningProblem.goalState.position.orientation.exact = planningProblem.child("goalState").child("position").child("rectangle").child("orientation").text().as_double();
  }
  
  out_common_road_data.planningProblem.goalState.time.intervalStart = planningProblem.child("goalState").child("time").child("intervalStart").text().as_int();
  out_common_road_data.planningProblem.goalState.time.intervalEnd = planningProblem.child("goalState").child("time").child("intervalEnd").text().as_int();

  return true;
}

}
}