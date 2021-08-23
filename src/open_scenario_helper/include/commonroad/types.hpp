#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace commonroad {


enum class ObstacleRole : unsigned int
{
  DYNAMIC,
  STATIC
};
enum class ObstacleType : unsigned int
{
  CAR,
  TRAM
};

enum class ObstacleGeometry : unsigned int
{
  RECTANGLE,
  CIRCLE
};

struct Orientation
{
  double exact;
};

struct Time
{
  int exact;
};

struct Velocity
{
  double exact;
};

struct Acceleration
{
  double exact;
};

struct YawRate
{
  double exact;
};

struct SlipAngle
{
  double exact;
};

struct Point
{
  double x;
  double y;
};

struct Position
{
  Point point;
};

struct ObstacleShape
{
  ObstacleGeometry geometry;
  double length;
  double width;
};

struct ObstacleState
{
  Position position;
  Orientation orientation;
  Time time;
  Velocity velocity;
  Acceleration acceleration;
};
struct ObstacleInitialState : public ObstacleState{};

struct ProblemState
{
  Position position;
  Orientation orientation;
  Time time;
  Velocity velocity;
  YawRate yawRate;
  SlipAngle slipAngle;
};

struct ProblemInitialState : public ProblemState{};

struct GoalPosition
{
  int laneletRef;
  Position pos;
  Orientation orientation;
};

struct GoalTime
{
  int intervalStart;
  int intervalEnd;
};

struct GoalState
{
  GoalPosition position;
  GoalTime time;

};

struct ObstacleInformation
{
  int id;
  ObstacleRole role;
  ObstacleType type;
  ObstacleShape shape;
  ObstacleInitialState initialState;

  std::vector<ObstacleState> trajectory;
};

struct PlanningProblem
{
  int id;
  ProblemInitialState initialState;
  GoalState goalState;
};

struct CommonRoadData
{
  std::vector<ObstacleInformation> obstacles;
  PlanningProblem planningProblem;
};

}