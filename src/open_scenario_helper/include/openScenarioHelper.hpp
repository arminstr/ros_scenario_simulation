#include "xosc.h"
#include "commonroad/CommonRoad.hpp"

class openScenarioHelper{
    public:
    static bool Load(const std::string &path, commonroad::CommonRoadData &cr);
    static bool checkFileTypeXOSC(std::string file);
};