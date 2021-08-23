#include "commonroad/CommonRoad.hpp"

#include "commonroad/parser/CommonRoadParser.hpp"

namespace commonroad {

    bool Load(std::string const &file, commonroad::CommonRoadData &common_road_data)
    {
        return parser::CommonRoadParser::Parse(file.c_str(), common_road_data, parser::XmlInputType::FILE);
    }

}