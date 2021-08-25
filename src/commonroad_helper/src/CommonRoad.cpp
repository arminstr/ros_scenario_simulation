#include "commonroad/CommonRoad.hpp"

#include "commonroad/parser/CommonRoadParser.hpp"

namespace commonroad {

    bool Load(std::string const &file, commonroad::CommonRoadData &common_road_data)
    {
        return parser::CommonRoadParser::Parse(file.c_str(), common_road_data, parser::XmlInputType::FILE);
    }

    bool checkFileTypeXML(std::string file){
        std::string fileEnding;
        try{
            for (int i = 4; i>0; i--){
                fileEnding.push_back(file[file.size()-i]);
            }
            if (fileEnding == "xml") return true;

        } catch(...){
            std::cout << "ERROR IN: checkFileTypeXML" << std::endl;
        }
        return false;
    }

}