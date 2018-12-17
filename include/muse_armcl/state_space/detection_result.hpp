#ifndef DETECTION_RESULT_HPP
#define DETECTION_RESULT_HPP
#include <string>
#include <sstream>
#include <fstream>
namespace muse_armcl {

struct DetectionResult{
    double s;
    double phi;
    double error_dist;
    double error_ori;
    double contact_force;
    double contact_force_true;
    int true_point;
    int closest_point;

    inline std::string to_string(const std::string delimiter = std::string(";")) const
    {
        std::stringstream sstream;
        sstream << true_point << delimiter;
        sstream << closest_point << delimiter;
        sstream << s << delimiter;
        sstream << phi << delimiter;
        sstream << contact_force << delimiter;
        sstream << contact_force_true << delimiter;
        sstream << error_dist << delimiter;
        sstream << error_ori << delimiter;

        return sstream.str();
    }

    inline std::string header(const std::string delimiter = std::string(";")) const
    {
        std::stringstream sstream;
        sstream << "true_point" << delimiter;
        sstream << "closest_point" << delimiter;
        sstream << "s" << delimiter;
        sstream << "phi" << delimiter;
        sstream << "contact_force" << delimiter;
        sstream << "contact_force_true" << delimiter;
        sstream << "error_dist" << delimiter;
        sstream << "error_ori" << delimiter;

        return sstream.str();
    }

};

inline void save(const std::vector<DetectionResult>& data, const std::string& file)
{
    std::ofstream of(file);
    of << data.front().header() << std::endl;
    for(const DetectionResult& d : data){
        of << d.to_string() << std::endl;
    }
    of.close();
}

}

#endif // DETECTION_RESULT_HPP
