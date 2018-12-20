#ifndef DETECTION_RESULT_HPP
#define DETECTION_RESULT_HPP
#include <sys/stat.h>
#include <unistd.h>
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

inline void save(std::vector<DetectionResult>& data, const std::string& file)
{
    auto exists = [](const std::string& name) {
        std::ifstream f(name.c_str());
        return f.good();
    };
    bool print_header = !exists(file);
    std::ofstream of(file, std::ofstream::out | std::ofstream::app );
    if(print_header)
        of << data.front().header() << std::endl;
    for(const DetectionResult& d : data){
        of << d.to_string() << std::endl;
    }
    of.close();
    data.clear();
}

}

#endif // DETECTION_RESULT_HPP
