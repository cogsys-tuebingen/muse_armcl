#ifndef GROUND_TRUTH_PARTICLE_SET_DISTANCE_HPP
#define GROUND_TRUTH_PARTICLE_SET_DISTANCE_HPP
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <fstream>
namespace muse_armcl {
struct  GroundTruthParticleSetDistance
{
     int true_point;
     int link;
     double likely_hood;
     double distance;
     double angle;
     double contact_force;
     double contact_force_true;

     inline std::string to_string(const std::string delimiter = std::string(";")) const
     {
         std::stringstream sstream;
         sstream << true_point << delimiter;
         sstream << link << delimiter;
         sstream << likely_hood << delimiter;
         sstream << distance << delimiter;
         sstream << angle << delimiter;
         sstream << contact_force << delimiter;
         sstream << contact_force_true << delimiter;

         return sstream.str();
     }

     inline std::string header(const std::string delimiter = std::string(";")) const
     {
         std::stringstream sstream;
         sstream << "true_point" << delimiter;
         sstream << "link" << delimiter;
         sstream << "likely_hood" << delimiter;
         sstream << "distance" << delimiter;
         sstream << "angle" << delimiter;
         sstream << "contact_force" << delimiter;
         sstream << "contact_force_true" << delimiter;
         return sstream.str();
     }
};

inline void save(std::vector<GroundTruthParticleSetDistance>& data, const std::string& file)
{
    auto exists = [](const std::string& name) {
        std::ifstream f(name.c_str());
        return f.good();
    };
    bool print_header = !exists(file);
    std::ofstream of(file, std::ofstream::out | std::ofstream::app );
    if(print_header)
        of << data.front().header() << std::endl;
    for(const GroundTruthParticleSetDistance& d : data){
        of << d.to_string() << std::endl;
    }
    of.close();
    data.clear();
}

}
#endif // GROUND_TRUTH_PARTICLE_SET_DISTANCE_HPP
