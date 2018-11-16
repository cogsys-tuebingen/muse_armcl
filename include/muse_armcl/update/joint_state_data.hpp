#ifndef MUSE_ARMCL_JOINT_STATE_DATA_HPP
#define MUSE_ARMCL_JOINT_STATE_DATA_HPP
#include <eigen3/Eigen/Core>
#include <cslibs_plugins_data/data.hpp>
#include <sensor_msgs/JointState.h>

namespace muse_armcl {
class EIGEN_ALIGN16 JointStateData : public cslibs_plugins_data::Data
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<JointStateData>;
    using Ptr = std::shared_ptr<JointStateData>;

    JointStateData() = delete;
    inline JointStateData(const std::string              &frame,
                          const cslibs_time::TimeFrame   &time_frame,
                          const cslibs_time::Time        &time_received,
                          const std::vector<std::string> &_name,
                          const std::vector<double>      &_position,
                          const std::vector<double>      &_velocity,
                          const std::vector<double>      &_effort) :
        cslibs_plugins_data::Data(frame, time_frame, time_received),
        name(_name),
        position(_position),
        velocity(_velocity),
        effort(_effort)
    {
    }

    std::vector<std::string> name;
    std::vector<double>      position;
    std::vector<double>      velocity;
    std::vector<double>      effort;
};
}

#endif // MUSE_ARMCL_JOINT_STATE_DATA_HPP
