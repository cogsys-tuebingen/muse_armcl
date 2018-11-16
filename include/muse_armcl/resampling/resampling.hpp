#ifndef MUSE_ARMCL_RESAMPLING_HPP
#define MUSE_ARMCL_RESAMPLING_HPP

#include <muse_smc/resampling/resampling.hpp>
#include <muse_armcl/state_space/state_space_description.hpp>

#include <cslibs_plugins/plugin.hpp>
#include <ros/ros.h>
namespace muse_armcl {
class EIGEN_ALIGN16 Resampling : public muse_smc::Resampling<StateSpaceDescription>,
                                 public cslibs_plugins::Plugin
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Resampling>;
    using Ptr = std::shared_ptr<Resampling>;

    inline const static std::string Type()
    {
        return "muse_armcl::Resampling";
    }

    inline void setup(const typename sample_uniform_t::Ptr &uniform_pose_sampler,
                      const typename sample_normal_t::Ptr  &normal_pose_sampler,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        muse_smc::Resampling<StateSpaceDescription>::setup(uniform_pose_sampler,
                                                           normal_pose_sampler,
                                                           nh.param(param_name("recovery_alpha_fast"), 0.0),
                                                           nh.param(param_name("recovery_alpha_slow"), 0.0),
                                                           nh.param(param_name("variance_thresold"), 0.0));
        doSetup(nh);
    }

protected:
    virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}

#endif // MUSE_ARMCL_RESAMPLING_HPP
