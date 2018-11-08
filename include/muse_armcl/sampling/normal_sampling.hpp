#ifndef MUSE_ARMCL_NORMAL_SAMPLING_HPP
#define MUSE_ARMCL_NORMAL_SAMPLING_HPP

#include <muse_smc/sampling/normal.hpp>
#include <muse_armcl/state_space/mesh_map_provider.hpp>

#include <cslibs_plugins/plugin.hpp>
#include <cslibs_math_ros/tf/tf_provider.hpp>

namespace muse_armcl {
class NormalSampling : public muse_smc::NormalSampling<StateSpaceDescription>,
                       public cslibs_plugins::Plugin
{
public:
    using Ptr = std::shared_ptr<NormalSampling>;
    using tf_provider_t = cslibs_math_ros::tf::TFProvider;

    inline const static std::string Type()
    {
        return "muse_armcl::NormalSampling";
    }

    inline void setup(const std::map<std::string, MeshMapProvider::Ptr> &map_providers,
                      const tf_provider_t::Ptr &tf,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        sample_size_ = static_cast<std::size_t>(nh.param(param_name("sample_size"), 500));
        sampling_timeout_ = ros::Duration(nh.param(param_name("sampling_timeout"), 10.0));
        tf_timeout_ = ros::Duration(nh.param(param_name("tf_timeout"), 0.1));
        tf_ = tf;

        doSetup(map_providers, nh);
    }

protected:
    std::size_t        sample_size_;
    ros::Duration      sampling_timeout_;
    ros::Duration      tf_timeout_;
    tf_provider_t::Ptr tf_;

    virtual void doSetup(const std::map<std::string, MeshMapProvider::Ptr> &map_providers,
                         ros::NodeHandle &nh) = 0 ;
};
}

#endif // MUSE_ARMCL_NORMAL_SAMPLING_HPP
