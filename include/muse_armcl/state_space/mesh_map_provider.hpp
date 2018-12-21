#ifndef MUSE_ARMCL_MESH_MAP_PROVIDER_HPP
#define MUSE_ARMCL_MESH_MAP_PROVIDER_HPP

#include <muse_smc/state_space/state_space_provider.hpp>
#include <muse_armcl/state_space/mesh_map.hpp>

#include <cslibs_plugins/plugin.hpp>
#include <cslibs_math_ros/tf/tf_provider.hpp>
#include <ros/ros.h>

namespace muse_armcl {
class EIGEN_ALIGN16 MeshMapProvider : public muse_smc::StateSpaceProvider<StateSpaceDescription>,
                        public cslibs_plugins::Plugin
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<MeshMapProvider>;
    using Ptr      = std::shared_ptr<MeshMapProvider>;
    using ConstPtr = std::shared_ptr<MeshMapProvider const>;
    using tf_provider_t = cslibs_math_ros::tf::TFProvider;

    virtual ~MeshMapProvider() = default;

    inline const static std::string Type()
    {
        return "muse_armcl::MeshMapProvider";
    }

    virtual inline const std::string getName() const override
    {
        return cslibs_plugins::Plugin::getName();
    }

    void setup(const tf_provider_t::Ptr &tf,
                       ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        tf_timeout_ = ros::Duration(nh.param(param_name("tf_timeout"), 0.1));
        tf_ = tf;

        doSetup(nh);
    }

protected:
    ros::Duration      tf_timeout_;
    tf_provider_t::Ptr tf_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}

#endif // MUSE_ARMCL_MESH_MAP_PROVIDER_HPP
