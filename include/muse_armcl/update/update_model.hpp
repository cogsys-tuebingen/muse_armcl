#ifndef MUSE_ARMCL_UPDATE_MODEL_HPP
#define MUSE_ARMCL_UPDATE_MODEL_HPP

#include <muse_smc/update/update_model.hpp>
#include <muse_armcl/state_space/state_space_description.hpp>

#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/data.hpp>
#include <ros/ros.h>
namespace muse_armcl {
class EIGEN_ALIGN16 UpdateModel : public muse_smc::UpdateModel<StateSpaceDescription, cslibs_plugins_data::Data>,
                    public cslibs_plugins::Plugin
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<UpdateModel>;
    using Ptr = std::shared_ptr<UpdateModel>;
    using data_t  = cslibs_plugins_data::Data;
    using state_t = StateSpaceDescription::state_t;

    inline const static std::string Type()
    {
        return "muse_armcl::UpdateModel";
    }

    virtual inline std::size_t getId() const override
    {
        return cslibs_plugins::Plugin::getId();
    }

    virtual inline const std::string getName() const override
    {
        return cslibs_plugins::Plugin::getName();
    }

    virtual void setup(ros::NodeHandle &nh) = 0;
};
}

#endif // MUSE_ARMCL_UPDATE_MODEL_HPP
