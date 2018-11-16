#ifndef MUSE_ARMCL_SCHEDULER_HPP
#define MUSE_ARMCL_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>
#include <muse_armcl/state_space/state_space_description.hpp>
#include <muse_armcl/update/update_model.hpp>

#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/data.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 Scheduler : public muse_smc::Scheduler<StateSpaceDescription, cslibs_plugins_data::Data>,
                  public cslibs_plugins::Plugin
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Scheduler>;
    using Ptr                = std::shared_ptr<Scheduler>;
    using update_model_map_t = std::map<std::string, UpdateModel::Ptr>;

    inline const static std::string Type()
    {
        return "muse_armcl::Scheduler";
    }

    virtual void setup(const update_model_map_t &update_models,
                       ros::NodeHandle &nh) = 0;
};
}

#endif // MUSE_ARMCL_SCHEDULER_HPP
