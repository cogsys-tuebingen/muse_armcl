#include <muse_armcl/scheduling/scheduler.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 IntegrateAll : public Scheduler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<IntegrateAll>;
    using Ptr                 = std::shared_ptr<IntegrateAll>;
    using update_t            = muse_smc::Update<StateSpaceDescription, cslibs_plugins_data::Data>;
    using resampling_t        = muse_smc::Resampling<StateSpaceDescription>;
    using sample_set_t        = muse_smc::SampleSet<StateSpaceDescription>;
    using time_t              = cslibs_time::Time;
    using duration_t          = cslibs_time::Duration;
    using update_model_map_t  = std::map<std::string, UpdateModel::Ptr>;

    inline void setup(const update_model_map_t &update_models,
                      ros::NodeHandle &nh) override
    {
        may_resample_ = false;
    }

    virtual bool apply(typename update_t::Ptr     &u,
                       typename sample_set_t::Ptr &s) override
    {
            u->apply(s->getWeightIterator());
            may_resample_ = true;
            return true;
    }

    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        auto do_apply = [ &r, &s, this] () {
            r->apply(*s);

            may_resample_ = false;
            return true;
        };
        return  may_resample_ ? do_apply() : false;
    }

private:
    bool   may_resample_;
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::IntegrateAll, muse_armcl::Scheduler)
