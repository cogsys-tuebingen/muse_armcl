#include <muse_armcl/scheduling/scheduler.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 Dummy : public Scheduler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Dummy>;
    using Ptr                 = std::shared_ptr<Dummy>;
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
        const time_t stamp = u->getStamp();
        const time_t time_now(ros::Time::now().toNSec());

        if (next_update_time_.isZero())
            next_update_time_ = time_now;

        if (stamp >= next_update_time_) {
            u->apply(s->getWeightIterator());
            next_update_time_ = time_now;

            may_resample_ = true;
            return true;
        }
        return false;
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
    time_t next_update_time_;
    bool   may_resample_;
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::Dummy, muse_armcl::Scheduler)
