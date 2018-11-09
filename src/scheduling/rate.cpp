#include <muse_armcl/scheduling/scheduler.hpp>

#include <unordered_map>
#include <ext/pb_ds/priority_queue.hpp>

namespace muse_armcl {
class Rate : public Scheduler
{
public:
    using Ptr                 = std::shared_ptr<Rate>;
    using rate_t              = cslibs_time::Rate;
    using update_t            = muse_smc::Update<StateSpaceDescription, cslibs_plugins_data::Data>;
    using time_priority_map_t = std::unordered_map<id_t, double>;
    using resampling_t        = muse_smc::Resampling<StateSpaceDescription>;
    using sample_set_t        = muse_smc::SampleSet<StateSpaceDescription>;
    using time_t              = cslibs_time::Time;
    using duration_t          = cslibs_time::Duration;
    using update_model_map_t  = std::map<std::string, UpdateModel::Ptr>;

    inline void setup(const update_model_map_t &update_models,
                      ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        double resampling_rate = nh.param<double>(param_name("resampling_rate"), 5.0);
        resampling_period_ = duration_t(resampling_rate > 0.0 ? 1.0 / resampling_rate : 0.0);
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
        const time_t &stamp = s->getStamp();
        const time_t time_now(ros::Time::now().toNSec());

        if (resampling_time_.isZero())
            resampling_time_ = time_now;

        auto do_apply = [ &r, &s, &time_now, this] () {
            r->apply(*s);

            resampling_time_ = time_now + resampling_period_;
            may_resample_ = false;
            return true;
        };
        return (may_resample_ && resampling_time_ <= stamp) ? do_apply() : false;
    }

private:
    time_t              next_update_time_;
    time_t              resampling_time_;
    duration_t          resampling_period_;
    bool                may_resample_;
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::Rate, muse_armcl::Scheduler)
