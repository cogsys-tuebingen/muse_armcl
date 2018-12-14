#pragma once

#include <muse_armcl/state_space/state_publisher.h>

namespace muse_armcl {
class EIGEN_ALIGN16 StatePublisherOffline : public StatePublisher
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<StatePublisherOffline>;

    using Ptr                = std::shared_ptr<StatePublisherOffline>;

    /// callback_t
    using time_callback_t = cslibs_utility::common::delegate<void(const cslibs_time::Time &t)>;


    ///  weight_iterator_t::notify_touch::template    from<sample_set_t, &sample_set_t::weightStatisticReset>(this)
    void setup(ros::NodeHandle &nh, map_provider_map_t &map_providers, time_callback_t &set_time)
    {
        StatePublisher::setup(nh, map_providers);
        set_time_ = set_time;
    }

    virtual void publish(const typename sample_set_t::ConstPtr &sample_set) override
    {
        StatePublisher::publish(sample_set);
    }
    virtual void publishIntermediate(const typename sample_set_t::ConstPtr &sample_set) override
    {
        StatePublisher::publishIntermediate(sample_set);
    }
    virtual void publishConstant(const typename sample_set_t::ConstPtr &sample_set) override
    {
        StatePublisher::publishConstant(sample_set);
    }

private:
    time_callback_t     set_time_;

};
}
