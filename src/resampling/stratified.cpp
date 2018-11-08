#include <muse_armcl/resampling/resampling.hpp>
#include <muse_smc/resampling/impl/stratified.hpp>

namespace muse_armcl {
class Stratified : public Resampling
{
protected:
    virtual void doSetup(ros::NodeHandle &nh) override
    {
    }

    void doApply(sample_set_t &sample_set)
    {
        muse_smc::impl::Stratified<StateSpaceDescription>::apply(sample_set);
    }

    void doApplyRecovery(sample_set_t &sample_set)
    {
        muse_smc::impl::Stratified<StateSpaceDescription>::applyRecovery(uniform_pose_sampler_,
                                                                         recovery_random_pose_probability_,
                                                                         sample_set);
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::Stratified, muse_armcl::Resampling)
