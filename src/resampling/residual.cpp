#include <muse_armcl/resampling/resampling.hpp>
#include <muse_smc/resampling/impl/residual.hpp>

namespace muse_armcl {
class Residual : public Resampling
{
protected:
    virtual void doSetup(ros::NodeHandle &nh) override
    {
    }

    void doApply(sample_set_t &sample_set)
    {
        muse_smc::impl::Residual<StateSpaceDescription>::apply(sample_set);
    }

    void doApplyRecovery(sample_set_t &sample_set)
    {
        muse_smc::impl::Residual<StateSpaceDescription>::applyRecovery(uniform_pose_sampler_,
                                                                       recovery_random_pose_probability_,
                                                                       sample_set);
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::Residual, muse_armcl::Resampling)
