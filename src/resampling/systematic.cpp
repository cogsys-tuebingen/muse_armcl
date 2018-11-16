#include <muse_armcl/resampling/resampling.hpp>
#include <muse_smc/resampling/impl/systematic.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 Systematic : public Resampling
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Systematic>;
protected:
    virtual void doSetup(ros::NodeHandle &nh) override
    {
    }

    void doApply(sample_set_t &sample_set)
    {
        muse_smc::impl::Systematic<StateSpaceDescription>::apply(sample_set);
    }

    void doApplyRecovery(sample_set_t &sample_set)
    {
        muse_smc::impl::Systematic<StateSpaceDescription>::applyRecovery(uniform_pose_sampler_,
                                                                         recovery_random_pose_probability_,
                                                                         sample_set);
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::Systematic, muse_armcl::Resampling)
