#ifndef MUSE_ARMCL_PREDICTION_INTEGRAL_HPP
#define MUSE_ARMCL_PREDICTION_INTEGRAL_HPP

#include <muse_smc/prediction/prediction_integral.hpp>
#include <muse_armcl/state_space/state_space_description.hpp>

#include <cslibs_plugins_data/data.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 PredictionIntegral : public muse_smc::PredictionIntegral<StateSpaceDescription, cslibs_plugins_data::Data>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<PredictionIntegral>;
    virtual void add(const typename prediction_model_t::Result::ConstPtr &step) override
    {
    }

    virtual void reset() override
    {
    }

    virtual bool thresholdExceeded() const override
    {
        return true; // always allow resampling
    }

    virtual bool isZero() const override
    {
        return false; // always allow clustering for state estimation
    }

    virtual void info() const override
    {
    }
};
}

#endif // MUSE_ARMCL_PREDICTION_INTEGRAL_HPP
