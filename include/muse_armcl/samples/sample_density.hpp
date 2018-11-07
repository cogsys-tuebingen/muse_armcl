#ifndef MUSE_ARMCL_SAMPLE_DENSITY_HPP
#define MUSE_ARMCL_SAMPLE_DENSITY_HPP

#include <muse_smc/samples/sample_density.hpp>
#include <muse_armcl/state_space/state_space_description.hpp>

#include <cslibs_plugins/plugin.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 SampleDensity : public muse_smc::SampleDensity<StateSpaceDescription::sample_t>,
                                    public cslibs_plugins::Plugin
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<SampleDensity>;

    using Ptr          = std::shared_ptr<SampleDensity>;
    using ConstPtr     = std::shared_ptr<SampleDensity const>;
    using state_t      = StateSpaceDescription::state_t;
    using covariance_t = StateSpaceDescription::covariance_t;

    inline const static std::string Type()
    {
        return "muse_armcl::SampleDensity";
    }

    virtual void setup(ros::NodeHandle &nh) = 0;
    // TODO: mean, bzw. cluster und multi-mean... ? what to provide?
    virtual void means(std::vector<state_t> &means, std::vector<covariance_t> &covariances) const = 0;
};
}

#endif // MUSE_ARMCL_SAMPLE_DENSITY_HPP
