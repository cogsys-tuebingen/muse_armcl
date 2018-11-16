#ifndef MUSE_ARMCL_SAMPLE_DENSITY_HPP
#define MUSE_ARMCL_SAMPLE_DENSITY_HPP

#include <muse_smc/samples/sample_density.hpp>
#include <muse_armcl/state_space/state_space_description.hpp>
#include <muse_armcl/state_space/mesh_map_provider.hpp>

#include <cslibs_plugins/plugin.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 SampleDensity : public muse_smc::SampleDensity<StateSpaceDescription::sample_t>,
                                    public cslibs_plugins::Plugin
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<SampleDensity>;

    using Ptr                = std::shared_ptr<SampleDensity>;
    using ConstPtr           = std::shared_ptr<SampleDensity const>;
    using sample_t           = StateSpaceDescription::sample_t;
    using sample_vector_t    = std::vector<sample_t, sample_t::allocator_t>;
    using map_provider_map_t = std::map<std::string, MeshMapProvider::Ptr>;

    inline const static std::string Type()
    {
        return "muse_armcl::SampleDensity";
    }

    virtual void setup(const map_provider_map_t &map_providers,
                       ros::NodeHandle &nh) = 0;
    virtual std::size_t histogramSize() const = 0;
    virtual void contacts(sample_vector_t &states) const = 0;
};
}

#endif // MUSE_ARMCL_SAMPLE_DENSITY_HPP
