#ifndef MUSE_ARMCL_CLUSTER_DATA_HPP
#define MUSE_ARMCL_CLUSTER_DATA_HPP

#include <muse_armcl/state_space/state_space_description.hpp>

namespace muse_armcl {
struct EIGEN_ALIGN16 ClusterData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ClusterData>;

    using sample_t        = StateSpaceDescription::sample_t;
    using position_t      = cslibs_math_3d::Vector3d;
    using sample_vector_t = std::vector<std::pair<const sample_t*, position_t>>;

    inline ClusterData() = default;
    inline ClusterData(const sample_t &sample, const position_t &position)
    {
        samples.push_back(std::make_pair(&sample, position));
    }

    inline void merge(const ClusterData& other)
    {
        samples.insert(samples.end(), other.samples.begin(), other.samples.end());
    }

    int             cluster = -1;
    sample_vector_t samples;
};
}

#endif // MUSE_ARMCL_CLUSTER_DATA_HPP
