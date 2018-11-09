#include "sample_density_impl.hpp"
#include <muse_armcl/density/cluster_weighted_distribution.hpp>

namespace muse_armcl {
using WeightedMeans = SampleDensityImpl<ClusterWeightedDistribution>;
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::WeightedMeans, muse_armcl::SampleDensity)
