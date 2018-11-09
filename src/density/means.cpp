#include "sample_density_impl.hpp"
#include <muse_armcl/density/cluster_distribution.hpp>

namespace muse_armcl {
using Means = SampleDensityImpl<ClusterDistribution>;
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::Means, muse_armcl::SampleDensity)
