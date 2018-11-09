#include "sample_density_impl.hpp"
#include <muse_armcl/density/cluster_dominant.hpp>

namespace muse_armcl {
using Dominants = SampleDensityImpl<ClusterDominant>;
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::Dominants, muse_armcl::SampleDensity)
