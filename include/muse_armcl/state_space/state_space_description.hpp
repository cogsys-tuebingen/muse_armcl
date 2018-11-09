#ifndef MUSE_ARMCL_STATE_SPACE_DESCRIPTION_HPP
#define MUSE_ARMCL_STATE_SPACE_DESCRIPTION_HPP

#include <cslibs_math_3d/linear/transform.hpp>
#include <cslibs_math_2d/linear/covariance.hpp>

#include <cslibs_mesh_map/edge_particle.h>

namespace muse_armcl {
struct EIGEN_ALIGN16 Sample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using allocator_t = Eigen::aligned_allocator<Sample>;
    using Ptr         = std::shared_ptr<Sample>;
    using state_t     = cslibs_mesh_map::EdgeParticle;

    state_t state;
    double  weight;

    Sample() : weight(0.0) { }
    Sample(state_t& s, const double w) : state(s), weight(w) { }
};

struct StateSpaceDescription
{
    using sample_t               = Sample;
    using state_t                = cslibs_mesh_map::EdgeParticle;
    using state_space_boundary_t = cslibs_math_3d::Vector3d;
    using transform_t            = cslibs_math_3d::Transform3d;
    using covariance_t           = cslibs_math_2d::Covariance3d;
};
}

#endif // MUSE_ARMCL_STATE_SPACE_DESCRIPTION_HPP
