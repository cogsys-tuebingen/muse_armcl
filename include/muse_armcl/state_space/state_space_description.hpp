#ifndef MUSE_ARMCL_STATE_SPACE_DESCRIPTION_HPP
#define MUSE_ARMCL_STATE_SPACE_DESCRIPTION_HPP

#include <cslibs_mesh_map/edge_particle.h>
#include <cslibs_math_3d/linear/transform.hpp>
#include <cslibs_math_3d/linear/covariance.hpp>

namespace muse_armcl {
struct StateSpaceDescription
{
    using sample_t               = cslibs_mesh_map::EdgeParticle;
    using state_t                = cslibs_mesh_map::EdgeParticle;
    using state_space_boundary_t = cslibs_math_3d::Vector3d;
    using transform_t            = cslibs_math_3d::Transform3d;
    using covariance_t           = cslibs_math_3d::Covariance3d;
};
}

#endif // MUSE_ARMCL_STATE_SPACE_DESCRIPTION_HPP
