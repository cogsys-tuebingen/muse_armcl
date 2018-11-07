#ifndef MUSE_ARMCL_MESH_MAP_H
#define MUSE_ARMCL_MESH_MAP_H

#include <muse_smc/state_space/state_space.hpp>
#include <muse_armcl/state_space/state_space_description.hpp>

#include <cslibs_mesh_map/mesh_map_tree.h>

namespace muse_armcl {
class MeshMap : public muse_smc::StateSpace<StateSpaceDescription>
{
public:
    using Ptr = std::shared_ptr<MeshMap>;
    using map_t = cslibs_mesh_map::MeshMapTree;

    MeshMap(const map_t::Ptr &map, const std::string &frame_id);

    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const state_t &p) const override;
    map_t::Ptr& data();
    map_t::Ptr const & data() const;

private:
    map_t::Ptr data_;
};
}

#endif // MUSE_ARMCL_MESH_MAP_H
