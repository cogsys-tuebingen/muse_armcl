#include <muse_armcl/state_space/mesh_map.h>

namespace muse_armcl {
MeshMap::MeshMap(const map_t::Ptr &map, const std::string &frame_id) :
    muse_smc::StateSpace<StateSpaceDescription>(frame_id),
    data_(map)
{
}

bool MeshMap::validate(const state_t &p) const
{
    return true; // TODO
}

MeshMap::state_space_boundary_t MeshMap::getMin() const
{
    return MeshMap::state_space_boundary_t(); // TODO
}

MeshMap::state_space_boundary_t MeshMap::getMax() const
{
    return MeshMap::state_space_boundary_t(); // TODO
}

MeshMap::state_space_transform_t MeshMap::getOrigin() const
{
    return MeshMap::state_space_transform_t(); // TODO
}

MeshMap::map_t::Ptr& MeshMap::data()
{
    return data_;
}

MeshMap::map_t::Ptr const& MeshMap::data() const
{
    return data_;
}
}
