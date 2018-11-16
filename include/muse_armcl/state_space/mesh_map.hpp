#ifndef MUSE_ARMCL_MESH_MAP_HPP
#define MUSE_ARMCL_MESH_MAP_HPP

#include <muse_smc/state_space/state_space.hpp>
#include <muse_armcl/state_space/state_space_description.hpp>

#include <cslibs_mesh_map/mesh_map_tree.h>

namespace muse_armcl {
class EIGEN_ALIGN16 MeshMap : public muse_smc::StateSpace<StateSpaceDescription>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t   = Eigen::aligned_allocator<MeshMap>;
    using Ptr           = std::shared_ptr<MeshMap>;
    using map_t         = cslibs_mesh_map::MeshMapTree;

    MeshMap(map_t* map, const std::string &frame_id) :
        muse_smc::StateSpace<StateSpaceDescription>(frame_id),
        data_(map)
    {
    }

    virtual ~MeshMap()
    {

    }

    virtual bool validate(const state_t &p) const override
    {
        return true;
    }

    virtual state_space_boundary_t getMin() const override
    {
        return state_space_boundary_t();
    }

    virtual state_space_boundary_t getMax() const override
    {
        return state_space_boundary_t();
    }

    virtual state_space_transform_t getOrigin() const override
    {
        return state_space_transform_t();
    }

    const map_t* data() const
    {
        return data_;
    }

    map_t* data()
    {
        return data_;
    }

private:
    map_t* data_;
};
}

#endif // MUSE_ARMCL_MESH_MAP_HPP
