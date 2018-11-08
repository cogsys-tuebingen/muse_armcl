#include <muse_armcl/state_space/state_publisher.h>
#include <cslibs_mesh_map/mesh_map_tree.h>

namespace muse_armcl {
void StatePublisher::setup(ros::NodeHandle &nh, map_provider_map_t &map_providers)
{
    // TODO: parameters etc...

    map_providers_ = map_providers;
}

void StatePublisher::publish(const sample_set_t::ConstPtr &sample_set)
{
    if (map_providers_.empty())
        return;

    const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_providers_.begin()->second->getStateSpace();
    if (!ss->isType<MeshMap>())
        return;

    using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
    using mesh_map_t      = cslibs_mesh_map::MeshMap;
    const mesh_map_tree_t::Ptr &m = ss->as<MeshMap>().data();
}

void StatePublisher::publishIntermediate(const sample_set_t::ConstPtr &sample_set)
{
    publish(sample_set);
}

void StatePublisher::publishConstant(const sample_set_t::ConstPtr &sample_set)
{
    publish(sample_set);
}
}
