#include <muse_armcl/state_space/state_publisher.h>
#include <muse_armcl/density/sample_density.hpp>

#include <cslibs_mesh_map/mesh_map_tree.h>
#include <cslibs_mesh_map/cslibs_mesh_map_visualization.h>

#include <visualization_msgs/MarkerArray.h>

namespace muse_armcl {
void StatePublisher::setup(ros::NodeHandle &nh, map_provider_map_t &map_providers)
{
    map_providers_ = map_providers;

    const std::string topic_particles = nh.param<std::string>("topic_particles", "muse_armcl/particles");
    const std::string topic_contacts  = nh.param<std::string>("topic_contacts", "muse_armcl/contacts");

    pub_particles_ = nh.advertise<visualization_msgs::MarkerArray>(topic_particles, 1);
    pub_contacts_  = nh.advertise<visualization_msgs::MarkerArray>(topic_contacts, 1);
}

void StatePublisher::publish(const sample_set_t::ConstPtr &sample_set)
{
    publish(sample_set, true);
}

void StatePublisher::publishIntermediate(const sample_set_t::ConstPtr &sample_set)
{
    publish(sample_set, false);
}

void StatePublisher::publishConstant(const sample_set_t::ConstPtr &sample_set)
{
    publish(sample_set, false);
}

void StatePublisher::publish(const sample_set_t::ConstPtr &sample_set, const bool &publish_contacts)
{
    if (map_providers_.empty())
        return;

    /// use only the first map...
    const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_providers_.begin()->second->getStateSpace();
    if (!ss->isType<MeshMap>())
        return;

    using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
    const mesh_map_tree_t::Ptr &map = ss->as<MeshMap>().data();
    const ros::Time stamp = ros::Time().fromNSec(sample_set->getStamp().nanoseconds());

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker msg;
    msg.header.stamp = stamp;
    msg.action = visualization_msgs::Marker::MODIFY;
    msg.lifetime = ros::Duration(0.2);
    msg.id = 0;

    /// publish all particles
    for (const StateSpaceDescription::sample_t& p : sample_set->getSamples()) {
        mesh_map_tree_t* p_map = map->getNode(p.state.map_id);
        if (p_map) {
            cslibs_mesh_map::visualization::visualizeEdgeParticle(p.state, p_map->map_, msg);
            markers.markers.push_back(msg);
        }
    }
    pub_particles_.publish(markers);

    if (publish_contacts) {
        markers.markers.clear();

        /// density estimation
        SampleDensity::ConstPtr density = std::dynamic_pointer_cast<SampleDensity const>(sample_set->getDensity());
        if (!density) {
            std::cerr << "[StatePublisher]: Incomaptible sample density estimation!" << "\n";
            return;
        }

        /// publish all detected contacts
        std::vector<StateSpaceDescription::sample_t> states;
        density->contacts(states);
        for (const StateSpaceDescription::sample_t& p : states) {
            mesh_map_tree_t* p_map = map->getNode(p.state.map_id);
            if (p_map) {
                cslibs_mesh_map::visualization::visualizeEdgeParticle(p.state, p_map->map_, msg);
                markers.markers.push_back(msg);
            }
        }
        pub_contacts_.publish(markers);
    }
}
}
