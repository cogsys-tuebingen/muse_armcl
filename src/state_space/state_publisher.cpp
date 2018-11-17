#include <muse_armcl/state_space/state_publisher.h>
#include <muse_armcl/density/sample_density.hpp>

#include <cslibs_mesh_map/mesh_map_tree.h>
#include <cslibs_mesh_map/cslibs_mesh_map_visualization.h>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

namespace muse_armcl {
void StatePublisher::setup(ros::NodeHandle &nh, map_provider_map_t &map_providers)
{
    const std::string map_provider_id = nh.param<std::string>("map", ""); /// toplevel parameter
    if (map_provider_id == "")
        throw std::runtime_error("[UniformSampling]: No map provider was found!");

    if (map_providers.find(map_provider_id) == map_providers.end())
        throw std::runtime_error("[UniformSampling]: Cannot find map provider '" + map_provider_id + "'!");

    map_provider_ = map_providers.at(map_provider_id);

    const std::string topic_particles = nh.param<std::string>("topic_particles", "particles");
    const std::string topic_contacts  = nh.param<std::string>("topic_contacts", "contacts");

//    pub_particles_ = nh.advertise<visualization_msgs::MarkerArray>(topic_particles, 1);
    pub_particles_ = nh.advertise<sensor_msgs::PointCloud2>(topic_particles, 1);
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
    if (!map_provider_)
        return;

    /// get the map
    const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
    if (!ss->isType<MeshMap>())
        return;

    using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
    const mesh_map_tree_t* map = ss->as<MeshMap>().data();
    uint64_t nsecs = static_cast<uint64_t>(sample_set->getStamp().nanoseconds());
    const ros::Time stamp = ros::Time().fromNSec(nsecs);

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker msg;
    msg.header.stamp = stamp;
    msg.id = 0;

    msg.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(msg);

    std::shared_ptr<cslibs_math_3d::PointcloudRGB3d> part_cloud(new cslibs_math_3d::PointcloudRGB3d);

    /// publish all particles
    for (const StateSpaceDescription::sample_t& p : sample_set->getSamples()) {
        const mesh_map_tree_t* p_map = map->getNode(p.state.map_id);
        if (p_map) {
            cslibs_math_3d::Transform3d T = map->getTranformToBase(p_map->map_.frame_id_);
            cslibs_math_3d::Point3d pos = p.state.getPosition(p_map->map_);
            pos = T * pos;
            cslibs_math::color::Color color(cslibs_math::color::interpolateColor(p.weight,0,1.0));
            cslibs_math_3d::PointRGB3d point(pos, 0.9f, color);
            part_cloud->insert(point);

//            cslibs_mesh_map::visualization::visualizeEdgeParticle(p.state, p_map->map_, msg);
            //msg.scale.x = p.weight; // TODO: test
//            msg.lifetime = ros::Duration(0.0);
//            markers.markers.push_back(msg);
        }
    }
    sensor_msgs::PointCloud2 cloud;
    cslibs_math_ros::sensor_msgs::conversion_3d::from(part_cloud, cloud);
    cloud.header.frame_id = map->parent_id_;
    cloud.header.stamp = stamp;
    pub_particles_.publish(cloud);

    if (publish_contacts) {
        markers.markers.clear();
        msg.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(msg);

        /// density estimation
        SampleDensity::ConstPtr density = std::dynamic_pointer_cast<SampleDensity const>(sample_set->getDensity());
        if (!density) {
            std::cerr << "[StatePublisher]: Incomaptible sample density estimation!" << "\n";
            return;
        }

        /// publish all detected contacts
        std::vector<StateSpaceDescription::sample_t, StateSpaceDescription::sample_t::allocator_t> states;
        density->contacts(states);
        for (const StateSpaceDescription::sample_t& p : states) {
            const mesh_map_tree_t* p_map = map->getNode(p.state.map_id);
            if (p_map) {
                cslibs_mesh_map::visualization::visualizeEdgeParticle(p.state, p_map->map_, msg);
                //msg.scale.x = p.weight; // TODO: test
                msg.lifetime = ros::Duration(0.0);
                msg.color.r = 1.0;
                msg.color.g = 0.0;
                msg.color.b = 0.0;
                markers.markers.push_back(msg);
            }
        }
        pub_contacts_.publish(markers);
    }
}
}
