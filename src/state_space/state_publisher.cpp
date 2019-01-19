#include <muse_armcl/state_space/state_publisher.h>
#include <muse_armcl/density/sample_density.hpp>

#include <cslibs_mesh_map/mesh_map_tree.h>
#include <cslibs_mesh_map/cslibs_mesh_map_visualization.h>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>
#include <cslibs_math_ros/geometry_msgs/conversion_3d.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <cslibs_kdl_msgs/ContactMessageArray.h>

namespace muse_armcl {
void StatePublisher::setup(ros::NodeHandle &nh, map_provider_map_t &map_providers)
{
    const std::string map_provider_id = nh.param<std::string>("map", ""); /// toplevel parameter
    if (map_provider_id == "")
        throw std::runtime_error("[UniformSampling]: No map provider was found!");

    if (map_providers.find(map_provider_id) == map_providers.end())
        throw std::runtime_error("[UniformSampling]: Cannot find map provider '" + map_provider_id + "'!");

    map_provider_ = map_providers.at(map_provider_id);

    const std::string topic_particles     = nh.param<std::string>("topic_particles", "particles");
    const std::string topic_contacts      = nh.param<std::string>("topic_contacts", "contacts");
    const std::string topic_contacts_vis  = nh.param<std::string>("topic_contacts_visualization", "contacts_visualization");

    contact_marker_r_ = nh.param<double>("contact_marker_r", 0.0);
    contact_marker_g_ = nh.param<double>("contact_marker_g", 0.0);
    contact_marker_b_ = nh.param<double>("contact_marker_b", 1.0);

    no_contact_torque_threshold_ = nh.param<double>("no_contact_threshold", 0.1);

    pub_particles_     = nh.advertise<sensor_msgs::PointCloud2>(topic_particles, 1);
    pub_contacts_      = nh.advertise<cslibs_kdl_msgs::ContactMessageArray>(topic_contacts, 1);
    pub_contacts_vis_  = nh.advertise<visualization_msgs::MarkerArray>(topic_contacts_vis, 1);
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
    using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;
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
        const mesh_map_tree_node_t* p_map = map->getNode(p.state.map_id);
        if (p_map) {
            cslibs_math_3d::Transform3d T = map->getTranformToBase(p_map->map.frame_id_);
            cslibs_math_3d::Point3d pos = p.state.getPosition(p_map->map);
            pos = T * pos;
            cslibs_math::color::Color color(cslibs_math::color::interpolateColor(p.state.last_update,0,1.0));
            cslibs_math_3d::PointRGB3d point(pos, 0.9f, color);
            part_cloud->insert(point);
        }
    }
    sensor_msgs::PointCloud2 cloud;
    cslibs_math_ros::sensor_msgs::conversion_3d::from(part_cloud, cloud);
    cloud.header.frame_id = map->front()->frameId();
    cloud.header.stamp = stamp;
    pub_particles_.publish(cloud);


    if (publish_contacts) {
        markers.markers.clear();
        msg.action = visualization_msgs::Marker::MODIFY;

        /// density estimation
        SampleDensity::ConstPtr density = std::dynamic_pointer_cast<SampleDensity const>(sample_set->getDensity());
        if (!density) {
            std::cerr << "[StatePublisher]: Incomaptible sample density estimation!" << "\n";
            return;
        }

        /// publish all detected contacts
        std::vector<StateSpaceDescription::sample_t, StateSpaceDescription::sample_t::allocator_t> states;
        density->contacts(states);
        std::cout << "[StatePublisher]: number of contacts: " << states.size() << std::endl;

        msg.lifetime = ros::Duration(0.2);
        msg.color.a = 0.8;
        msg.color.r = contact_marker_r_;
        msg.color.g = contact_marker_g_;
        msg.color.b = contact_marker_b_;
        msg.scale.x = 0.005;
        msg.scale.y = 0.01;
        msg.scale.z = 0.01;
        msg.ns = "contact";
        msg.type = visualization_msgs::Marker::ARROW;
        msg.action = visualization_msgs::Marker::MODIFY;
        msg.points.resize(2);
        cslibs_kdl_msgs::ContactMessageArray contact_msg;
        bool diff_colors = states.size() > 1;
        for (const StateSpaceDescription::sample_t& p : states) {
            const mesh_map_tree_node_t* p_map = map->getNode(p.state.map_id);
            if (p_map){

                cslibs_kdl_msgs::ContactMessage contact;
                contact.header.frame_id = p_map->frameId();
                contact.header.stamp = stamp;
                cslibs_math_3d::Vector3d point = p.state.getPosition(p_map->map);
                cslibs_math_3d::Vector3d direction = p.state.getDirection(p_map->map);
                contact.location = cslibs_math_ros::geometry_msgs::conversion_3d::toVector3(point);
                contact.direction = cslibs_math_ros::geometry_msgs::conversion_3d::toVector3(direction);
                contact.force = p.state.force;
                msg.header.frame_id = p_map->map.frame_id_;
                ++msg.id;
                double fac = 1.0;
                if(diff_colors){
                    fac = p.state.last_update;
                    msg.color.r *= fac;
                    msg.color.g *= fac;
                    msg.color.b *= fac;
                }
                msg.points[0] = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(point - direction * 0.2 * fac);
                msg.points[1] = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(point);

                markers.markers.push_back(msg);
                contact_msg.contacts.push_back(contact);
            }
        }
        pub_contacts_.publish(contact_msg);
        pub_contacts_vis_.publish(markers);
    }
}
}

