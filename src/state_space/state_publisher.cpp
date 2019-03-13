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

    contact_marker_scale_x_ = nh.param<double>("contact_marker_scale_x", 0.005 * 1.9);
    contact_marker_scale_y_ = nh.param<double>("contact_marker_scale_y", 0.01 * 2.8);
    contact_marker_scale_z_ = nh.param<double>("contact_marker_scale_z", 0.01 * 3.3);

    no_contact_torque_threshold_ = nh.param<double>("no_contact_threshold", 0.1);

    std::string path;
    path = nh.param<std::string>("contact_points_file", std::string(""));
    if(path != ""){
        std::vector<cslibs_kdl::KDLTransformation> labeled_contact_points;
        cslibs_kdl::load(path, labeled_contact_points);
        labeled_contact_points_.clear();
        for(auto p : labeled_contact_points){
            std::string name = p.name;
            name.erase(0,1);
            int label = std::stoi(name);
            labeled_contact_points_[label] = p;
        }
    }

    pub_particles_     = nh.advertise<sensor_msgs::PointCloud2>(topic_particles, 10);
    pub_contacts_      = nh.advertise<cslibs_kdl_msgs::ContactMessageArray>(topic_contacts, 10);
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

    const mesh_map_tree_t* map = ss->as<MeshMap>().data();
    uint64_t nsecs = static_cast<uint64_t>(sample_set->getStamp().nanoseconds());
    const ros::Time stamp = ros::Time().fromNSec(nsecs);

    publishSet(sample_set, map, stamp);


    if (publish_contacts) {
        visualization_msgs::Marker msg;
        msg.lifetime = ros::Duration(0.2);
        msg.color.a = 0.8;
        msg.color.r = contact_marker_r_;
        msg.color.g = contact_marker_g_;
        msg.color.b = contact_marker_b_;
        msg.scale.x = contact_marker_scale_x_;
        msg.scale.y = contact_marker_scale_y_;
        msg.scale.z = contact_marker_scale_z_;
        msg.ns = "contact";
        msg.type = visualization_msgs::Marker::ARROW;
        msg.action = visualization_msgs::Marker::MODIFY;
        msg.points.resize(2);

        /// density estimation
        ContactPointHistogram::ConstPtr histogram = std::dynamic_pointer_cast<ContactPointHistogram const>(sample_set->getDensity());
        if(histogram && !labeled_contact_points_.empty()){
            std::vector<std::pair<int,double>> labels;
            histogram->getTopLabels(labels);
            publishDiscretePoints(labels, stamp, msg);
        } else {
            publishContacts(sample_set, map, stamp, msg);
        }

    }
}
void StatePublisher::publishContacts(const typename sample_set_t::ConstPtr & sample_set,
                                     const mesh_map_tree_t* map,
                                     const ros::Time& stamp,
                                     visualization_msgs::Marker& msg)
{
    visualization_msgs::MarkerArray markers;
    msg.header.stamp = stamp;
    msg.id = 0;

    SampleDensity::ConstPtr density = std::dynamic_pointer_cast<SampleDensity const>(sample_set->getDensity());
    if (!density) {
        std::cerr << "[StatePublisher]: Incomaptible sample density estimation!" << "\n";
        return;
    }

    /// publish all detected contacts
    std::vector<StateSpaceDescription::sample_t, StateSpaceDescription::sample_t::allocator_t> states;
    density->contacts(states);
    //        std::cout << "[StatePublisher]: number of contacts: " << states.size() << std::endl;


    cslibs_kdl_msgs::ContactMessageArray contact_msg;
    bool diff_colors = states.size() > 1;
    for (const StateSpaceDescription::sample_t& p : states) {
        const mesh_map_tree_node_t* p_map = map->getNode(p.state.map_id);
        if (p_map && std::fabs(p.state.force) > 1e-3){

            cslibs_kdl_msgs::ContactMessage contact;
            contact.header.frame_id = p_map->frameId();
            contact.header.stamp = stamp;
            cslibs_math_3d::Vector3d point = p.state.getPosition(p_map->map);
            cslibs_math_3d::Vector3d direction = p.state.getDirection(p_map->map);
            contact.location = cslibs_math_ros::geometry_msgs::conversion_3d::toVector3(point);
            contact.direction = cslibs_math_ros::geometry_msgs::conversion_3d::toVector3(direction);
            contact.force = static_cast<float>(p.state.force);
            msg.header.frame_id = p_map->map.frame_id_;
            ++msg.id;
            double fac = 1.0;
            if(diff_colors){
                fac = p.state.last_update;
                msg.color.r *= fac;
                msg.color.g *= fac;
                msg.color.b *= fac;
            }
            msg.points[0] = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(point - direction * 0.2 );
            msg.points[1] = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(point);

            markers.markers.push_back(msg);
            contact_msg.contacts.push_back(contact);

        }
    }

    pub_contacts_.publish(contact_msg);
    pub_contacts_vis_.publish(markers);
}

void StatePublisher::publishSet(const typename sample_set_t::ConstPtr &sample_set,
                                const mesh_map_tree_t* map,
                                const ros::Time& stamp)
{
    std::shared_ptr<cslibs_math_3d::PointcloudRGB3d> part_cloud(new cslibs_math_3d::PointcloudRGB3d);
    /// publish all particles
    for (const StateSpaceDescription::sample_t& p : sample_set->getSamples()) {
        const mesh_map_tree_node_t* p_map = map->getNode(p.state.map_id);
        if (p_map) {
            cslibs_math_3d::Transform3d T = map->getTranformToBase(p_map->map.frame_id_);
            cslibs_math_3d::Point3d pos = p.state.getPosition(p_map->map);
            pos = T * pos;
            cslibs_math::color::Color<double> color(cslibs_math::color::interpolateColor<double>(p.state.last_update,0,1.0));
            cslibs_math_3d::PointRGB3d point(pos, 0.9f, color);
            part_cloud->insert(point);
        }
    }
    sensor_msgs::PointCloud2 cloud;
    cslibs_math_ros::sensor_msgs::conversion_3d::from<double>(part_cloud, cloud);
    cloud.header.frame_id = map->front()->frameId();
    cloud.header.stamp = stamp;
    pub_particles_.publish(cloud);
}

void StatePublisher::publishDiscretePoints(const std::vector<std::pair<int, double>>& labels,
                                           const ros::Time& stamp,
                                           visualization_msgs::Marker& msg)
{
    cslibs_kdl_msgs::ContactMessageArray contact_msg;
    visualization_msgs::MarkerArray markers;
    msg.header.stamp = stamp;
    for(const std::pair<int, double>& p : labels){
        if(p.first <= 0){
            std::cerr << "[StatePublisher]: Got label "<< p.first << " clusering failed?? "<< "\n";
            continue; // no contact detected?
        }
        cslibs_kdl_msgs::ContactMessage contact;
        const cslibs_kdl::KDLTransformation& t = labeled_contact_points_.at(p.first);
        contact.header.frame_id = t.parent;
        msg.header.frame_id = t.parent;
        contact.header.stamp = stamp;
        contact.force = static_cast<float>(p.second);
        contact.location.x = t.frame.p.x();
        contact.location.y = t.frame.p.y();
        contact.location.z = t.frame.p.z();
        KDL::Vector dir = t.frame.M * KDL::Vector(-1,0,0);
        contact.direction.x = dir.x();
        contact.direction.y = dir.y();
        contact.direction.z = dir.z();
        geometry_msgs::Point p0;
        p0.x = t.frame.p.x();
        p0.y = t.frame.p.y();
        p0.z = t.frame.p.z();
        geometry_msgs::Point p1;
        p1.x = t.frame.p.x()  - 0.2 * dir.x();
        p1.y = t.frame.p.y()  - 0.2 * dir.y();
        p1.z = t.frame.p.z()  - 0.2 * dir.z();
        msg.points[0] = p1;
        msg.points[1] = p0;

        markers.markers.push_back(msg);
        contact_msg.contacts.push_back(contact);

    }
    pub_contacts_.publish(contact_msg);
    pub_contacts_vis_.publish(markers);
}
}

