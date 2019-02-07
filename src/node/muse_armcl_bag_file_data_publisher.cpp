#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <muse_armcl/evaluation/contact_evaluation_data.hpp>
#include <muse_armcl/evaluation/data_set_loader.hpp>
#include <muse_armcl/evaluation/msgs_conversion.hpp>
#include <cslibs_kdl_conversion/cslibs_kdl_conversion.h>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>
#include <visualization_msgs/MarkerArray.h>
#include <cslibs_mesh_map/mesh_map_tree.h>
#include <cslibs_math_ros/geometry_msgs/conversion_3d.hpp>
using namespace muse_armcl;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_armcl_test_bag_node");
    if(argc != 2) {
        std::cout << "Please run 'rosrun muse_armcl muse_armcl_test_bag_node <path-to-bag>'" << std::endl;
        return 0;
    }

    ros::NodeHandle nh("~");

    std::vector<std::string> topics = {{nh.param<std::string>("tf", "/first_tf"),
                                        nh.param<std::string>("joint", "/contact_data")}};
    rosbag::Bag     bag(argv[1]);
    rosbag::View    view(bag, rosbag::TopicQuery(topics));

    std::cout << view.getBeginTime() << "\n";

    muse_armcl::DataSet data_set;
    bool success = muse_armcl::DataSetLoader::loadFromBag(bag, topics[0], topics[1], data_set);
    std::cout << (success ? " loaded data successfully" : " loading data failed") << std::endl;

    if(!success){
        return 42;
    }
    std::string joint_state_topic  = nh.param<std::string>("joint_state_topic","/estimated_ext_torque");
    std::string topic_contacts     = nh.param<std::string>("topic_true_contact", "true_contact");
    std::string topic_contacts_vis = nh.param<std::string>("topic_true_contact_visualization", "true_contact_visualization");

    ros::Publisher pub_joint        = nh.advertise<sensor_msgs::JointState>(joint_state_topic, 1);
    ros::Publisher pub_contacts     = nh.advertise<cslibs_kdl_msgs::ContactMessage>(topic_contacts, 1);
    ros::Publisher pub_contacts_vis = nh.advertise<visualization_msgs::Marker>(topic_contacts_vis, 1);

    double contact_marker_r = nh.param<double>("contact_marker_r", 1.0);
    double contact_marker_g = nh.param<double>("contact_marker_g", 0.0);
    double contact_marker_b = nh.param<double>("contact_marker_b", 0.0);

    bool vertex_gt_model = nh.param<bool>("vertex_gt_model", true);

    std::map<int, cslibs_kdl::KDLTransformation> labeled_contact_points;
    cslibs_mesh_map::MeshMapTree tree;
    if(!vertex_gt_model){

        std::string path = nh.param<std::string>("contact_points_file", std::string(""));
        if(path != ""){
            std::vector<cslibs_kdl::KDLTransformation> labeled_contact_points;
            cslibs_kdl::load(path, labeled_contact_points);
            labeled_contact_points.clear();
            for(auto p : labeled_contact_points){
                std::string name = p.name;
                name.erase(0,1);
                int label = std::stoi(name);
                labeled_contact_points[label] = p;
            }
        }

    } else {
        std::string path                    = nh.param<std::string>("path", "");
        std::vector<std::string> files      = nh.param<std::vector<std::string>>("meshes",     std::vector<std::string>());
        std::vector<std::string> parent_ids = nh.param<std::vector<std::string>>("parent_ids", std::vector<std::string>());
        std::vector<std::string> frame_ids  = nh.param<std::vector<std::string>>("frame_ids",  std::vector<std::string>());

        tree.loadFromFile(path, parent_ids, frame_ids, files);
    }

    double msg_ratio = nh.param<double>("sample_ration", 0.5);
    double frequency = nh.param<double>("frequency", 30);
    double loop_time = nh.param<double>("loop_time", 30);
    ros::Duration loop_duration(loop_time);
    ros::Rate rate(frequency);

    visualization_msgs::Marker msg;
    msg.lifetime = ros::Duration(0.2);
    msg.color.a = 0.8;
    msg.color.r = contact_marker_r;
    msg.color.g = contact_marker_g;
    msg.color.b = contact_marker_b;
    msg.scale.x = 0.005;
    msg.scale.y = 0.01;
    msg.scale.z = 0.01;
    msg.ns = "true_contact";
    msg.type = visualization_msgs::Marker::ARROW;
    msg.action = visualization_msgs::Marker::MODIFY;
    msg.points.resize(2);
    msg.id = 0;

    ros::Time start = ros::Time::now();
    auto it = data_set.begin();
    while(ros::ok() && it !=  data_set.end()){
        ros::Time current = ros::Time::now();
        ros::Duration delta = current  - start;
        if(delta > loop_duration){
            ++it;
            if(it == data_set.end()){
                break;
            }
        }
        std::size_t sample_id = std::floor(static_cast<std::size_t>(static_cast<double>(it->data.size() -1) * msg_ratio));
        const ContactSample& cs = it->data.entry(sample_id);
        if(cs.label < 0){
            continue;
        }
        sensor_msgs::JointState state;
        cslibs_kdl::JointStateConversion::data2ros(cs.state.data, state);
        state.header.frame_id = cs.state.frameId();
        state.header.stamp = current;

        cslibs_kdl_msgs::ContactMessage contact;
        contact.header.stamp = current;
        cslibs_math_3d::Vector3d pos;
        cslibs_math_3d::Vector3d actual_dir;
        if(vertex_gt_model) {
            cslibs_mesh_map::MeshMapTreeNode* map = tree.getNode(cs.contact_force.frameId());
            contact.header.frame_id = cs.contact_force.frameId();
            if(map){
                cslibs_mesh_map::MeshMap::VertexHandle vh = map->map.vertexHandle(cs.label);
                pos = map->map.getPoint(vh);
                cslibs_math_3d::Vector3d n = map->map.getNormal(vh);
                cslibs_math_3d::Vector3d z(0,0,1);
                cslibs_math_3d::Vector3d  axis = z.cross(n);
                double alpha = std::acos(z.dot(n));
                cslibs_math_3d::Vector3d dir_local(cs.contact_force(0),
                                                   cs.contact_force(1),
                                                   cs.contact_force(2));
                dir_local = dir_local.normalized();
                cslibs_math_3d::Quaternion q(alpha, axis);
                actual_dir = q*dir_local;
                msg.header.frame_id = cs.contact_force.frameId();
            }
        } else{
            const cslibs_kdl::KDLTransformation& t = labeled_contact_points.at(cs.label);
            pos(0) = t.frame.p.x();
            pos(1) = t.frame.p.y();
            pos(2) = t.frame.p.z();
            KDL::Vector dir = t.frame.M * KDL::Vector(-1,0,0);
            actual_dir(0) = dir.x();
            actual_dir(1) = dir.y();
            actual_dir(2) = dir.z();
            msg.header.frame_id = t.parent;
        }
        contact.location = cslibs_math_ros::geometry_msgs::conversion_3d::toVector3(pos);
        contact.direction = cslibs_math_ros::geometry_msgs::conversion_3d::toVector3(actual_dir);
        contact.force = cs.contact_force.norm();

        msg.points[0] = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(pos - actual_dir * 0.2 );
        msg.points[1] = cslibs_math_ros::geometry_msgs::conversion_3d::toPoint(pos);
        msg.header.stamp = current;

        pub_joint.publish(state);
        pub_contacts.publish(contact);
        pub_contacts_vis.publish(msg);
        rate.sleep();
    }



    return 0;
}
