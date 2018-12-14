#pragma once
// ARMCL
#include <muse_armcl/state_space/state_publisher.h>
#include <muse_armcl/state_space/confusion_matrix.hpp>
#include <muse_armcl/state_space/detection_result.hpp>
#include <muse_armcl/density/sample_density.hpp>

#include <jaco2_contact_msgs/Jaco2CollisionSequence.h>
#include <jaco2_contact_msgs/Jaco2CollisionSample.h>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>
#include <eigen_conversions/eigen_kdl.h>

namespace muse_armcl {
class /*EIGEN_ALIGN16*/ StatePublisherOffline : public StatePublisher
{
public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<StatePublisherOffline>;

    using Ptr                = std::shared_ptr<StatePublisherOffline>;

    /// callback_t
    using time_callback_t = cslibs_utility::common::delegate<void(const cslibs_time::Time &t)>;


    ///  weight_iterator_t::notify_touch::template    from<sample_set_t, &sample_set_t::weightStatisticReset>(this)
    void setup(ros::NodeHandle &nh, map_provider_map_t &map_providers, time_callback_t &set_time)
    {
        StatePublisher::setup(nh, map_providers);
        std::string path;
        path = nh.param<std::string>("contact_points_file", std::string(""));
        if(path != ""){
            cslibs_kdl::load(path, labeled_contact_points_);
        }
        no_collision_label_ = nh.param<int>("no_collision_label", -1);

        set_time_ = set_time;
    }

    virtual void publish(const typename sample_set_t::ConstPtr &sample_set) override
    {
        StatePublisher::publish(sample_set);

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;

        if (!map_provider_)
            return;

        /// get the map
        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;
        const mesh_map_tree_t* map = ss->as<MeshMap>().data();


        /// density estimation
        SampleDensity::ConstPtr density = std::dynamic_pointer_cast<SampleDensity const>(sample_set->getDensity());
        std::vector<StateSpaceDescription::sample_t, StateSpaceDescription::sample_t::allocator_t> states;
        density->contacts(states);

        // ground truth data
        uint64_t nsecs = static_cast<uint64_t>(sample_set->getStamp().nanoseconds());
        const jaco2_contact_msgs::Jaco2CollisionSample& gt = data_[nsecs];
        cslibs_math_3d::Vector3d actual_pos;
        cslibs_math_3d::Vector3d actual_dir;
        std::string actual_frame = getGroundTruthContact(gt.label, actual_pos, actual_dir);

        cslibs_math_3d::Transform3d baseTactual = map->getTranformToBase(actual_frame);
        actual_pos = baseTactual * actual_pos;
        actual_dir = baseTactual * actual_dir;

        tf::Vector3 force;
        tf::vector3MsgToTF(gt.contact_force.vector, force);
        DetectionResult event;
        event.true_point = gt.label;
        event.contact_force_true = force.length();
        event.error_dist = std::numeric_limits<double>::infinity();
        event.error_ori = std::numeric_limits<double>::infinity();
        event.contact_force = 0;
        event.phi = 0;
        event.s = 0;
        event.closest_point = no_collision_label_;

        for (const StateSpaceDescription::sample_t& p : states) {
            const mesh_map_tree_node_t* p_map = map->getNode(p.state.map_id);
            cslibs_math_3d::Transform3d baseTpred= map->getTranformToBase(p_map->frameId());
            if (p_map && p.state.force > no_contact_force_threshold_) {
                std::string link = p_map->frameId();
                cslibs_math_3d::Vector3d point = baseTpred * p.state.getPosition(p_map->map);
                cslibs_math_3d::Vector3d direction = baseTpred * p.state.getDirection(p_map->map);
                int prediction = getClosetPoint(link, point);
                double dist_error = (point - actual_pos).length();
                double tmp = direction.dot(actual_dir) /direction.length() / actual_dir.length();
                double angle = std::acos(tmp);
                if(dist_error < event.error_dist){
                    event.closest_point = prediction;
                    event.error_dist = dist_error;
                    event.error_ori = angle;
                }
            }
        }
        confusion_matrix_.reportClassification(gt.label, event.closest_point);
        results_.push_back(event);

        set_time_(sample_set->getStamp());

    }
    virtual void publishIntermediate(const typename sample_set_t::ConstPtr &sample_set) override
    {
        StatePublisher::publishIntermediate(sample_set);
        set_time_(sample_set->getStamp());
    }
    virtual void publishConstant(const typename sample_set_t::ConstPtr &sample_set) override
    {
        StatePublisher::publishConstant(sample_set);
        set_time_(sample_set->getStamp());
    }

    void setData(const jaco2_contact_msgs::Jaco2CollisionSequence& data)
    {
        for(const jaco2_contact_msgs::Jaco2CollisionSample& s : data.data){
            data_[s.header.stamp.toNSec()] = s;
        }
    }

    int getClosetPoint(const std::string& frame_id, const cslibs_math_3d::Vector3d& estimated) const
    {
        KDL::Vector pos;
        tf::vectorEigenToKDL(estimated.data(),pos);
        std::pair<int,double> min;
        min.first = -1;
        min.second = std::numeric_limits<double>::infinity();
        for(const cslibs_kdl::KDLTransformation t : labeled_contact_points_){
            if(t.parent == frame_id){
                std::pair<int,double> p;
                std::string name = t.name;
                name.erase(0,1);
                int label = std::stoi(name);
                double dist = (t.frame.p - pos).Norm();
                if(dist < min.second){
                    min.first = label;
                    min.second = dist;
                }
            }
        }
        return min.first;
    }

    std::string getGroundTruthContact(int label, cslibs_math_3d::Vector3d& position, cslibs_math_3d::Vector3d& direction) const
    {
        std::string point_name = "p" + std::to_string(label);
        for(const cslibs_kdl::KDLTransformation t : labeled_contact_points_)
        {
            if(point_name == t.parent){
                tf::vectorKDLToEigen(t.frame.p, position.data());
                KDL::Vector dir = t.frame.M * KDL::Vector(-1,0,0);
                tf::vectorKDLToEigen(dir, direction);
                return t.parent;
            }
        }
        throw std::runtime_error("Cannot find point with label " + std::to_string(label));
    }


private:
    time_callback_t     set_time_;
    std::map<uint64_t, jaco2_contact_msgs::Jaco2CollisionSample> data_;
    std::vector<cslibs_kdl::KDLTransformation> labeled_contact_points_;
    ConfusionMatrix confusion_matrix_;
    int no_collision_label_;
    std::vector<DetectionResult> results_;

};
}
