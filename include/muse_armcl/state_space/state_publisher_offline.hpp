#pragma once
// ARMCL
#include <muse_armcl/state_space/state_publisher.h>
#include <muse_armcl/evaluation/confusion_matrix.hpp>
#include <muse_armcl/evaluation/detection_result.hpp>
#include <muse_armcl/evaluation/contact_evaluation_data.hpp>

#include <muse_armcl/density/sample_density.hpp>

#include <jaco2_contact_msgs/Jaco2CollisionSequence.h>
#include <jaco2_contact_msgs/Jaco2CollisionSample.h>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>

namespace muse_armcl {
class EIGEN_ALIGN16 StatePublisherOffline : public StatePublisher
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
//        std::cout << "after resampling" << std::endl;

        StatePublisher::publish(sample_set);

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;

        if (!map_provider_) {
            std::cerr << "[StatePublisherOffline]: I have no map."  << std::endl;
            return;
        }
        if(!data_) {
            std::cerr << "[StatePublisherOffline]: I have no data." << std::endl;
            return;
        }

//        std::cout << "evaluate" << std::endl;

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
        if(!data_->contains(nsecs)){
            return;
        }
        const ContactSample& gt = data_->at(nsecs);
        std::size_t sample_id = data_->getID(nsecs);
        if(gt.state.torque.empty()) {
            std::cout << nsecs << std::endl;
            throw std::runtime_error("Empty data recieved");
        }

        double tau_norm =  gt.state.norm(cslibs_kdl_data::JointStateData::DataType::JOINT_TORQUE);
        cslibs_math_3d::Vector3d actual_pos;
        cslibs_math_3d::Vector3d actual_dir;

        if(gt.label != no_collision_label_ && gt.label > 0){
            std::string actual_frame = getGroundTruthContact(gt.label, actual_pos, actual_dir);
            cslibs_math_3d::Transform3d baseTactual = map->getTranformToBase(actual_frame);
            actual_pos = baseTactual * actual_pos;
            actual_dir = baseTactual * actual_dir;
        }


        DetectionResult event;
        event.true_point = gt.label;
        event.contact_force_true = gt.contact_force.norm();
        event.error_dist = std::numeric_limits<double>::infinity();
        event.error_ori = std::numeric_limits<double>::infinity();
        event.contact_force = 0;
        event.phi = 0;
        event.s = 0;
        event.closest_point = no_collision_label_;

        for (const StateSpaceDescription::sample_t& p : states) {
            const mesh_map_tree_node_t* p_map = map->getNode(p.state.map_id);
            cslibs_math_3d::Transform3d baseTpred= map->getTranformToBase(p_map->frameId());
            if (p_map && (tau_norm > no_contact_torque_threshold_)) {
                std::string link = p_map->frameId();
                cslibs_math_3d::Vector3d point = baseTpred * p.state.getPosition(p_map->map);
                cslibs_math_3d::Vector3d direction = baseTpred * p.state.getDirection(p_map->map);
                int prediction = getClosetPoint(link, point);
                double dist_error = (point - actual_pos).length();
                double tmp = direction.dot(actual_dir) /direction.length() / actual_dir.length();
                double angle = std::acos(tmp);
                if(dist_error < event.error_dist){
                    event.contact_force = p.state.force;
                    event.closest_point = prediction;
                    event.error_dist = dist_error;
                    event.error_ori = angle;
                }
            }
        }
//        std::cout << "[" << nsecs << "]" << "(" << sample_id << ")" << " torque res: " << tau_norm << " gt label: "<< gt.label
//                  << " detected: " << event.closest_point << std::endl;
        confusion_matrix_.reportClassification(gt.label, event.closest_point);
        results_.push_back(event);

        set_time_(sample_set->getStamp());

    }
    virtual void publishIntermediate(const typename sample_set_t::ConstPtr &sample_set) override
    {
//        std::cout << "intermediate" << "\n";
//        StatePublisher::publishIntermediate(sample_set);
        publish(sample_set);
//        set_time_(sample_set->getStamp());
    }
    virtual void publishConstant(const typename sample_set_t::ConstPtr &sample_set) override
    {
//        std::cout << "constant" << "\n";
//        StatePublisher::publishConstant(sample_set);
        publish(sample_set);
//        set_time_(sample_set->getStamp());
    }

    void setData(const ContactSequence& data)
    {
        data_ = &data;
    }

    void reset()
    {
        data_ = nullptr;
    }

      int getClosetPoint(const std::string& frame_id, const cslibs_math_3d::Vector3d& estimated) const
    {
        KDL::Vector pos (estimated(0), estimated(1), estimated(2));
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
//        std::cout << "get point" << std::endl;
        return min.first;
    }

    std::string getGroundTruthContact(int label, cslibs_math_3d::Vector3d& position, cslibs_math_3d::Vector3d& direction) const
    {
        std::string point_name = "p" + std::to_string(label);
        for(const cslibs_kdl::KDLTransformation t : labeled_contact_points_)
        {
            if(point_name == t.name){
                position(0) = t.frame.p.x();
                position(1) = t.frame.p.y();
                position(2) = t.frame.p.z();
                KDL::Vector dir = t.frame.M * KDL::Vector(-1,0,0);
                direction(0) = dir.x();
                direction(1) = dir.y();
                direction(2) = dir.z();
                return t.parent;
            }
        }
        throw std::runtime_error("Cannot find point with label " + std::to_string(label));
    }

    void exportResults(const std::string& path)
    {
        std::string file_cm = path + "confusion_matrix.csv";
        std::string file_ds = path + "detection_results.csv";
        confusion_matrix_.exportCsv(file_cm);
        save(results_, file_ds);
    }

    void getLikelyHoodOfGt(const typename sample_set_t::ConstPtr &sample_set, const ContactSample& gt,
                           const cslibs_mesh_map::MeshMapTree* map)
    {
//        cslibs_kdl::KDLTransformation true_contact_point = getLabledPoint(gt.label);
//        cslibs_math_3d::Transform3d T = map->getTranformToBase(true_contact_point.frame);
////        cslibs_kdl::convert()

//        for (const StateSpaceDescription::sample_t& p : sample_set->getSamples()) {
//            const slibs_mesh_map::MeshMapTree* p_map = map->getNode(p.state.map_id);
//            if (p_map) {
//                cslibs_math_3d::Transform3d T = map->getTranformToBase(p_map->map.frame_id_);
//                cslibs_math_3d::Point3d pos = p.state.getPosition(p_map->map);
//                pos = T * pos;
//            }
//        }
    }

    const cslibs_kdl::KDLTransformation& getLabledPoint(int label) const
    {
        std::string point_name = "p" + std::to_string(label);
        for(const cslibs_kdl::KDLTransformation t : labeled_contact_points_){
            if(point_name == t.name){
                return t;
            }
        }
        throw std::runtime_error("Cannot find point with label " + std::to_string(label));
    }


private:
    time_callback_t     set_time_;
    const ContactSequence* data_;
    std::vector<cslibs_kdl::KDLTransformation> labeled_contact_points_;
    ConfusionMatrix confusion_matrix_;
    int no_collision_label_;
    std::vector<DetectionResult> results_;

};
}
