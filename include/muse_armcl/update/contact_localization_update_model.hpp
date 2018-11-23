#ifndef CONTACT_LOCALIZATION_UPDATE_MODEL_HPP
#define CONTACT_LOCALIZATION_UPDATE_MODEL_HPP

#include <muse_armcl/update/update_model.hpp>
#include <muse_armcl/state_space/mesh_map.hpp>
#include <muse_armcl/update/joint_state_data.hpp>

#include <cslibs_kdl/external_forces.h>

namespace muse_armcl {
class /*EIGEN_ALIGN16*/ ContactLocalizationUpdateModel : public muse_armcl::UpdateModel
{
public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ContactLocalizationUpdateModel>;

    virtual void apply(const typename data_t::ConstPtr          &data,
                       const typename state_space_t::ConstPtr   &ss,
                       typename sample_set_t::weight_iterator_t  set) override
    {
        if (!ss->isType<MeshMap>() || !data->isType<JointStateData>())
            return;
        ros::Time start = ros::Time::now();
        /// cast map to specific type
        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;
//        using mesh_map_t      = cslibs_mesh_map::MeshMap;
        const mesh_map_tree_t* map = ss->as<MeshMap>().data();

        /// cast data to specific type
        const JointStateData &joint_states = data->as<JointStateData>();
//        ROS_INFO_STREAM("update: now - recived " <<  (ros::Time::now().toNSec() - joint_states.stampReceived().nanoseconds()) *1e-6 << "ms");
        // -> access data via joint_states.position, joint_states.velocity, ...

        // update transformations in the map and derive jacobians
        std::map<std::size_t, Eigen::MatrixXd> jacobians;
        std::map<std::size_t, KDL::Frame> transforms;
        for(const mesh_map_tree_node_t::Ptr& partial_map : *map){
            std::string frame_id = partial_map->frameId();
            std::string parent;
            if(!partial_map->parentFrameId(parent)){
                parent = frame_id;
            }
            KDL::Frame p_T_li = model_.getFKPose(joint_states.position, parent, frame_id);
            cslibs_math_3d::Vector3d trans(p_T_li.p.x(),p_T_li.p.y(), p_T_li.p.z());
            double x,y,z,w;
            p_T_li.M.GetQuaternion(x,y,z,w);
            cslibs_math_3d::Quaternion q(x,y,z,w);
            cslibs_math_3d::Transform3d transform(trans,q);
            partial_map->update(transform);
            Eigen::MatrixXd jac;
            model_.getGeometricJacobianTransposed(joint_states.position, frame_id, jac);
            std::size_t map_id = partial_map->mapId();
            jacobians[map_id] = jac;
            transforms[map_id] = p_T_li;
        }



        for(auto it = set.begin() ; it != set.end() ; ++it) {
            /// access particle
            const state_t& state = it.state();

            /// apply estimated weight on particle
            *it *= calculateWeight(state, joint_states, map, jacobians, transforms);
        }
        std::cout << "update done; took: " << (ros::Time::now() - start).toNSec() * 1e-6 << "ms\n";
    }

    virtual double calculateWeight(const state_t& state,
                                   const JointStateData& joint_state,
                                   const cslibs_mesh_map::MeshMapTree* map,
                                   const std::map<std::size_t, Eigen::MatrixXd>& jacobianans,
                                   const std::map<std::size_t, KDL::Frame>& transforms) = 0;

    virtual void setup(ros::NodeHandle &nh) override
    {
        /// TODO: if TF is needed, modify setup method in update model interface and pass via muse_armcl_node

        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        /// TODO: load parameters with nh.param(param_name(<name>), <default>);
        std::string robot_model = nh.param<std::string>(param_name("robot_description"), "robot_description");
        std::string chain_root = nh.param<std::string>(param_name("chain_root"), "jaco_link_base");
        std::string chain_tip = nh.param<std::string>(param_name("chain_tip"), "jaco_link_hand");
        std::string chain_tip_f1 = nh.param<std::string>(param_name("finger_1_tip"), "jaco_finger_1_tip");
        std::string chain_tip_f2 = nh.param<std::string>(param_name("finger_2_tip"), "jaco_finger_2_tip");
        std::string chain_tip_f3 = nh.param<std::string>(param_name("finger_3_tip"), "jaco_finger_3_tip");
        model_.setModel(robot_model,
                        chain_root,
                        chain_tip,
                        chain_tip_f1,
                        chain_tip_f2,
                        chain_tip_f3);
        model_.initialize();
        n_joints_ = model_.getNrJoints();

        std::vector<double> info_default(n_joints_, 0.5);
        info_values_ = nh.param<std::vector<double>>(param_name("information_matrix"), info_default);


        info_matrix_.setZero(n_joints_, n_joints_);
        if(info_values_.size() >= n_joints_*n_joints_){
             for(std::size_t i = 0; i < n_joints_; ++i){
                  for(std::size_t j = 0; j < n_joints_; ++j){
                      std::size_t index = i * n_joints_ + j;
                      info_matrix_(i,j) = info_values_[index];
                  }
             }
        } else{
            for(std::size_t i = 0; i < n_joints_; ++i){
                if(i <info_values_.size()){
                    info_matrix_(i,i) = info_values_[i];
                } else {
                    info_matrix_(i,i) = info_values_.back();
                }
            }

        }
        double sigma = info_matrix_.determinant();
        normalizer_ = 1.0 / (2.0 * M_PI * std::sqrt(2.0 * M_PI * sigma));

        ROS_INFO_STREAM("Information matrix: \n"<< info_matrix_);
    }

protected:
    cslibs_kdl::ExternalForcesSerialChain model_;
    std::vector<double> info_values_;
    Eigen::MatrixXd info_matrix_;
    double normalizer_;
    std::size_t n_joints_;

};
}
#endif // CONTACT_LOCALIZATION_UPDATE_MODEL_HPP
