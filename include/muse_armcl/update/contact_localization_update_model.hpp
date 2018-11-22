#ifndef CONTACT_LOCALIZATION_UPDATE_MODEL_HPP
#define CONTACT_LOCALIZATION_UPDATE_MODEL_HPP

#include <muse_armcl/update/update_model.hpp>
#include <muse_armcl/state_space/mesh_map.hpp>
#include <muse_armcl/update/joint_state_data.hpp>

#include <cslibs_kdl/external_forces.h>

namespace muse_armcl {
class EIGEN_ALIGN16 ContactLocalizationUpdateModel : public muse_armcl::UpdateModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ContactLocalizationUpdateModel>;
    virtual void apply(const typename data_t::ConstPtr          &data,
                       const typename state_space_t::ConstPtr   &ss,
                       typename sample_set_t::weight_iterator_t  set) override
    {
        if (!ss->isType<MeshMap>() || !data->isType<JointStateData>())
            return;

        /// cast map to specific type
        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;
//        using mesh_map_t      = cslibs_mesh_map::MeshMap;
        const mesh_map_tree_t* map = ss->as<MeshMap>().data();

        /// cast data to specific type
        const JointStateData &joint_states = data->as<JointStateData>();
//        ROS_INFO_STREAM("update: now - recived " <<  (ros::Time::now().toNSec() - joint_states.stampReceived().nanoseconds()) *1e-6 << "ms");
        // -> access data via joint_states.position, joint_states.velocity, ...

        // update transformations in the map
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
        }



        for(auto it = set.begin() ; it != set.end() ; ++it) {
            /// access particle
            const state_t& state = it.state();

            /// apply estimated weight on particle
            *it *= calculateWeight(state, joint_states, map);
        }
    }

    virtual double calculateWeight(const state_t& state,
                                   const JointStateData& joint_state,
                                   const cslibs_mesh_map::MeshMapTree* map) = 0;

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
        std::size_t nj = model_.getNrJoints();

        std::vector<double> info_default(nj, 0.5);
        info_values_ = nh.param<std::vector<double>>(param_name("information_matrix"), info_default);


        info_matrix_.setZero(nj, nj);
        if(info_values_.size() >= nj*nj){
             for(std::size_t i = 0; i < nj; ++i){
                  for(std::size_t j = 0; j < nj; ++j){
                      std::size_t index = i*nj+j;
                      info_matrix_(i,j) = info_values_[index];
                  }
             }
        } else{
            for(std::size_t i = 0; i < nj; ++i){
                if(i <info_values_.size()){
                    info_matrix_(i,i) = info_values_[i];
                } else {
                    info_matrix_(i,i) = info_values_.back();
                }
            }

        }
        ROS_INFO_STREAM("Information matrix: \n"<< info_matrix_);
    }

protected:
    cslibs_kdl::ExternalForcesSerialChain model_;
    std::vector<double> info_values_;
    Eigen::MatrixXd info_matrix_;

};
}
#endif // CONTACT_LOCALIZATION_UPDATE_MODEL_HPP
