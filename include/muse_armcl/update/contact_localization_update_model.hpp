#ifndef CONTACT_LOCALIZATION_UPDATE_MODEL_HPP
#define CONTACT_LOCALIZATION_UPDATE_MODEL_HPP

#include <muse_armcl/update/update_model.hpp>
#include <muse_armcl/state_space/mesh_map.hpp>
#include <muse_armcl/update/joint_state_data.hpp>

#include <cslibs_kdl/external_forces.h>

namespace muse_armcl {
class ContactLocalizationUpdateModel : public muse_armcl::UpdateModel
{
public:
    virtual void apply(const typename data_t::ConstPtr          &data,
                       const typename state_space_t::ConstPtr   &ss,
                       typename sample_set_t::weight_iterator_t  set) override
    {
        if (!ss->isType<MeshMap>() || !data->isType<JointStateData>())
            return;

        /// cast map to specific type
        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_t      = cslibs_mesh_map::MeshMap;
        const mesh_map_tree_t::Ptr &map = ss->as<MeshMap>().data();

        /// cast data to specific type
        const JointStateData &joint_states = data->as<JointStateData>();
        // -> access data via joint_states.position, joint_states.velocity, ...

        for(auto it = set.begin() ; it != set.end() ; ++it) {
            /// access particle
            const state_t& state = it.state();

            /// apply estimated weight on particle
            *it *= calculateWeight(state, joint_states, map);
        }
    }

    virtual double calculateWeight(const state_t& state,
                                   const JointStateData& joint_state,
                                   const cslibs_mesh_map::MeshMapTree::Ptr& map) = 0;

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
        measurment_noise_ = nh.param<double>(param_name("measurment_noise"), 0.1);

        model_.setModel(robot_model,
                        chain_root,
                        chain_tip,
                        chain_tip_f1,
                        chain_tip_f2,
                        chain_tip_f3);

        model_.initialize();
        std::size_t nj = model_.getNrJoints();
        info_matrix_.setZero(nj, nj);
        for(std::size_t i = 0; i < nj; ++i){
            info_matrix_(i,i) = 1.0/measurment_noise_;
        }
    }

protected:
    cslibs_kdl::ExternalForcesSerialChain model_;
    double measurment_noise_;
    Eigen::MatrixXd info_matrix_;

};
}
#endif // CONTACT_LOCALIZATION_UPDATE_MODEL_HPP
