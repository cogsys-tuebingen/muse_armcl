#include <muse_armcl/update/contact_localization_update_model.hpp>

#include <muse_armcl/state_space/mesh_map.hpp>
#include <muse_armcl/update/joint_state_data.hpp>
#include <kdl/frames.hpp>
#include <cslibs_kdl/kdl_conversion.h>

namespace muse_armcl {
class NormalizedUpdateModel : public ContactLocalizationUpdateModel
{
public:
    using allocator_t = Eigen::aligned_allocator<NormalizedUpdateModel>;
    virtual double calculateWeight(const state_t&state,
                                   const JointStateData &joint_state,
                                   const cslibs_mesh_map::MeshMapTree *maps,
                                   const std::map<std::size_t, Eigen::MatrixXd>& jacobian,
                                   const std::map<std::size_t, KDL::Frame>& transforms) override
    {
        const cslibs_mesh_map::MeshMapTreeNode* particle_map = maps->getNode(state.map_id);
        const cslibs_mesh_map::MeshMap& map = particle_map->map;
        std::string frame_id = map.frame_id_;
        cslibs_math_3d::Vector3d pos = state.getPosition(map);
        cslibs_math_3d::Vector3d normal = state.getNormal(map);
        KDL::Vector n(normal(0), normal(1), normal(2));
        KDL::Vector p(pos(0), pos(1), pos(2));
        KDL::Wrench w = cslibs_kdl::ExternalForcesSerialChain::createWrench(p, n);

//        Eigen::VectorXd tau_particle_local = model_.getExternalTorques(joint_state.position, frame_id, w);
        if(frame_id.find("finger") != std::string::npos ){
            w = transforms.at(state.map_id) *  w;
        }
        Eigen::VectorXd F = cslibs_kdl::convert2Eigen(w);
        Eigen::VectorXd tau_particle_local  = jacobian.at(state.map_id) * F;

        std::size_t rows = tau_particle_local.rows();
        int n_torques = joint_state.effort.size();
        std::size_t offset = static_cast<std::size_t>(std::max(0, n_torques - static_cast<int>(n_joints_)));
        std::size_t max_dim = std::min(rows, n_joints_);

        Eigen ::VectorXd tau_particle(Eigen::VectorXd::Zero(n_joints_));
        for(std::size_t i= 0; i < max_dim ; ++i){
            tau_particle(i) = tau_particle_local(i);
        }
        Eigen::VectorXd tau_sensed;
        cslibs_kdl::convert(joint_state.effort, tau_sensed, offset);
        if(tau_sensed.norm() > 1e-5){
            tau_sensed.normalize();
        }
        if(tau_particle.norm() > 1e-5){
            tau_particle.normalize();
        }
//        std::cout << "torque part: \n"<< tau_particle << std::endl;
        Eigen::VectorXd diff = tau_sensed - tau_particle;

        double expo = (diff.transpose()).eval() * info_matrix_ * diff;
//        double result = normalizer_ * std::exp(-0.5*expo);
        double result = std::exp(-0.5*expo);

//        std::cout << "weight of particle: " << result << " exponent: " << expo << std::endl;
//        if(result < 0.2){
//          ROS_INFO_STREAM("small weight");
//        }
        state.last_update = result;
        return result;
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::NormalizedUpdateModel, muse_armcl::UpdateModel)
