#include <muse_armcl/update/contact_localization_update_model.hpp>

#include <muse_armcl/state_space/mesh_map.hpp>
#include <muse_armcl/update/joint_state_data.hpp>
#include <kdl/frames.hpp>
#include <cslibs_kdl/kdl_conversion.h>

namespace muse_armcl {
class EIGEN_ALIGN16  NormalizedUpdateModel : public ContactLocalizationUpdateModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<NormalizedUpdateModel>;
    virtual double calculateWeight(const state_t &state, const JointStateData &joint_state, const cslibs_mesh_map::MeshMapTree *maps) override
    {
        const cslibs_mesh_map::MeshMapTree* particle_map = maps->getNode(state.map_id);
        const cslibs_mesh_map::MeshMap& map = particle_map->map_;
        std::string frame_id = map.frame_id_;
        cslibs_math_3d::Vector3d pos = state.getPosition(map);
        cslibs_math_3d::Vector3d normal = state.getNormal(map);
        KDL::Vector n(normal(0), normal(1), normal(2));
        KDL::Vector p(pos(0), pos(1), pos(2));
        KDL::Wrench w(-n,KDL::Vector::Zero());
//        ROS_INFO_STREAM("p: " << p.x() << ", " <<p.y() << ", " << p.z());
//        ROS_INFO_STREAM("n: " << n.x() << ", " <<n.y() << ", " << n.z());
        KDL::Frame T(KDL::Rotation::Identity(),p);
        w = T * w;
        Eigen::VectorXd tau_p = model_.getExternalTorques(joint_state.position, frame_id, w);
        std::size_t rows = tau_p.rows();
        std::size_t n_torques = joint_state.effort.size();
        std::size_t offset = 0;
        std::size_t dim = std::min(rows, n_torques);
        if(rows > n_torques){
          Eigen ::VectorXd tau_pp = Eigen::VectorXd::Zero(dim);
          for (std::size_t i = 0; i < dim; ++i){
            tau_pp(i) = tau_p(i);
          }
          tau_p = tau_pp;
        } else {
          offset = n_torques - rows;
        }
//        std::cout <<" rows " << rows << " | offset " << offset << " | dim " << dim << " | nt " << n_torques << std::endl;
        Eigen::VectorXd tau_f;
        cslibs_kdl::convert(joint_state.effort, tau_f, offset);
//        ROS_INFO_STREAM("particle torque: " << tau_p << " sensed torque: " << tau_f);
        tau_p.normalize();
        tau_f.normalize();
        Eigen::VectorXd diff = tau_f - tau_p;
//        ROS_INFO_STREAM("difference: "<< diff);

        double expo = (diff.transpose()).eval() * info_matrix_.block(0,0,dim,dim) * diff;
        double result = std::exp(-0.5*expo);
//        ROS_INFO_STREAM("weight of particle: " << result << " exponent: " << expo);
//        if(result < 0.2){
//          ROS_INFO_STREAM("small weight");
//        }
        return result;
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::NormalizedUpdateModel, muse_armcl::UpdateModel)
