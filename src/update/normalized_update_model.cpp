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
        KDL::Wrench w(n,KDL::Vector::Zero());
        KDL::Frame T(KDL::Rotation::Identity(),p);
        w = T * w;
        Eigen::VectorXd tau_p = model_.getExternalTorques(joint_state.position, frame_id, w);
        tau_p.normalize();
        Eigen::VectorXd tau_f;
        cslibs_kdl::convert(joint_state.position, tau_f,  joint_state.position.size() - model_.getNrJoints());
        tau_f.normalize();
        Eigen::VectorXd diff = tau_f - tau_p;

        double expo = (diff.transpose()).eval() * info_matrix_ * diff;
        return std::exp(-0.5*expo);

    }
    
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::NormalizedUpdateModel, muse_armcl::UpdateModel)
