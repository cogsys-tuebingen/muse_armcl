#include <muse_armcl/update/contact_localization_update_model.hpp>

#include <muse_armcl/state_space/mesh_map.hpp>
#include <muse_armcl/update/joint_state_data.hpp>
#include <kdl/frames.hpp>
#include <cslibs_kdl/kdl_conversion.h>
#include <nlopt.hpp>

namespace muse_armcl {
class NormalizedConeUpdateModel : public ContactLocalizationUpdateModel
{
public:
    using allocator_t = Eigen::aligned_allocator<NormalizedConeUpdateModel>;

    NormalizedConeUpdateModel():
        ContactLocalizationUpdateModel()
    {
        lower_bound_.resize(2,0);
        upper_bound_ = {M_PI, 2*M_PI};
    }

    virtual void setup(ros::NodeHandle &nh) override
    {
        ContactLocalizationUpdateModel::setup(nh);
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        theta_max_ = nh.param<double>(param_name("theta_max"), 0.1);
        upper_bound_[0] = theta_max_;

        opt_.set_lower_bounds(lower_bound_);
        opt_.set_upper_bounds(upper_bound_);
        opt_.set_min_objective(minfunc, this);


        double xtol_rel = nh.param<double>(param_name("xtol_rel"), 1e-5);
        double max_time = nh.param<double>(param_name("max_time"),1.0/15);
        opt_.set_xtol_rel(xtol_rel);
        opt_.set_maxtime(max_time);

    }

    virtual double calculateWeight(const state_t& state,
                                   const Eigen::VectorXd &tau_ext_sensed,
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

        KDL::Vector z(0,0,1);
        KDL::Vector axis = z * n;
        double alpha = std::acos(dot(z, n));
        tranform_ = KDL::Frame(KDL::Rotation::Rot(axis, alpha), p);
        jacobian_ = &jacobian.at(state.map_id);

        if(frame_id.find("finger") != std::string::npos ){
            tranform_ = transforms.at(state.map_id) *  tranform_;
        }

        std::size_t rows = jacobian_->cols();
        max_dim_ = std::min(rows, n_joints_);

        tau_sensed_ = tau_ext_sensed;
        if(tau_sensed_.norm() > 1e-5){
            tau_sensed_.normalize();
        }


        double minf;
        std::vector<double> x = {0,0};
        /*nlopt::result res =*/ opt_.optimize(x, minf);
        double result = std::exp(-0.5*minf);

        state.last_update = result;
        return result;
    }

    double objectiveFuntion(const std::vector<double>& x, std::vector<double>& grad)
    {

        const double& theta = x[0];
        const double& phi = x[1];
        KDL::Vector force_dir(std::sin(theta)*std::cos(phi),
                              std::sin(theta)*std::sin(phi),
                              std::cos(theta));
        KDL::Wrench w(force_dir, KDL::Vector::Zero());
        w = tranform_ * w;

        Eigen::VectorXd F = cslibs_kdl::convert2Eigen(w);
        Eigen::VectorXd tau_particle_local  = (*jacobian_)* F;


        Eigen ::VectorXd tau_particle(Eigen::VectorXd::Zero(max_dim_));
        for(std::size_t i= 0; i < max_dim_ ; ++i){
            tau_particle(i) = tau_particle_local(i);
        }
        double tau_particle_norm = tau_particle.norm();
        if(tau_particle_norm > 1e-5){
            tau_particle.normalize();
        }

        Eigen::VectorXd diff = tau_sensed_ - tau_particle;
        Eigen::VectorXd tmp = (diff.transpose()).eval() * info_matrix_;

        double result = tmp.dot(diff);
        if(!grad.empty()){
            grad.resize(2);
            KDL::Vector dfdtheta(  std::cos(theta)*std::cos(phi),
                                   std::cos(theta)*std::sin(phi),
                                   -std::cos(theta));
            KDL::Wrench wtheta (dfdtheta, KDL::Vector::Zero());

            KDL::Vector dfdphi( -std::sin(theta)*std::sin(phi),
                                std::sin(theta)*std::cos(phi),
                                0);
            KDL::Wrench wphi (dfdphi, KDL::Vector::Zero());

            double fac = (-tau_particle.transpose().eval() * tau_particle + 1.0)/tau_particle_norm;
            grad[0] = 2.0 * tmp.dot(fac * (*jacobian_) * cslibs_kdl::convert2Eigen(tranform_ * wtheta));
            grad[1] = 2.0 * tmp.dot(fac * (*jacobian_) * cslibs_kdl::convert2Eigen(tranform_ * wphi));

        }
        return result;
    }

    static double minfunc(const std::vector<double>& x, std::vector<double>& grad, void* data) {

        NormalizedConeUpdateModel *c = (NormalizedConeUpdateModel *) data;

        return c->objectiveFuntion(x, grad);
    }

private:
    double theta_max_;
    std::size_t max_dim_;
    const Eigen::MatrixXd* jacobian_;
    KDL::Frame tranform_;
    nlopt::opt opt_;
    std::vector<double> lower_bound_;
    std::vector<double> upper_bound_;
    Eigen::VectorXd tau_sensed_;


};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::NormalizedConeUpdateModel, muse_armcl::UpdateModel)
