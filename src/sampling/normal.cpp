#include <muse_armcl/sampling/normal_sampling.hpp>

#include <cslibs_mesh_map/random_walk.hpp>
#include <cslibs_math/sampling/normal.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 Normal : public NormalSampling
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Normal>;

    virtual bool apply(const state_t      &state,
                       const covariance_t &covariance,
                       sample_set_t       &sample_set) override
    {
        /// draw random samples in 3D
        using Metric = cslibs_math::sampling::Metric;
        using rng_t  = cslibs_math::sampling::Normal<Metric, Metric, Metric>;

        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return false;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        const mesh_map_tree_t::Ptr &map = ss->as<MeshMap>().data();

        /// set up random generator
        cslibs_math_3d::Vector3d start = state.getPosition(map->getNode(state.map_id)->map_);
        rng_t::Ptr rng(new rng_t(start, covariance));
        if (random_seed_ >= 0)
            rng.reset(new rng_t(start, covariance, random_seed_));

        if (sample_size_ < sample_set.getMinimumSampleSize() &&
            sample_size_ > sample_set.getMaximumSampleSize())
            throw std::runtime_error("[NormalSampling]: Initialization sample size invalid!");

        /// set up random walk
        cslibs_mesh_map::RandomWalk random_walk;
        random_walk.jump_probability_ = jump_probability_;

        /// draw samples
        sample_set_t::sample_insertion_t insertion = sample_set.getInsertion();
        const double weight = 1.0 / static_cast<double>(sample_size_);

        auto likelihood = [&start, &covariance](const cslibs_math_3d::Vector3d& p) {
            const Eigen::Matrix<double, 3, 1> q = p - start;
            const Eigen::Matrix<double, 3, 3>& cov = covariance;
            const double exp = -0.5 * static_cast<double>(
                        static_cast<Eigen::Matrix<double, 1, 3>>(q.transpose()) * cov.inverse() * q);
            static constexpr double sqrt_2_M_PI = cslibs_math::common::sqrt(2.0 * M_PI);
            const double denominator = 1.0 / (cov.determinant() * sqrt_2_M_PI);
            return denominator * std::exp(exp);
        };

        const ros::Time sampling_start = ros::Time::now();
        for (std::size_t i = 0; i < sample_size_; ++i) {
            bool valid = false;
            while (!valid) {
                /// timeout after too many tries
                if (sampling_start + sampling_timeout_ < ros::Time::now())
                    return false;

                /// estimate length
                cslibs_math_3d::Vector3d end(rng->get());
                random_walk.distance_to_travel_ = (end - start).length();
                const double end_lk = likelihood(end);

                /// do random walk by length
                state_t p = state;
                random_walk.update(p, *map);
                cslibs_math_3d::Vector3d reached = p.getPosition(map->getNode(p.map_id)->map_);

                /// check if reached point has about the same likelihood as target
                const double reached_lk = likelihood(reached);
                valid = std::fabs((end_lk - reached_lk) / end_lk) < likelihood_tolerance_;
                if (valid) {
                    sample_t s(p, weight);
                    insertion.insert(s);
                }

                /// if not, don't insert into sample set, discard, draw again...
            }
        }
        return true;
    }

    virtual bool update(const std::string &frame) override
    {
        return true;
    }

private:
    int                  random_seed_;
    double               jump_probability_;
    double               likelihood_tolerance_;
    MeshMapProvider::Ptr map_provider_;

    using map_provider_map_t = std::map<std::string, MeshMapProvider::Ptr>;
    virtual void doSetup(const map_provider_map_t &map_providers,
                         ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        random_seed_          = nh.param(param_name("seed"), -1);
        jump_probability_     = nh.param(param_name("jump_probability"), 0.3);
        likelihood_tolerance_ = nh.param(param_name("likelihood_tolerance"), 0.1);

        const std::string map_provider_id = nh.param<std::string>(param_name("map"), "");
        if (map_provider_id == "")
            throw std::runtime_error("[NormalSampling]: No map provider was found!");

        if (map_providers.find(map_provider_id) == map_providers.end())
            throw std::runtime_error("[NormalSampling]: Cannot find map provider '" + map_provider_id + "'!");

        map_provider_ = map_providers.at(map_provider_id);
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::Normal, muse_armcl::NormalSampling)
