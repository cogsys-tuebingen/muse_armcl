#include <muse_armcl/prediction/prediction_model.hpp>
#include <muse_armcl/state_space/mesh_map.hpp>

#include <cslibs_mesh_map/random_walk.hpp>
#include <cslibs_math/random/random.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 RandomWalk : public PredictionModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<RandomWalk>;
    using data_t = cslibs_plugins_data::Data;
    using time_t = cslibs_time::Time;
    using rng_t  = cslibs_math::random::Uniform<1>;

    virtual void setup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        random_seed_      = nh.param(param_name("seed"), -1);
        min_distance_     = nh.param(param_name("min_distance"), 0.0);
        max_distance_     = nh.param(param_name("max_distance"), 0.1);
        jump_probability_ = nh.param(param_name("jump_probability"), 0.3);
    }

    virtual Result::Ptr apply(const data_t::ConstPtr         &data,
                              const cslibs_time::Time        &until,
                              sample_set_t::state_iterator_t  states) override
    {
        std::cerr << "[PredictionModel]: Model called without map!" << std::endl;
        return Result::Ptr(new Result(data));
    }

    virtual Result::Ptr apply(const data_t::ConstPtr                 &data,
                              const typename state_space_t::ConstPtr &state_space,
                              const cslibs_time::Time                &until,
                              sample_set_t::state_iterator_t          states) override
    {
        if (!state_space->isType<MeshMap>())
            return false;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        const mesh_map_tree_t::Ptr &map = state_space->as<MeshMap>().data();

        if (!rng_) {
            rng_.reset(random_seed_ >= 0 ?
                           new rng_t(min_distance_, max_distance_, random_seed_) :
                           new rng_t(min_distance_, max_distance_));
        } else
            rng_->set(min_distance_, max_distance_);

        /// execute random walk for all particles
        /// use random step width between given min_distance and max_distance
        random_walk_.jump_probability_ = jump_probability_;
        for (sample_t &sample : states)
            random_walk_.update(sample, *map, rng_->get());
        return Result::Ptr(new Result(data));
    }

private:
    int                         random_seed_;
    double                      min_distance_;
    double                      max_distance_;
    double                      jump_probability_;

    rng_t::Ptr                  rng_;
    cslibs_mesh_map::RandomWalk random_walk_;
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::RandomWalk, muse_armcl::PredictionModel)
