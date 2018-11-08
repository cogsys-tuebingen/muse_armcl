#include <muse_armcl/sampling/uniform_sampling.hpp>

#include <cslibs_mesh_map/random_walk.hpp>
#include <cslibs_math/sampling/uniform.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 UniformAllMaps : public UniformSampling
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<UniformAllMaps>;

    virtual bool update(const std::string &frame) override
    {
        return true;
    }

    virtual void apply(sample_t &sample) override
    {
        using rng_t  = cslibs_math::random::Uniform<1>;

        std::size_t map_i = 0;
        if (map_providers_.size() > 1) {
            rng_t::Ptr rng(new rng_t(0.0, map_providers_.size()));
            if (random_seed_ >= 0)
                rng.reset(new rng_t(0.0, map_providers_.size(), random_seed_));

            /// random map
            map_i = std::min(map_providers_.size()-1, static_cast<std::size_t>(rng->get()));
        }

        /// get map
        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_providers_[map_i]->getStateSpace();
        if (!ss->isType<MeshMap>())
            return false;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_t      = cslibs_mesh_map::MeshMap;
        const mesh_map_tree_t::Ptr &map = ss->as<MeshMap>().data();
        const std::vector<std::string> frame_ids = map->getFrameIds();

        /// random link
        rng_t::Ptr rng(new rng_t(0.0, frame_ids.size()));
        if (random_seed_ >= 0)
            rng.reset(new rng_t(0.0, frame_ids.size(), random_seed_));

        std::size_t link_i = std::min(frame_ids.size()-1, static_cast<std::size_t>(rng->get()));
        const mesh_map_tree_t* link = map->getNode(frame_ids[link_i]);
        const mesh_map_t& mesh = link->map_;

        /// cumsum
        const std::size_t size = mesh.numberOfEdges();
        std::vector<double> cumsum(size + 1);
        cumsum[0] = 0.0;
        std::size_t i = 0;
        for (auto edge_it = mesh.edgeBegin(); edge_it != mesh.edgeEnd(); ++edge_it, ++i)
            cumsum[i+1] = cumsum[i] + mesh.calculateEdgeLength(edge_it);

        rng.reset(new rng_t(0.0, cumsum.back()));
        if (random_seed_ >= 0)
            rng.reset(new rng_t(0.0, cumsum.back(), random_seed_));

        /// random element
        const double u = rng.get();
        i = 0;
        auto edge_it = mesh.edgeBegin();
        for (; edge_it != mesh.edgeEnd(); ++edge_it, ++i)
            if (cumsum[i] <= u && u < cumsum[i+1])
                break;

        sample.active_vertex = mesh.fromVertexHandle(edge_it);
        sample.goal_vertex = mesh.toVertexHandle(edge_it);
        sample.updateEdgeLength(mesh);
        sample.s = 0;
        sample.map_id = mesh.id_;
    }

private:
    int                               random_seed_;
    std::vector<MeshMapProvider::Ptr> map_providers_;

    virtual bool apply(sample_set_t &particle_set) override
    {
        const std::size_t n_maps = map_providers_.size();
        const std::size_t particles_per_map = sample_size_;

        for (auto &map_provider : map_providers_) {
            const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider->getStateSpace();
            if (!ss->isType<MeshMap>())
                continue;

            using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
            using mesh_map_t      = cslibs_mesh_map::MeshMap;
            const mesh_map_tree_t::Ptr &map = ss->as<MeshMap>().data();
            const std::vector<std::string> frame_ids = map->getFrameIds(); // TO IMPLEMENT!

            // TODO...
        }
    }

    using map_provider_map_t = std::map<std::string, MeshMapProvider::Ptr>;
    virtual void doSetup(const map_provider_map_t &map_providers,
                         ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        random_seed_ = nh.param(param_name("seed"), -1);

        std::vector<std::string> map_provider_ids;
        nh.getParam(param_name("maps"), map_provider_ids);

        if (map_provider_ids.size() == 0) {
            throw std::runtime_error("[NormalSampling]: No map providers were found!");
        }

        std::string ms ="[";
        for (auto m : map_provider_ids) {
            if (map_providers.find(m) == map_providers.end())
                throw std::runtime_error("[NormalSampling]: Cannot find map provider '" + m + "'!");

            map_providers_.emplace_back(map_providers.at(m));
            ms += m + ",";
        }
        ms.back() = ']';
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::UniformAllMaps, muse_armcl::NormalSampling)
