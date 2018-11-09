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

        /// get map
        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_t      = cslibs_mesh_map::MeshMap;
        const mesh_map_tree_t::Ptr &map = ss->as<MeshMap>().data();
        std::vector<std::string> frame_ids;
        map->getFrameIds(frame_ids);

        /// random link
        rng_t::Ptr rng(new rng_t(0.0, frame_ids.size()));
        if (random_seed_ >= 0)
            rng.reset(new rng_t(0.0, frame_ids.size(), random_seed_));

        std::size_t link_i = std::min(frame_ids.size()-1, static_cast<std::size_t>(rng->get()));
        mesh_map_tree_t* link = map->getNode(frame_ids[link_i]);
        mesh_map_t& mesh = link->map_;

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
        const double u = rng->get();
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
    int                  random_seed_;
    MeshMapProvider::Ptr map_provider_;

    virtual bool apply(sample_set_t &sample_set) override
    {
        sample_set_t::sample_insertion_t insertion = sample_set.getInsertion();
        const double weight = 1.0 / static_cast<double>(sample_size_);

        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return false;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_t      = cslibs_mesh_map::MeshMap;
        const mesh_map_tree_t::Ptr &map = ss->as<MeshMap>().data();
        std::vector<std::string> frame_ids;
        map->getFrameIds(frame_ids);

        const std::size_t particles_per_frame = static_cast<std::size_t>(
                    std::round(static_cast<double>(sample_size_) / static_cast<double>(frame_ids.size())));

        /// uniform over all links
        for (const auto &frame_id : frame_ids) {
            mesh_map_tree_t* link = map->getNode(frame_id);
            mesh_map_t& mesh = link->map_;
            const double edges_length = mesh.sumEdgeLength();

            /// prepare ordered sequence of random numbers
            cslibs_math::random::Uniform<1> rng(0.0, 1.0);
            std::vector<double> u(particles_per_frame, std::pow(rng.get(), 1.0 / static_cast<double>(particles_per_frame)));
            for (std::size_t k = particles_per_frame - 1; k > 0; --k) {
                const double _u = std::pow(rng.get(), 1.0 / static_cast<double>(k));
                u[k-1] = u[k] * _u;
            }

            /// draw samples
            auto edge_it = mesh.edgeBegin();
            double cumsum_last = 0.0;
            double cumsum = mesh.calculateEdgeLength(edge_it)/edges_length;
            for (auto &u_r : u) {
                while (u_r < cumsum_last) {
                    ++edge_it;
                    cumsum_last = cumsum;
                    cumsum += mesh.calculateEdgeLength(edge_it)/edges_length;
                }

                /// insert sample
                sample_t p;
                p.active_vertex = mesh.fromVertexHandle(edge_it);
                p.goal_vertex = mesh.toVertexHandle(edge_it);
                p.updateEdgeLength(mesh);
                p.s = 0;
                p.map_id = mesh.id_;
                p.weight = weight;
                insertion.insert(p);
            }
        }
        return true;
    }

    using map_provider_map_t = std::map<std::string, MeshMapProvider::Ptr>;
    virtual void doSetup(const map_provider_map_t &map_providers,
                         ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        random_seed_ = nh.param(param_name("seed"), -1);

        const std::string map_provider_id = nh.param<std::string>(param_name("map"), "");
        if (map_provider_id == "")
            throw std::runtime_error("[UniformSampling]: No map provider was found!");

        if (map_providers.find(map_provider_id) == map_providers.end())
            throw std::runtime_error("[UniformSampling]: Cannot find map provider '" + map_provider_id + "'!");

        map_provider_ = map_providers.at(map_provider_id);
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::UniformAllMaps, muse_armcl::UniformSampling)
