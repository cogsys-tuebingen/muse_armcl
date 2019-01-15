#include <muse_armcl/density/sample_density.hpp>

#include <unordered_map>
#include <cslibs_math/statistics/weighted_distribution.hpp>

namespace muse_armcl {
class /*EIGEN_ALIGN16*/ NearestNeighborDensity : public muse_armcl::SampleDensity
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using vertex_t               = cslibs_mesh_map::MeshMap::VertexHandle;
    using distribution_t         = cslibs_math::statistics::WeightedDistribution<3>;

    struct vertex_distribution_t /*EIGEN_ALIGN16*/ {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using allocator_t = Eigen::aligned_allocator<vertex_distribution_t>;
        using Ptr = std::shared_ptr<vertex_distribution_t>;

        vertex_t        handle;
        distribution_t  mean;
        int             cluster_id = -1;
    };

    using vertex_distributions_t = std::unordered_map<std::size_t,
                                                      std::unordered_map<int, vertex_distribution_t::Ptr>>;

    using cluster_map_t  = std::unordered_map<int,
    distribution_t,
    std::hash<int>,
    std::equal_to<int>,
    Eigen::aligned_allocator<std::pair<const int, distribution_t>>>;


    void setup(const map_provider_map_t &map_providers,
               ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        ignore_weight_ = nh.param(param_name("ignore_wieght"), false);
        radius_        = nh.param(param_name("radius"), 0.1);
        radius_ *= radius_;

        const std::string map_provider_id = nh.param<std::string>("map", ""); /// toplevel parameter
        if (map_provider_id == "")
            throw std::runtime_error("[SampleDensity]: No map provider was found!");

        if (map_providers.find(map_provider_id) == map_providers.end())
            throw std::runtime_error("[SampleDensity]: Cannot find map provider '" + map_provider_id + "'!");

        map_provider_ = map_providers.at(map_provider_id);

    }

    std::size_t histogramSize() const override
    {
        return vertex_distributions_.size();
    }

    void contacts(sample_vector_t &states) const override
    {

    }

    void clear() override
    {
        vertex_distributions_.clear();
        cluster_distributions_.clear();
    }

    void insert(const sample_t &sample) override
    {
        const double s = sample.state.s;
        const auto &goal_handle = sample.state.goal_vertex;
        const auto &acti_handle = sample.state.active_vertex;
        const int id =  s < 0.5 ? acti_handle.idx() : goal_handle.idx();
        const std::size_t map_id = sample.state.map_id;

        vertex_distribution_t::Ptr &d = vertex_distributions_[map_id][id];
        if(!d) {
            d.reset(new vertex_distribution_t);
            d->handle = s < 0.5 ? acti_handle : goal_handle;
        }
        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        const mesh_map_tree_t *map = ss->as<MeshMap>().data();
        const cslibs_math_3d::Vector3d pos = sample.state.getPosition(map->getNode(sample.state.map_id)->map);
        d->mean.add(pos.data(), ignore_weight_ ? 1.0 : sample.weight);
    }

    void estimate() override
    {

        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        const mesh_map_tree_t *map = ss->as<MeshMap>().data();
        int cluster_id = -1;
        /// clustering by map
        for(auto &vd : vertex_distributions_) {
            for(auto &v : vd.second) {
                vertex_distribution_t::Ptr &d = v.second;

                if(d->cluster_id != -1)
                    continue;

                /// check all neighbors for cluster id
                /// if d < dmax : assign to neighbor
                /// else new cluster
                const Eigen::Vector3d mean = d->mean.getMean();
                const cslibs_mesh_map::MeshMapTreeNode* state_map = map->getNode(vd.first);
                const auto& neighbors = state_map->map.getNeighbors(d->handle);
                double min_dist = std::numeric_limits<double>::max();
                int min_id = -1;
                for(const auto& n : neighbors){
                    const Eigen::Vector3d pn = state_map->map.getPoint(n);
                    double dist = (pn - mean).squaredNorm();
                    if(dist < radius_ ){
                        min_dist = dist;
                        min_id = n.idx();
                    }
                }

                if(vertex_distributions_.find(min_id) == vertex_distributions_.end())
                    min_id = -1;

                if(min_id != -1){
                    d->cluster_id = min_id;
                    distribution_t &md = cluster_distributions_[min_id];
                    md += d->mean;
                } else {
                    ++cluster_id;
                    d->cluster_id = cluster_id;
                    cluster_distributions_[cluster_id] = d->mean;
                }
            }
        }
    }

private:
    vertex_distributions_t      vertex_distributions_;
    cluster_map_t               cluster_distributions_;
    MeshMapProvider::Ptr        map_provider_;
    bool                        ignore_weight_;
    double                      radius_;

};
}

