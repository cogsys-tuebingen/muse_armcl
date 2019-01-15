#include <muse_armcl/density/sample_density.hpp>

#include <unordered_map>
#include <cslibs_math/statistics/weighted_distribution.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 NearestNeighborDensity : public muse_armcl::SampleDensity
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using vertex_t               = cslibs_mesh_map::MeshMap::VertexHandle;
    using distribution_t         = cslibs_math::statistics::WeightedDistribution<3>;

    struct EIGEN_ALIGN16 vertex_distribution
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using allocator_t = Eigen::aligned_allocator<vertex_distribution>;
        using Ptr = std::shared_ptr<vertex_distribution>;

        vertex_t                        handle;
        distribution_t                  distribution;
        int                             cluster_id = -1;
        std::vector<sample_t const*>    samples;
    };

    struct EIGEN_ALIGN16 cluster_distribution
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using allocator_t = Eigen::aligned_allocator<cluster_distribution>;
        using Ptr = std::shared_ptr<cluster_distribution>;

        distribution_t                  distribution;
        std::vector<sample_t const*>    samples;
    };


    using vertex_distributions_t = std::unordered_map<std::size_t,
                                                      std::unordered_map<int, vertex_distribution::Ptr>>;

    using cluster_map_t          = std::unordered_map<int, cluster_distribution::Ptr>;


    void setup(const map_provider_map_t &map_providers,
               ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        ignore_weight_ = nh.param(param_name("ignore_wieght"), false);
        radius_        = nh.param(param_name("radius"), 0.1);
        radius_ *= radius_;
        relative_weight_threshold_ = nh.param(param_name("relative_weight_threshold"), 0.8);

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
        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        const mesh_map_tree_t *map = ss->as<MeshMap>().data();

        auto get_nearest = [&map](const cluster_distribution &c)
        {
            double min_distance = std::numeric_limits<double>::max();
            sample_t const * sample = nullptr;
            for(const sample_t* s : c.samples) {
                const Eigen::Vector3d mean = c.distribution.getMean();
                const Eigen::Vector3d pos =  s->state.getPosition(map->getNode(sample->state.map_id)->map);
                const double distance = (mean - pos).squaredNorm();
                if(distance < min_distance) {
                    min_distance = distance;
                    sample = s;
                }
            }
            return sample;
        };

        double mean_weight = 0;
        for(auto &c : clusters_) {
            const cluster_distribution::Ptr &d = c.second;
            mean_weight += d->distribution.getWeight();
        }
        mean_weight /= static_cast<double>(clusters_.size());

        /// iterate the clusters and find the point nearest to the mean to estimate the contact
        ///  mean cluster weights ...
        for(auto &c : clusters_) {
            /// drop a cluster if it is not weighted high enough compared to the others
            if(c.second->distribution.getWeight() * relative_weight_threshold_ < mean_weight)
                continue;

            sample_t const * s = get_nearest(*c.second);
            if(s != nullptr) {
                states.emplace_back(*s);
            }
        }
    }

    void clear() override
    {
        vertex_distributions_.clear();
        clusters_.clear();
    }

    void insert(const sample_t &sample) override
    {
        const double s = sample.state.s;
        const auto &goal_handle = sample.state.goal_vertex;
        const auto &acti_handle = sample.state.active_vertex;
        const int id =  s < 0.5 ? acti_handle.idx() : goal_handle.idx();
        const std::size_t map_id = sample.state.map_id;

        vertex_distribution::Ptr &d = vertex_distributions_[map_id][id];
        if(!d) {
            d.reset(new vertex_distribution);
            d->handle = s < 0.5 ? acti_handle : goal_handle;
        }
        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        const mesh_map_tree_t *map = ss->as<MeshMap>().data();
        const cslibs_math_3d::Vector3d pos = sample.state.getPosition(map->getNode(sample.state.map_id)->map);
        d->distribution.add(pos.data(), ignore_weight_ ? 1.0 : sample.weight);
        d->samples.emplace_back(&sample);
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
                vertex_distribution::Ptr &d = v.second;

                if(d->cluster_id != -1)
                    continue;

                /// check all neighbors for cluster id
                /// if d < dmax : assign to neighbor
                /// else new cluster
                const Eigen::Vector3d mean = d->distribution.getMean();
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
                    cluster_distribution::Ptr &cd = clusters_[min_id];
                    assert(cd);
                    cd->samples.insert(cd->samples.end(), d->samples.begin(), d->samples.end());
                    cd->distribution += d->distribution;
                } else {
                    ++cluster_id;
                    d->cluster_id = cluster_id;
                    cluster_distribution::Ptr cd = clusters_[cluster_id];
                    if(!cd) {
                        cd.reset(new cluster_distribution);
                    }
                    cd->distribution = d->distribution;
                    cd->samples = d->samples;
                }
            }
        }
    }

private:
    vertex_distributions_t      vertex_distributions_;
    cluster_map_t               clusters_;
    MeshMapProvider::Ptr        map_provider_;
    bool                        ignore_weight_;
    double                      radius_;
    double                      relative_weight_threshold_;

};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::NearestNeighborDensity, muse_armcl::SampleDensity)

