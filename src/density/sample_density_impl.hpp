#include <muse_armcl/density/sample_density.hpp>
#include <muse_armcl/density/indexation.hpp>
#include <muse_armcl/density/cluster_data.hpp>

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>
#include <cslibs_indexed_storage/operations/clustering.hpp>
namespace cis = cslibs_indexed_storage;

namespace muse_armcl {
template <typename clustering_t>
class EIGEN_ALIGN16 SampleDensityImpl : public SampleDensity
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t           = Eigen::aligned_allocator<SampleDensityImpl>;

    using indexation_t          = Indexation;
    using index_t               = indexation_t::index_t;
    using position_t            = indexation_t::position_t;
    using data_t                = ClusterData;
    using sample_map_t          = typename clustering_t::sample_map_t;

    using kd_tree_t            = cis::Storage<data_t, index_t, cis::backend::kdtree::KDTreeBuffered>;
    using kd_tree_clustering_t = cis::operations::clustering::Clustering<kd_tree_t>;

    virtual void setup(const map_provider_map_t &map_providers,
                       ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        weight_threshold_percentage_ = nh.param(param_name("weight_threshold"), 0.1);
        const double resolution = nh.param(param_name("resolution"), 0.1);
        const double clustering_weight_threshold_percentage = nh.param(param_name("clustering_weight_threshold"), 0.1);
        const std::size_t maximum_sample_size = static_cast<std::size_t>(nh.param<int>(param_name("maximum_sample_size"), 0));

        /// initialize indexation, kdtree, clustering
        indexation_ = indexation_t(resolution);
        clustering_ = clustering_t(clustering_weight_threshold_percentage);
        kdtree_.reset(new kd_tree_t);
        kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * maximum_sample_size + 1);

        const std::string map_provider_id = nh.param<std::string>("map", ""); /// toplevel parameter
        if (map_provider_id == "")
            throw std::runtime_error("[SampleDensity]: No map provider was found!");

        if (map_providers.find(map_provider_id) == map_providers.end())
            throw std::runtime_error("[SampleDensity]: Cannot find map provider '" + map_provider_id + "'!");

        map_provider_ = map_providers.at(map_provider_id);
    }

    virtual void clear()
    {
        clustering_.clear();
        kdtree_->clear();
    }

    virtual void insert(const sample_t &sample)
    {
        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        const mesh_map_tree_t *map = ss->as<MeshMap>().data();

        const position_t pos = sample.state.getPosition(map->getNode(sample.state.map_id)->map);
        kdtree_->insert(indexation_.create(pos), data_t(sample, pos));

        if (sample.weight > max_weight_)
            max_weight_ = sample.weight;
    }

    virtual void estimate()
    {
        clustering_.setMaxWeight(max_weight_);
        kd_tree_clustering_t clustering(*kdtree_);
        clustering.cluster(clustering_);
        weight_threshold_ = weight_threshold_percentage_ * max_weight_;
        max_weight_ = 0.0;
    }

    virtual std::size_t histogramSize() const
    {
        return kdtree_->size();
    }

    virtual void contacts(sample_vector_t &states) const override
    {
        const sample_map_t map = clustering_.getSamples();
        for (const auto &entry : map)
            if (entry.second->weight > weight_threshold_)
                states.push_back(*entry.second);
    }

private:
    MeshMapProvider::Ptr       map_provider_;
    double                     weight_threshold_percentage_;
    double                     weight_threshold_;
    double                     max_weight_;

    indexation_t               indexation_;
    clustering_t               clustering_;
    std::shared_ptr<kd_tree_t> kdtree_;
};
}
