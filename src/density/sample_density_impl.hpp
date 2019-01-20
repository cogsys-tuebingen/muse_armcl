#include <muse_armcl/density/sample_density.hpp>
#include <muse_armcl/density/indexation.hpp>
#include <muse_armcl/density/cluster_data.hpp>

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/simple/unordered_component_map.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>
#include <cslibs_indexed_storage/operations/clustering.hpp>
namespace cis = cslibs_indexed_storage;

namespace std
{
//! needed for simple::UnorderedMap
template<>
struct hash<std::array<int, 3>>
{
    typedef std::array<int, 3> argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type const& s) const
    {
        result_type const h1 ( std::hash<int>{}(s[0]) );
        result_type const h2 ( std::hash<int>{}(s[1]) );
        result_type const h3 ( std::hash<int>{}(s[2]) );

        return (h1 ^ (h2 << 1)) | h3;
    }
};
}

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
    using sample_map_t          = typename clustering_t::sample_map_ranked_t;

//    using kd_tree_t            = cis::Storage<data_t, index_t, cis::backend::kdtree::KDTreeBuffered>;
    using kd_tree_t            = cis::Storage<data_t, index_t, cis::backend::simple::UnorderedComponentMap>;
    using kd_tree_clustering_t = cis::operations::clustering::Clustering<kd_tree_t>;

    virtual void setup(const map_provider_map_t &map_providers,
                       ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        n_contacts_                = nh.param(param_name("number_of_contacts"), 10);
        weight_threshold_percentage_ = nh.param(param_name("weight_threshold"), 0.1);
        std::cout << "weight_threshold: "<< weight_threshold_percentage_ << std::endl;
        const double resolution = nh.param(param_name("resolution"), 0.1);
        const double clustering_weight_threshold_percentage = nh.param(param_name("clustering_weight_threshold"), 0.1);
         std::cout << "clustering_weight_threshold: "<< clustering_weight_threshold_percentage << std::endl;
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
        const auto map_sample = map->getNode(sample.state.map_id);
        position_t pos = sample.state.getPosition(map_sample->map);
        cslibs_math_3d::Transform3d base_T_sample = map->getTranformToBase(map_sample->frameId());
        pos = base_T_sample * pos;
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
        std::size_t n_cluster = 0;
        for(const auto& entry : map){
            n_cluster += entry.second.size();
        }

        states.clear();
        auto it = map.rbegin();
        while(states.size() < std::min(n_contacts_, map.size()) && it != map.rend()){
            std::size_t remaining = n_contacts_ - states.size();
            if(it->second.size() > remaining){
                std::map<double, std::vector<const sample_t*>> cand2;
                for(std::vector<const sample_t*>::const_iterator s  = it->second.begin(); s < it->second.end(); ++s){
                    double lu = (*s)->state.last_update;
                    cand2[lu].emplace_back(*s);
                }

                auto it2 = cand2.rbegin();
                while(states.size() < std::min(n_contacts_, n_cluster) && it2 != cand2.rend()){
                    std::size_t remaining = n_contacts_ - states.size();
                    if(it2->second.size() > remaining){
                        for(std::size_t i = 0; i < remaining; ++i){
                            states.emplace_back(*it2->second.at(i));
                        }
//                        states.insert(states.end(), it2->second.begin(), it2->second.begin() + remaining);
                    } else{
                        for(auto si : it2->second){
                            states.emplace_back(*si);
                        }
//                        states.insert(states.end(), it2->second.begin(), it2->second.end());
                    }
                    ++it2;
                }
            } else{
                for(auto si : it->second){
                    states.emplace_back(*si);
                }
//                states.insert(states.end(), it->second.begin(), it->second.end());
            }
            ++it;
        }

//        for (const auto &entry : map)
//            if (entry.second->weight > weight_threshold_)
//                states.push_back(*entry.second);
    }

private:
    MeshMapProvider::Ptr       map_provider_;
    double                     weight_threshold_percentage_;
    double                     weight_threshold_;
    double                     max_weight_;

    indexation_t               indexation_;
    clustering_t               clustering_;
    std::shared_ptr<kd_tree_t> kdtree_;
    std::size_t                n_contacts_;
};
}
