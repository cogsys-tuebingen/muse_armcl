#ifndef MUSE_ARMCL_CLUSTER_DOMINANT_HPP
#define MUSE_ARMCL_CLUSTER_DOMINANT_HPP

#include <unordered_map>

#include <muse_armcl/density/cluster_data.hpp>
#include <muse_armcl/density/indexation.hpp>

#include <cslibs_indexed_storage/operations/clustering.hpp>
namespace cis = cslibs_indexed_storage;

namespace muse_armcl {
struct EIGEN_ALIGN16 ClusterDominant
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ClusterDominant>;

    using index_t      = Indexation::index_t;
    using data_t       = ClusterData;
    using sample_t     = StateSpaceDescription::sample_t;
    using sample_map_t = std::unordered_map<int, const sample_t*>;

    ClusterDominant(double threshold = 0.1) :
        clustering_weight_threshold_percentage(threshold)
    {
    }

    inline void setMaxWeight(const double max_weight)
    {
        clustering_weight_threshold = clustering_weight_threshold_percentage * max_weight;
    }

    inline void clear()
    {
        current_cluster = -1;
        dominants.clear();
    }

    /// called when a new cluster should be started
    bool start(const index_t&, data_t& data)
    {
        if (data.cluster != -1)
            return false;

        ++current_cluster;
        data.cluster = current_cluster;        

        for (const auto &sample : data.samples) {
            if (sample.first->weight >= clustering_weight_threshold &&
                    (!dominants[current_cluster] || sample.first->weight > dominants[current_cluster]->weight))
                dominants[current_cluster] = sample.first;
        }

        return true;
    }

    /// called when a cluster is extended due to found neighbors
    bool extend(const index_t&, const index_t&, data_t& data)
    {
        if (data.cluster != -1)
            return false;

        data.cluster = current_cluster;

        for (const auto &sample : data.samples) {
            if (sample.first->weight >= clustering_weight_threshold &&
                    (!dominants[current_cluster] || sample.first->weight > dominants[current_cluster]->weight))
                dominants[current_cluster] = sample.first;
        }

        return true;
    }

    /// used neighborhood, look at direct neighbors only
    using neighborhood_t  = cis::operations::clustering::GridNeighborhoodStatic<std::tuple_size<index_t>::value, 3>;
    using visitor_index_t = neighborhood_t::offset_t;   /// currently needed by the clustering API

    /// vistor implementation for neighbors
    template<typename visitor_t>
    void visit_neighbours(const index_t&, const visitor_t& visitior)
    {
        static constexpr auto neighborhood = neighborhood_t{};
        neighborhood.visit(visitior);
    }

    /// get cluster samples
    inline sample_map_t getSamples() const
    {
        return dominants;
    }

    int current_cluster = -1;   /// keep track of the current cluster index
    sample_map_t dominants;

    double clustering_weight_threshold_percentage;
    double clustering_weight_threshold;
};
}

#endif // MUSE_ARMCL_CLUSTER_DOMINANT_HPP
