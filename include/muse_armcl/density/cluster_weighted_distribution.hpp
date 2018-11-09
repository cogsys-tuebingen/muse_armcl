#ifndef MUSE_ARMCL_CLUSTER_WEIGHTED_DISTRIBUTION_HPP
#define MUSE_ARMCL_CLUSTER_WEIGHTED_DISTRIBUTION_HPP

#include <unordered_map>

#include <muse_armcl/density/cluster_data.hpp>
#include <muse_armcl/density/indexation.hpp>

#include <cslibs_math/statistics/weighted_distribution.hpp>
#include <cslibs_indexed_storage/operations/clustering.hpp>
namespace cis = cslibs_indexed_storage;

namespace muse_armcl {
struct EIGEN_ALIGN16 ClusterWeightedDistribution
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ClusterWeightedDistribution>;

    using index_t             = Indexation::index_t;
    using position_t          = Indexation::position_t;
    using data_t              = ClusterData;
    using sample_t            = ClusterData::sample_t;
    using sample_map_t        = std::unordered_map<int, const sample_t*>;
    using sample_vector_t     = ClusterData::sample_vector_t;
    using sample_vector_map_t = std::unordered_map<int, sample_vector_t>;
    using distribution_t      = cslibs_math::statistics::WeightedDistribution<3>;
    using distribution_map_t  = std::unordered_map<int, distribution_t>;

    inline void clear()
    {
        current_cluster = -1;
        samples.clear();
        distributions.clear();
    }

    /// called when a new cluster should be started
    bool start(const index_t&, data_t& data)
    {
        if (data.cluster != -1)
            return false;

        current_cluster += 1;
        data.cluster = current_cluster;

        samples[current_cluster].insert(samples[current_cluster].end(), data.samples.begin(), data.samples.end());
        for (const auto &s : data.samples)
            distributions[current_cluster].add(s.second, s.first->weight);

        return true;
    }

    /// called when a cluster is extended due to found neighbors
    bool extend(const index_t&, const index_t&, data_t& data)
    {
        if (data.cluster != -1)
            return false;

        data.cluster = current_cluster;

        samples[current_cluster].insert(samples[current_cluster].end(), data.samples.begin(), data.samples.end());
        for (const auto &s : data.samples)
            distributions[current_cluster].add(s.second, s.first->weight);

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

    /// get cluster sample
    inline sample_map_t getSamples() const
    {
        sample_map_t map;

        for (auto &d : distributions) {
            const int cluster = d.first;
            const auto m = d.second.getMean();
            const position_t mean(m(0), m(1), m(2));
            double dist = std::numeric_limits<double>::max();

            const sample_t* res = NULL;
            for (auto &s : samples.at(cluster)) {
                const position_t pos = s.second;
                const double d = (mean - pos).length();
                if (d < dist) {
                    dist = d;
                    res = s.first;
                }
            }

            map[cluster] = res;
        }
        return map;
    }

    int current_cluster = -1;   /// keep track of the current cluster index
    sample_vector_map_t samples;
    distribution_map_t  distributions;
};
}

#endif // MUSE_ARMCL_CLUSTER_WEIGHTED_DISTRIBUTION_HPP
