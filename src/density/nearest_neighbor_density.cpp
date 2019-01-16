#include <muse_armcl/density/sample_density.hpp>

#include <unordered_map>
#include <cslibs_math/statistics/weighted_distribution.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math/color/color.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>
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
        int                             cluster_id = 0;
        std::set<sample_t const*>       samples;
    };

    struct EIGEN_ALIGN16 cluster_distribution
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using allocator_t = Eigen::aligned_allocator<cluster_distribution>;
        using Ptr = std::shared_ptr<cluster_distribution>;

        std::size_t                     map_id;
        distribution_t                  distribution;
        std::set<sample_t const*>       samples;
        std::set<int>                   vertex_ids;
    };


    using vertex_distributions_t = std::unordered_map<std::size_t,
    std::unordered_map<int, vertex_distribution::Ptr>>;
    using vertex_labels_t        = std::unordered_map<int, int>;
    using cluster_map_t          = std::unordered_map<int, cluster_distribution::Ptr>;


    void setup(const map_provider_map_t &map_providers,
               ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        ignore_weight_ = nh.param(param_name("ignore_wieght"), false);
        radius_        = nh.param(param_name("radius"), 0.1);
        radius_       *= radius_;
        relative_weight_threshold_ = nh.param(param_name("relative_weight_threshold"), 0.8);
        n_contacts_                = nh.param(param_name("number_of_contacts"), 10);
        min_cluster_size_          = nh.param(param_name("min_cluster_size"), 10);

        const std::string map_provider_id = nh.param<std::string>("map", ""); /// toplevel parameter
        if (map_provider_id == "")
            throw std::runtime_error("[SampleDensity]: No map provider was found!");

        if (map_providers.find(map_provider_id) == map_providers.end())
            throw std::runtime_error("[SampleDensity]: Cannot find map provider '" + map_provider_id + "'!");

        map_provider_ = map_providers.at(map_provider_id);
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("cluster_cloud",1);

    }

    std::size_t histogramSize() const override
    {
        return vertex_distributions_.size();
    }

    void publishClusters(const cslibs_mesh_map::MeshMapTree* map) const
    {
        std::shared_ptr<cslibs_math_3d::PointcloudRGB3d> part_cloud(new cslibs_math_3d::PointcloudRGB3d);
        /// publish all particles

        for(auto &c : clusters_) {
            cslibs_math::color::Color ccolor = cslibs_math::color::random();
            for(const sample_t* s : c.second->samples) {
                const cslibs_mesh_map::MeshMapTreeNode* p_map = map->getNode(s->state.map_id);
                if (p_map) {
                    cslibs_math_3d::Transform3d T = map->getTranformToBase(p_map->map.frame_id_);
                    cslibs_math_3d::Point3d pos = s->state.getPosition(p_map->map);
                    pos = T * pos;
                    cslibs_math_3d::PointRGB3d point(pos, 0.9f, ccolor);
                    part_cloud->insert(point);
                }
            }
        }
        sensor_msgs::PointCloud2 cloud;
        cslibs_math_ros::sensor_msgs::conversion_3d::from(part_cloud, cloud);
        cloud.header.frame_id = map->front()->frameId();
        cloud.header.stamp = ros::Time::now();
        pub_.publish(cloud);
    }

    void contacts(sample_vector_t &states) const override
    {


        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        const mesh_map_tree_t *map = ss->as<MeshMap>().data();

        auto get_nearest = [&map](const cluster_distribution &c, double& sum_weight)
        {
            double min_distance = std::numeric_limits<double>::max();
            sum_weight = 0;
            sample_t const * sample = nullptr;
            const Eigen::Vector3d mean = c.distribution.getMean();
            for(const sample_t* s : c.samples) {
                const Eigen::Vector3d pos =  s->state.getPosition(map->getNode(s->state.map_id)->map);
                const double distance = (mean - pos).squaredNorm();
                //                std::cout << s->state.map_id << std::endl;
                //                sum_weight += s->weight;
                sum_weight += s->state.last_update;
                if(distance < min_distance) {
                    min_distance = distance;
                    sample = s;
                }
            }
//            sum_weight *= c.samples.size();
            return sample;
        };
        //        std::cout << "====="<< std::endl;

        //        double mean_weight = 0;
        //        double max_weight = 0;
        //        cluster_distribution::Ptr max;
        //        for(auto &c : clusters_) {
        //            const cluster_distribution::Ptr &d = c.second;
        //            const double w = d->distribution.getWeight();
        //            mean_weight += w;
        //            if(w > max_weight) {
        //                max_weight = w;
        //                max = d;
        //            }
        //        }
        //        std::cout << clusters_.size() << std::endl;

        //        mean_weight /= static_cast<double>(clusters_.size());

        /// iterate the clusters and find the point nearest to the mean to estimate the contact
        ///  mean cluster weights ...

        std::map<double, std::vector<sample_t>> candidates;
        for(auto &c : clusters_) {
            /// drop a cluster if it is not weighted high enough compared to the others
            //            if(c.second->distribution.getWeight() * relative_weight_threshold_ < mean_weight)
            //                continue;

            double weight = 0;
            sample_t const * s = get_nearest(*c.second, weight);
            if( s != nullptr){
                candidates[weight].emplace_back(*s);
                //                states.emplace_back(*s);
                std::cout << c.first << " " << weight  << " " << c.second->samples.size() << std::endl;
            }
        }
        states.clear();
        std::map<double, std::vector<sample_t>>::reverse_iterator it = candidates.rbegin();
        while(states.size() < std::min(n_contacts_, clusters_.size()) && it != candidates.rend()){
            std::size_t remaining = n_contacts_ - states.size();
            if(it->second.size() > remaining){
                states.insert(states.end(),
                              it->second.begin(),
                              it->second.begin() + remaining);
            } else{
                states.insert(states.end(), it->second.begin(), it->second.end());
            }
            ++it;
        }

        std::cout << " # clusters " << clusters_.size() << " # states: " << states.size() << std::endl;
        publishClusters(map);
    }

    void clear() override
    {
        vertex_distributions_.clear();
        clusters_.clear();
        vertex_labels_.clear();
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
        d->samples.insert(&sample);
    }

    void estimate() override
    {

        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        const mesh_map_tree_t *map = ss->as<MeshMap>().data();
        int cluster_id = 0;
        /// clustering by map
        for(auto &vd : vertex_distributions_) {
            for(auto &v : vd.second) {
                vertex_distribution::Ptr &d = v.second;

                if(d->cluster_id != 0)
                    continue;

                const cslibs_mesh_map::MeshMapTreeNode* state_map = map->getNode(vd.first);
                const auto& neighbors = state_map->map.getNeighbors(d->handle);
                std::set<int> found_labels;
                for(const auto& n : neighbors){
                    int l = vertex_labels_[n.idx()];
                    found_labels.emplace(l);
                }

                found_labels.erase(0);
                if(found_labels.size() == 0) {
                    /// the current node is alone ...
                    ++cluster_id;
                    d->cluster_id = cluster_id;
                    cluster_distribution::Ptr &cd = clusters_[cluster_id];
                    cd.reset(new cluster_distribution);
                    cd->samples = d->samples;
                    cd->map_id = vd.first;
                    vertex_labels_[d->handle.idx()];
                    for(const auto &n : neighbors) {
                        vertex_labels_[n.idx()] = cluster_id;
                        cd->vertex_ids.emplace(n.idx());
                    }
                    cd->vertex_ids.emplace(d->handle.idx());
                } else if(found_labels.size() == 1) {
                    const int fl = *(found_labels.begin());
                    cluster_distribution::Ptr &cd = clusters_[fl];
                    cd->samples.insert(d->samples.begin(), d->samples.end());
                    cd->distribution += d->distribution;
                    cd->vertex_ids.emplace(d->handle.idx());
                    vertex_labels_[d->handle.idx()] = fl;
                } else {
                    /// merge clusters
                    /// biggest cluster so far
                    std::size_t               large_count   = 0;
                    int                       large_label   = 0;
                    cluster_distribution::Ptr large_cluster = nullptr;
                    for(const int fl : found_labels) {
                        cluster_distribution::Ptr &cd = clusters_[fl];
                        std::size_t cs = cd->vertex_ids.size();
                        if(cs > large_count) {
                            large_count =  cs;
                            large_label =  fl;
                            large_cluster = cd;
                        }
                    }
                    /// add other clusters
                    found_labels.erase(large_label);
                    for(const int fl : found_labels) {
                        cluster_distribution::Ptr &cd = clusters_[fl];
                        if(large_cluster->map_id != cd->map_id){
                            std::cout << "thats where shit comes frome" << std::endl;
                        }
                        large_cluster->distribution += cd->distribution;

                        for(const int n : cd->vertex_ids) {
                            vertex_labels_[n] = large_label;
                        }

                        large_cluster->samples.insert(cd->samples.begin(), cd->samples.end());
                        large_cluster->vertex_ids.insert(cd->vertex_ids.begin(), cd->vertex_ids.end());
                    }

                    /// add new node and neighbors
                    large_cluster->samples.insert(d->samples.begin(), d->samples.end());
                    large_cluster->distribution += d->distribution;
                    large_cluster->vertex_ids.emplace(d->handle.idx());

                    for(const auto &n : neighbors) {
                        vertex_labels_[n.idx()] = large_label;
                        large_cluster->vertex_ids.emplace(n.idx());
                    }
                }
            }
            vertex_labels_.clear();
            ++cluster_id;
        }

        //        for(auto& c1 : clusters_){
        //            std::set<sample_t const*>::const_iterator s = c1.second->samples.begin();
        //            std::size_t map_id_c = (*s)->state.map_id;
        //            bool test = true;
        //            for(const sample_t* s : c1.second->samples){
        //                std::size_t id = s->state.map_id;
        //                test &= (id == map_id_c);
        //                if(!test){
        //                    std::cout << " cluster id: " << c1.first << ", " << map_id_c << "; " << id << std::endl;
        //                }
        //            }
        //            if(!test){
        //                std::cout << (test ? "test passed" : " test failed") << " cluster id: " << c1.first << std::endl;
        //            }
        //        }

        /// maybe use edge costs to expand on surface approximating an integral

        std::vector<int> to_remove;

        /// check for cluster size and remove too small clusters
        //        std::vector<int> to_remove;
        for(auto &c : clusters_) {
            if(c.second->samples.size() < min_cluster_size_) {
                to_remove.emplace_back(c.first);
            }
        }
        for(int c : to_remove)  {
            clusters_.erase(c);
        }

        to_remove.clear();
        /// fuse closeby clusters
//        for(auto &c1 : clusters_) {
//            if(std::fabs(c1.second->distribution.getWeight()) == 0 ){
//                continue;
//            }
//            const Eigen::Vector3d mean1 = c1.second->distribution.getMean();
//            for(auto &c2 : clusters_){
//                if( c1.first != c2.first && std::fabs(c2.second->distribution.getWeight()) > 0){
//                    if(c1.second->map_id == c2.second->map_id && std::find(to_remove.begin(), to_remove.end(), c1.first) == to_remove.end()){
//                        const Eigen::Vector3d mean2 = c2.second->distribution.getMean();
//                        double dist = (mean1 -mean2).squaredNorm();
//                        std::cout << "cluster distance " << dist << std::endl;
//                        if(std::isfinite(dist)){

//                            //                        std::cout << c1.second->map_id << "  | " << c2.second->samples.begin()->state->map_id << std::endl;
//                            if( dist < radius_){
//                                to_remove.emplace_back(c2.first);
//                                c1.second->samples.insert(c2.second->samples.begin(), c2.second->samples.end());
//                            }
//                        }
//                    }
//                }
//            }
//        }

//        for(int c : to_remove)  {
//            clusters_.erase(c);
//        }

    }

private:
    vertex_distributions_t      vertex_distributions_;
    vertex_labels_t             vertex_labels_;
    cluster_map_t               clusters_;
    MeshMapProvider::Ptr        map_provider_;
    bool                        ignore_weight_;
    double                      radius_;
    double                      relative_weight_threshold_;
    std::size_t                 n_contacts_;
    mutable ros::Publisher      pub_;
    int                         min_cluster_size_;

};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::NearestNeighborDensity, muse_armcl::SampleDensity)

