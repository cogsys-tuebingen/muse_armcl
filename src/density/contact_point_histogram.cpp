#include <muse_armcl/density/contact_point_histogram.h>
#include <muse_armcl/density/sample_density.hpp>
#include <unordered_map>
#include <cslibs_math/statistics/weighted_distribution.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math/color/color.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>
namespace muse_armcl {

    std::size_t ContactPointHistogram::histogramSize() const
    {
        return histo_.size();
    }

    void ContactPointHistogram::setup(const map_provider_map_t &map_providers,
               ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        std::string path = nh.param<std::string>("contact_points_file", std::string(""));
        std::vector<cslibs_kdl::KDLTransformation> labeled_contact_points;
        if(path != ""){
            cslibs_kdl::load(path, labeled_contact_points);
        }

        for(const auto& p : labeled_contact_points){
            labeled_contact_points_[p.parent].emplace_back(DiscreteContactPoint(p));
        }
        if(labeled_contact_points_.empty()){
            throw std::runtime_error("[SampleDensity]: Did not get a set od discrete contact point! Cannot cluster!");
        }

        ignore_func_ = nh.param(param_name("ignore_weight"), false);
        n_contacts_ = nh.param(param_name("number_of_contacts"), 10);

        const std::string map_provider_id = nh.param<std::string>("map", ""); /// toplevel parameter
        if (map_provider_id == "")
            throw std::runtime_error("[SampleDensity]: No map provider was found!");

        if (map_providers.find(map_provider_id) == map_providers.end())
            throw std::runtime_error("[SampleDensity]: Cannot find map provider '" + map_provider_id + "'!");

        map_provider_ = map_providers.at(map_provider_id);

    }

    void ContactPointHistogram::contacts(sample_vector_t &states) const
    {
        if(ranked_labels_.empty()){
            return;
        }
        states.clear();
        std::size_t min_element = std::min(n_contacts_, histo_.size());
        auto it = ranked_labels_.rbegin();
        ROS_INFO_STREAM("label: " << it->second.front());
        while(states.size() < min_element && it != ranked_labels_.rend()){
            std::size_t remaining = n_contacts_ - states.size();
            if(it->second.size() > remaining){
                std::map<double, std::vector<sample_t>> cand2;
                for(int label : it->second){
                    const DiscreteCluster& c = histo_.at(label);
                    cand2[c.dist].emplace_back(*c.sample);
                }

                auto it2 = cand2.begin();
                while(states.size() < min_element && it2 != cand2.end()){
                    std::size_t remaining = n_contacts_ - states.size();
                    if(it2->second.size() > remaining){
                        states.insert(states.end(), it2->second.begin(), it2->second.begin() + remaining);
                    } else{
                        states.insert(states.end(), it2->second.begin(), it2->second.end());
                    }
                    ++it2;
                }
            } else{
                for(int label : it->second){
                    const DiscreteCluster& c = histo_.at(label);
                    states.emplace_back(*(c.sample));
                }
            }
            ++it;
        }

    }

    void ContactPointHistogram::getTopLabels(std::vector<std::pair<int, double> > &labels) const
    {
        labels.clear();
        std::size_t min_element = std::min(n_contacts_, histo_.size());
        auto it = ranked_labels_.rbegin();
        while(labels.size() < min_element && it != ranked_labels_.rend()){
            std::size_t remaining = n_contacts_ - labels.size();
            if(it->second.size() > remaining){
                std::map<double, std::vector<int>> cand2;
                for(int label : it->second){
                    const DiscreteCluster& c = histo_.at(label);
                    cand2[c.dist].emplace_back(label);
                }

                auto it2 = cand2.begin();
                while(labels.size() < min_element && it2 != cand2.end()){
                    std::size_t remaining = n_contacts_ - labels.size();
                    if(it2->second.size() > remaining){
                        auto it3 = it2->second.begin();
                        for(std::size_t i = 0; i < remaining; ++i){
                            std::pair<int, double> p;
                            p.first = *it3;
                            p.second = histo_.at(*it3).sample->state.force;
                            labels.emplace_back(p);
                        }
                    } else{
                        for(auto l : it2->second){
                            std::pair<int, double> p;
                            p.first = l;
                            p.second = histo_.at(l).sample->state.force;
                            labels.emplace_back(p);
                        }
                    }
                    ++it2;
                }
            } else{
                for(int label : it->second){
                    std::pair<int, double> p;
                    p.first = label;
                    p.second = histo_.at(label).sample->state.force;
                    labels.emplace_back(p);
                }
            }
            ++it;
        }
    }

    void ContactPointHistogram::clear()
    {
        histo_.clear();
        ranked_labels_.clear();
    }

    void ContactPointHistogram::insert(const sample_t &sample)
    {
        if(labeled_contact_points_.empty()){
            ROS_ERROR("No discrete contact points provided!");
            return;
        }
        const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
        if (!ss->isType<MeshMap>())
            return;

        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;
        const mesh_map_tree_t *map = ss->as<MeshMap>().data();
        const mesh_map_tree_node_t* p_map = map->getNode(sample.state.map_id);
        if(!p_map){
            return;
        }

        if(sample.state.force < 0.01){
            return;
        }

        std::string link = p_map->frameId();
        cslibs_math_3d::Vector3d point =  sample.state.getPosition(p_map->map);
        //        cslibs_math_3d::Vector3d direction = baseTpred * p.state.getDirection(p_map->map);
        auto dist_sq = [point](const KDL::Vector& v){
            double d = 0;
            for(std::size_t i = 0; i < 3; ++i){
                double diff = point(i) - v(i);
                d += diff * diff;
            }
            return d;
        };

        double min_d = std::numeric_limits<double>::max();
        int min_id = -1;
        for(const auto& p : labeled_contact_points_.at(link)){
            double dist = dist_sq(p.frame.p);
            if(dist < min_d){
                min_d = dist;
                min_id = p.label;
            }
        }

        DiscreteCluster& cluster = histo_[min_id];
        if(ignore_func_){
          cluster.hits += 1.0;
        } else {
          cluster.hits += sample.state.last_update;
        }
        if(min_d < cluster.dist){
            cluster.dist = min_d;
            cluster.sample = &sample;
        }
    }

    void ContactPointHistogram::estimate()
    {
        ranked_labels_.clear();
        for(const std::pair<double, DiscreteCluster>& p : histo_){
            ranked_labels_[p.second.hits].emplace_back(p.first);
        }
    }
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::ContactPointHistogram, muse_armcl::SampleDensity)
