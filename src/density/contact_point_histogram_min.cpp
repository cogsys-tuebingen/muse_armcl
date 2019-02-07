#include <muse_armcl/density/contact_point_histogram_min.h>
#include <cslibs_math/statistics/weighted_distribution.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math/color/color.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>
using namespace muse_armcl;

void ContactPointHistogramMin::setup(const map_provider_map_t &map_providers, ros::NodeHandle &nh)
{
    ContactPointHistogram::setup(map_providers, nh);
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    restrict_neigbours_ = nh.param(param_name("restrict_neigbours"), true);
    scale_pos_          = nh.param(param_name("scale_pos"), 1.0);
    scale_dir_          = nh.param(param_name("scale_dir"), 1.0);
}

void ContactPointHistogramMin::insert(const sample_t &sample)
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
    cslibs_math_3d::Vector3d dir =  sample.state.getDirection(p_map->map);
    cslibs_math_3d::Transform3d base_T_sample = map->getTranformToBase(link);
    point = base_T_sample * point;
    dir   = base_T_sample * dir;

    //        cslibs_math_3d::Vector3d direction = baseTpred * p.state.getDirection(p_map->map);
    double scale_p = scale_pos_;
    double scale_d = scale_dir_;
    auto likelihood = [point, dir, scale_p, scale_d](const cslibs_math_3d::Vector3d& p,
                                                  const cslibs_math_3d::Vector3d& v){
        cslibs_math_3d::Vector3d dp = point -p;
        cslibs_math_3d::Vector3d dd = dir - v;
        double d = std::exp(-0.5* scale_p * dp.length2()) + std::exp(-0.5* scale_d * dd.length2());
        return d;
    };

    double max_likelihood = std::numeric_limits<double>::min();
    int min_id = -1;

    if(search_links_.empty()){
        setupSearchLinks();
        if(search_links_.empty()){
            return;
        }
    }
    try {
        for(const std::string& frame_id: search_links_.at(link)){
            try {
                const std::vector<DiscreteContactPoint>& points = labeled_contact_points_.at(frame_id);
                for(const DiscreteContactPoint& cp  : points){
                    const KDL::Frame& f = cp.frame;
                    cslibs_math_3d::Vector3d pos_cp (f.p(0), f.p(1), f.p(2));
                    KDL::Vector dir_kdl = f.M * KDL::Vector(-1,0,0);
                    cslibs_math_3d::Vector3d dir_cp(dir_kdl(0), dir_kdl(1), dir_kdl(2));
                    cslibs_math_3d::Transform3d base_T_cp = map->getTranformToBase(frame_id);
                    pos_cp = base_T_cp * pos_cp;
                    dir_cp = base_T_cp * dir_cp;
                    double l_cp = likelihood(pos_cp, dir_cp);

                    if(l_cp > max_likelihood){
                        max_likelihood = l_cp;
                        min_id = cp.label;
                    }
                }
            } catch (const std::exception &e) {
                std::cerr << "[ContactPointHistogramMin]: frame_id " << frame_id << " not found!" << std::endl;
                throw e;
            }

        }
    } catch (const std::exception &e) {
        std::cerr << "[ContactPointHistogramMin]: link " << link << " not found!" << std::endl;
        throw e;
    }


    double fitness = std::fabs(2.0 - max_likelihood);
    DiscreteCluster& cluster = histo_[min_id];
    if(ignore_func_){
      cluster.hits += 1.0;
    } else {
      cluster.hits += sample.state.last_update;
    }
    if(fitness < cluster.dist){
        cluster.dist = fitness;
        cluster.sample = &sample;
    }
}

void ContactPointHistogramMin::setupSearchLinks()
{
    const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
    if (!ss->isType<MeshMap>())
        return;

    using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
    using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;
    const mesh_map_tree_t *map = ss->as<MeshMap>().data();

    if(restrict_neigbours_){
        for(const mesh_map_tree_node_t::Ptr& partial_map : *map){
            std::string frame_id = partial_map->frameId();
            search_links_[frame_id].emplace_back(frame_id);
            std::string parent;
            if(partial_map->parentFrameId(parent)){
                search_links_[frame_id].emplace_back(parent);
            }
            for(const mesh_map_tree_node_t* c : partial_map->children){
                search_links_[frame_id].emplace_back(c->frameId());
            }
        }
    } else {
        std::vector<std::string> frame_ids;
        map->getFrameIds(frame_ids);
        for(const mesh_map_tree_node_t::Ptr& partial_map : *map){
            std::string frame_id = partial_map->frameId();
            search_links_[frame_id] = frame_ids;
        }
    }
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::ContactPointHistogramMin, muse_armcl::SampleDensity)
