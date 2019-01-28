#include <muse_armcl/density/contact_point_histogram_min.h>
#include <cslibs_math/statistics/weighted_distribution.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math/color/color.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>
using namespace muse_armcl;

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
    double scale_p = 1.0;
    double scale_d = 1.0;
    auto likelihood = [point, dir, scale_p, scale_d](const cslibs_math_3d::Vector3d& p,
                                                  const cslibs_math_3d::Vector3d& v){
        cslibs_math_3d::Vector3d dp = point -p;
        cslibs_math_3d::Vector3d dd = dir - v;
        double d = std::exp(-0.5* scale_p * dp.length2()) + std::exp(-0.5* scale_d * dd.length2());
        return d;
    };

    double max_likelihood = std::numeric_limits<double>::min();
    int min_id = -1;
    for(const std::pair<std::string, std::vector<DiscreteContactPoint>>& p : labeled_contact_points_){
        for(const DiscreteContactPoint& cp  : p.second){
            const KDL::Frame& f = cp.frame;
            cslibs_math_3d::Vector3d pos_cp (f.p(0), f.p(1), f.p(2));
            KDL::Vector dir_kdl = f.M * KDL::Vector(-1,0,0);
            cslibs_math_3d::Vector3d dir_cp(dir_kdl(0), dir_kdl(1), dir_kdl(2));
            cslibs_math_3d::Transform3d base_T_cp = map->getTranformToBase(p.first);
            pos_cp = base_T_cp * pos_cp;
            dir_cp = base_T_cp * dir_cp;

            double l_cp = likelihood(pos_cp, dir_cp);
            if(l_cp > max_likelihood){
                max_likelihood = l_cp;
                min_id = cp.label;
            }
        }

    }
    double fitness = std::fabs(2.0 - max_likelihood);
    DiscreteCluster& cluster = histo_[min_id];
    ++cluster.hits;
    if(fitness < cluster.dist){
        cluster.dist = fitness;
        cluster.sample = &sample;
    }
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::ContactPointHistogramMin, muse_armcl::SampleDensity)
