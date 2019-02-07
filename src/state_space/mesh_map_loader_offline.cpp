#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_armcl/state_space/transform_graph.h>
#include <muse_armcl/state_space/mesh_map_provider.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>
#include <cslibs_mesh_map/cslibs_mesh_map_visualization.h>
#include <ros/ros.h>
#include <tf/tf.h>
namespace muse_armcl {
class EIGEN_ALIGN16 MeshMapLoaderOffline : public MeshMapProvider
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<MeshMapLoaderOffline>;

    MeshMapLoaderOffline():
        stop_waiting_you_son_of_a_bitch_(false),
        set_tf_(false)
    {
    }

    virtual ~MeshMapLoaderOffline()
    {
//        stop_waiting_you_son_of_a_bitch_ = true;
//        if(worker_.joinable())
//            worker_.join();
    }

    state_space_t::ConstPtr getStateSpace() const override
    {
//        waitForStateSpace();

//        std::unique_lock<std::mutex> l(map_mutex_);
        return map_;
    }

    void waitForStateSpace() const override
    {
//        std::unique_lock<std::mutex> l(map_mutex_);
//        if (!map_)
//            notify_.wait(l);
    }

    void doSetup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        path_        = nh.param<std::string>(param_name("path"), "");
        files_       = nh.param<std::vector<std::string>>(param_name("meshes"),     std::vector<std::string>());
        parent_ids_  = nh.param<std::vector<std::string>>(param_name("parent_ids"), std::vector<std::string>());
        frame_ids_   = nh.param<std::vector<std::string>>(param_name("frame_ids"),  std::vector<std::string>());

        last_update_ = ros::Time::now();
        auto load = [this]() {
            if (!map_) {
                ROS_INFO_STREAM("[" << name_ << "]: Loading mesh map [" << path_ << "]");

                if (frame_ids_.empty())
                    throw std::runtime_error("[" + name_ + "]: No frame id found!");

                /// load map
                tree_.loadFromFile(path_, parent_ids_, frame_ids_, files_);

//                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new MeshMap(&tree_, frame_ids_.front()));
//                l.unlock();

                /// update transformations
                ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");

//                notify_.notify_all();
            }
        };

//        worker_ = std::thread(load);
        load();
    }

    bool initializeTF(const std::vector<tf::StampedTransform>& transforms)
    {
        ros::Rate r(20);

        TransformGraph tfg;
        if(!tfg.setup(transforms)){
            return false;
        }

        while(tree_.size() != frame_ids_.size()){
            r.sleep();
            ROS_INFO_STREAM(tree_.size() << " vs. " << frame_ids_.size());
        }

        bool set_all = true;
        for(mesh_map_tree_node_t::Ptr m : tree_){
            std::string frame_id = m->frameId();
            std::string parent = "";
            bool has_parent = m->parentFrameId(parent);
            ROS_INFO_STREAM("[" << name_ << "]: " << parent << ", " << frame_id);
            if(has_parent){
                std::unique_lock<std::mutex> l(map_mutex_);
                bool found =  tfg.query(parent, frame_id, m->transform);
                set_all &= found;
            }
        }
        if(set_all){
            ROS_INFO_STREAM("[" << name_ << "]: map transforms successfully set");
        } else{
            ROS_ERROR_STREAM("[" << name_ << "]: setting map transforms failed!");
        }
        set_tf_ = set_all;
        return set_all;
    }

private:
    using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;
    using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
    using mesh_map_t      = cslibs_mesh_map::MeshMap;

    mutable std::mutex                      map_mutex_;
    std::thread                             worker_;
    mutable std::condition_variable         notify_;

    mesh_map_tree_t                         tree_;
    mutable MeshMap::Ptr                    map_;
    bool                                    first_load_;
    mutable ros::Time                       last_update_;

    std::string                             path_;
    std::vector<std::string>                files_;
    std::vector<std::string>                parent_ids_;
    std::vector<std::string>                frame_ids_;

    std::atomic_bool                        stop_waiting_you_son_of_a_bitch_;
    std::atomic_bool                        set_tf_;
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::MeshMapLoaderOffline, muse_armcl::MeshMapProvider)
