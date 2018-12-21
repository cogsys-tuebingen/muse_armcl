#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_armcl/state_space/mesh_map_provider.hpp>

#include <cslibs_mesh_map/cslibs_mesh_map_visualization.h>
#include <ros/ros.h>

namespace muse_armcl {
class EIGEN_ALIGN16 MeshMapLoaderOffline : public MeshMapProvider
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<MeshMapLoaderOffline>;

    MeshMapLoaderOffline():
        stop_waiting_you_son_of_a_bitch_(false)
    {
    }

    virtual ~MeshMapLoaderOffline()
    {
        stop_waiting_you_son_of_a_bitch_ = true;
        if(worker_.joinable())
            worker_.join();
    }

    state_space_t::ConstPtr getStateSpace() const override
    {
        waitForStateSpace();

        std::unique_lock<std::mutex> l(map_mutex_);
        return map_;
    }

    void waitForStateSpace() const override
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        if (!map_)
            notify_.wait(l);
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
                tree.loadFromFile(path_, parent_ids_, frame_ids_, files_);

                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new MeshMap(&tree, frame_ids_.front()));

                /// update transformations
                first_load_ = true;
                updateTransformations();
                first_load_ = false;

                /// finish load by unlocking mutex
                l.unlock();
                ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");

                notify_.notify_all();
            }
        };

        worker_ = std::thread(load);
    }

private:
    using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;
    using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
    using mesh_map_t      = cslibs_mesh_map::MeshMap;

    mutable std::mutex                      map_mutex_;
    std::thread                             worker_;
    mutable std::condition_variable         notify_;

    mesh_map_tree_t                         tree;
    mutable MeshMap::Ptr                    map_;
    bool                                    first_load_;
    mutable ros::Time                       last_update_;

    std::string                             path_;
    std::vector<std::string>                files_;
    std::vector<std::string>                parent_ids_;
    std::vector<std::string>                frame_ids_;

    std::atomic_bool                        stop_waiting_you_son_of_a_bitch_;

    inline void updateTransformations() const
    {
        const ros::Time now = ros::Time::now();
        if (!map_ || (!first_load_ && (now == last_update_)))
            return;

        last_update_ = now;

        std::cout << tf_timeout_ << std::endl;

        for(mesh_map_tree_node_t::Ptr m : tree){
            std::string root;
            std::string frame_id = m->frameId();
            bool found = m->parentFrameId(root);
            if (!found) root = frame_id;
            cslibs_math_3d::Transform3d transform;

            while(!tf_->lookupTransform(root, frame_id , now, transform, tf_timeout_)) {
                std::cout << "[MeshMapLoaderOffline]: I am waiting another round! \n";
                if(stop_waiting_you_son_of_a_bitch_)
                    break;
            }
            m->transform = transform;

        }
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::MeshMapLoaderOffline, muse_armcl::MeshMapProvider)
