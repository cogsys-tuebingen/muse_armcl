#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_armcl/state_space/mesh_map_provider.hpp>
#include <muse_armcl/state_space/mesh_map.h>

#include <cslibs_mesh_map/cslibs_mesh_map_visualization.h>
#include <ros/ros.h>

namespace muse_armcl {
class MeshMapLoader : public MeshMapProvider
{
public:
    state_space_t::ConstPtr getStateSpace() const override
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        updateTransformations();
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

        const std::string              path       = nh.param<std::string>(param_name("path"), "");
        const std::vector<std::string> files      = nh.param<std::vector<std::string>>("meshes",std::vector<std::string>());
        const std::vector<std::string> parent_ids = nh.param<std::vector<std::string>>("parent_id",std::vector<std::string>());
        frame_ids_                                = nh.param<std::vector<std::string>>("frame_id",std::vector<std::string>());

        auto load = [this, path]() {
            if (!map_) {
                ROS_INFO_STREAM("[" << name_ << "]: Loading mesh map [" << path << "]");

                /// load map
                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new mesh_map_tree_t);
                map_->loadFromFile(path, parent_ids, frame_ids_, files);

                /// update transformations
                updateTransformations();

                /// finish load by unlocking mutex
                l.unlock();
                ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");

                notify_.notify_all();
            }
        };

        worker_ = std::thread(load);
    }

private:
    using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
    using mesh_map_t      = cslibs_mesh_map::MeshMap;

    mutable std::mutex                      map_mutex_;
    std::thread                             worker_;
    mutable std::condition_variable         notify_;

    mutable MeshMap::Ptr                    map_;
    std::vector<std::string>                frame_ids_;
    bool                                    first_load_ = true;
    ros::Time                               last_update_;

    ros::Publisher                          pub_surface_;
    mutable visualization_msgs::MarkerArray markers_;
    mutable visualization_msgs::Marker      msg_;

    inline void updateTransformations() const
    {
        const ros::Time now = ros::Time(0);
        if (!map_ || (!first_load_ && (now == last_update_)))
            return;

        first_load_  = false;
        last_update_ = now;
        resetMarkers();

        /// set current transforms between links
        for (const std::string& frame_id : frame_ids_) {
            const mesh_map_tree_t* m = map_->data()->getNode(frame_id);

            std::string root = m->parent_id_;
            if (root == "") root = frame_id;

            cslibs_math_3d::Transform3d transform;
            if (!tf_->lookupTransform(root, frame_id, now, transform, tf_timeout_))
                std::cerr << "Can't lookup transform: " << root << " <- " << frame_id << std::endl;
            else
                m->transform_ = transform;

            addMarker(m->map_);
        }

        publishMarkers();
    }

    inline void resetMarkers() const
    {
        markers_.markers.clear();
        msg_.action = visualization_msgs::Marker::MODIFY;
        msg_.lifetime = ros::Duration(0.2);
        msg_.id = 0;
    }

    inline void addMarker(const mesh_map_t& link) const
    {
        cslibs_mesh_map::visualization::visualizeVertices(link, msg_);
        markers_.markers.push_back(msg_);
        cslibs_mesh_map::visualization::visualizeBoundry(link, markers_);
    }

    inline void publishMarkers() const
    {
        const ros::Time now = ros::Time::now();
        for (auto &m : markers_.markers)
            m.header.stamp = now;
        pub_surface_.publish(markers_);
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::MeshMapLoader, muse_armcl::MeshMapProvider)
