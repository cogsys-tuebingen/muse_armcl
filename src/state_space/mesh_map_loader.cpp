#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_armcl/state_space/mesh_map_provider.hpp>
#include <muse_armcl/state_space/mesh_map.h>
#include <ros/ros.h>

namespace muse_armcl {
class MeshMapLoader : public MeshMapProvider
{
public:
    state_space_t::ConstPtr getStateSpace() const override
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        return map_;
    }

    void waitForStateSpace() const override
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        if (!map_)
            notify_.wait(l);
    }

    void setup(ros::NodeHandle &nh) override
    {
        // TODO
    }

protected:
    mutable std::mutex               map_mutex_;
    std::thread                      worker_;
    mutable std::condition_variable  notify_;

    MeshMap::Ptr                     map_;
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::MeshMapLoader, muse_armcl::MeshMapProvider)
