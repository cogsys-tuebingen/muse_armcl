#ifndef MUSE_ARMCL_STATE_PUBLISHER_H
#define MUSE_ARMCL_STATE_PUBLISHER_H

#include <muse_smc/smc/smc_state.hpp>
#include <muse_armcl/state_space/mesh_map_provider.hpp>

#include <ros/ros.h>

namespace muse_armcl {
class EIGEN_ALIGN16 StatePublisher : public muse_smc::SMCState<StateSpaceDescription>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<StatePublisher>;

    using Ptr = std::shared_ptr<StatePublisher>;
    using map_provider_map_t = std::map<std::string, MeshMapProvider::Ptr>;

    void setup(ros::NodeHandle &nh, map_provider_map_t &map_providers);

    virtual void publish(const typename sample_set_t::ConstPtr &sample_set) override;
    virtual void publishIntermediate(const typename sample_set_t::ConstPtr &sample_set) override;
    virtual void publishConstant(const typename sample_set_t::ConstPtr &sample_set) override;

private:
    MeshMapProvider::Ptr map_provider_;

    ros::Publisher pub_particles_;
    ros::Publisher pub_contacts_;

    void publish(const typename sample_set_t::ConstPtr &sample_set, const bool &publish_contacts);
};
}

#endif // MUSE_ARMCL_STATE_PUBLISHER_H
