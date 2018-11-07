#ifndef MUSE_ARMCL_STATE_PUBLISHER_H
#define MUSE_ARMCL_STATE_PUBLISHER_H

#include <muse_smc/smc/smc_state.hpp>
#include <muse_armcl/state_space/state_space_description.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 StatePublisher : public muse_smc::SMCState<StateSpaceDescription>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<StatePublisher>;

    using Ptr = std::shared_ptr<StatePublisher>;

    void setup(ros::NodeHandle &nh);

    virtual void publish(const typename sample_set_t::ConstPtr &sample_set) override;
    virtual void publishIntermediate(const typename sample_set_t::ConstPtr &sample_set) override;
    virtual void publishConstant(const typename sample_set_t::ConstPtr &sample_set) override;
};
}

#endif // MUSE_ARMCL_STATE_PUBLISHER_H
