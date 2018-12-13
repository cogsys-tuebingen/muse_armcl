#ifndef JOINT_STATE_PROVIDER_H
#define JOINT_STATE_PROVIDER_H

#include <muse_armcl/update/joint_state_data.hpp>
#include <cslibs_plugins_data/data_provider.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 JointStateProvider : public cslibs_plugins_data::DataProvider
{
public:
    using Ptr = std::shared_ptr<JointStateProvider>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<JointStateProvider>;
protected:
    ros::Subscriber source_; /// the subscriber to be used
    std::string     topic_;  /// topic to listen to

    ros::Duration   time_offset_;
    ros::Time       time_of_last_measurement_;

    virtual void doSetup(ros::NodeHandle &nh) override;

public:
    void callback(const sensor_msgs::JointStateConstPtr &msg);

};
}

#endif // JOINT_STATE_PROVIDER_H
