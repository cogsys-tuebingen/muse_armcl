#include <muse_armcl/update/joint_state_data.hpp>
#include <cslibs_plugins_data/data_provider.hpp>

namespace muse_armcl {
class JointStateProvider : public cslibs_plugins_data::DataProvider
{
protected:
    ros::Subscriber source_; /// the subscriber to be used
    std::string     topic_;  /// topic to listen to

    ros::Duration   time_offset_;
    ros::Time       time_of_last_measurement_;

    void callback(const sensor_msgs::JointStateConstPtr &msg)
    {
        if (!time_offset_.isZero() && !time_of_last_measurement_.isZero())
            if (msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
                return;

        const auto &nsec = msg->header.stamp.toNSec();
        JointStateData::Ptr data(new JointStateData(msg->header.frame_id,
                                                    cslibs_time::TimeFrame(nsec, nsec),
                                                    cslibs_time::Time(std::max(nsec, ros::Time::now().toNSec())),
                                                    msg->name,
                                                    msg->position,
                                                    msg->velocity,
                                                    msg->effort));

        data_received_(data);
        time_of_last_measurement_ = msg->header.stamp;
    }

    virtual void doSetup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){ return name_ + "/" + name; };

        int queue_size  = nh.param<int>(param_name("queue_size"), 1);
        topic_          = nh.param<std::string>(param_name("topic"), "");
        source_         = nh.subscribe(topic_, queue_size, &JointStateProvider::callback, this);

        double rate     = nh.param<double>(param_name("rate"), 0.0);
        if (rate > 0.0) {
            time_offset_ = ros::Duration(1.0 / rate);
            ROS_INFO_STREAM(name_ << ": Throttling joint states to rate of " << rate << "Hz!");
        }
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::JointStateProvider, cslibs_plugins_data::DataProvider)
