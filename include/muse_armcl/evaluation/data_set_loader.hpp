#ifndef DATA_SET_LOADER_HPP
#define DATA_SET_LOADER_HPP
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <muse_armcl/evaluation/contact_evaluation_data.hpp>
namespace muse_armcl {
namespace DataSetLoader {

bool loadFromBag(const rosbag::Bag& bag,
                 std::string topic_tf,
                 std::string topic_contact,
                 DataSet& set)
{
    std::vector<std::string> topics = {topic_tf, topic_contact};
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for(const rosbag::MessageInstance &m : view) {
        uint64_t nsecs = m.getTime().toNSec();
        if(m.getTopic() == topic_tf) {
            tf::tfMessage::Ptr tf = m.instantiate<tf::tfMessage>();
            if(tf)
                set.setTf(nsecs, *tf);
        }
        if(m.getTopic() == topic_contact){
            jaco2_contact_msgs::Jaco2CollisionSequence::Ptr sequence = m.instantiate<jaco2_contact_msgs::Jaco2CollisionSequence>();
            if(sequence)
                set.setContactData(nsecs, *sequence);
        }
    }
    return (set.size() == view.size() / 2) && ( view.size() % 2 == 0);
}
}
}
#endif // DATA_SET_LOADER_HPP
