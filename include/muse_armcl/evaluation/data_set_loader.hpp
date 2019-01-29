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
//    suint64_t last_nsecs = 0;
    tf::tfMessage::Ptr tf;
    jaco2_contact_msgs::Jaco2CollisionSequence::Ptr sequence;
    std::size_t calls_tf = 0;
    std::size_t calls_seq = 0;
    std::size_t iter = 0;
    for(const rosbag::MessageInstance &m : view) {
        uint64_t nsecs = m.getTime().toNSec();
        if(m.getTopic() == topic_tf) {
            ++iter;
            tf::tfMessage::Ptr tf = m.instantiate<tf::tfMessage>();
            if(tf){
                ++calls_tf;
                set.setTf(nsecs, *tf);
            }
        }
        if(m.getTopic() == topic_contact){
            ++iter;
            jaco2_contact_msgs::Jaco2CollisionSequence::Ptr sequence = m.instantiate<jaco2_contact_msgs::Jaco2CollisionSequence>();
            if(sequence){
                ++calls_seq;
                set.setContactData(nsecs, *sequence);
            }
        }
    }

//    std::cout << set.size() << " " << view.size() << " " << calls_tf + calls_seq << " " << iter << std::endl;
    return (set.size() == view.size() / 2) && ( view.size() % 2 == 0);
}
}
}
#endif // DATA_SET_LOADER_HPP
