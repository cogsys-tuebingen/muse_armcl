#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <muse_armcl/evaluation/contact_evaluation_data.hpp>
#include <muse_armcl/evaluation/data_set_loader.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_armcl_test_bag_node");
    if(argc != 2) {
        std::cout << "Please run 'rosrun muse_armcl muse_armcl_test_bag_node <path-to-bag>'" << std::endl;
        return 0;
    }

    ros::NodeHandle nh("~");

    std::vector<std::string> topics = {{nh.param<std::string>("tf", "/first_tf"),
                                        nh.param<std::string>("joint", "/contact_data")}};
    rosbag::Bag     bag(argv[1]);
    rosbag::View    view(bag, rosbag::TopicQuery(topics));

    std::cout << view.getBeginTime() << "\n";

    muse_armcl::DataSet data_set;
    bool success = muse_armcl::DataSetLoader::loadFromBag(bag, topics[0], topics[1], data_set);
    std::cout << (success ? " loaded data successfully" : " loading data failed") << std::endl;
//    std::size_t nmsg = view.size();
//    std::size_t counter = 0;
//    for(const rosbag::MessageInstance &m : view) {
//        uint64_t nsecs = m.getTime().toNSec();
//        if(m.getTopic() == topics[0]) {
////            std::cout << "[" << topics[0] << "]:" << nsecs << std::endl;
//            tf::tfMessage::Ptr tf = m.instantiate<tf::tfMessage>();
//            if(tf)
//                data_set.setTf(nsecs, *tf);
//        }
//        if(m.getTopic() == topics[1]){
////            std::cout << "[" << topics[1] << "]:" <<  nsecs << std::endl;
//            jaco2_contact_msgs::Jaco2CollisionSequence::Ptr sequence = m.instantiate<jaco2_contact_msgs::Jaco2CollisionSequence>();
//            if(sequence)
//                data_set.setContactData(nsecs, *sequence);
//        }
//        std::cout << static_cast<double>(++counter)/ static_cast<double>(nmsg) *100<< " % of msg read" << std::endl;
//    }

//    std::cout << view.size() << ", " << view.size() / 2 << ", " << view.size() % 2 << std::endl;
//    std::cout << data_set.size() << ((data_set.size() == view.size() / 2) ? " check passed " : " check failed ") << std::endl;

    bag.close();

    return 0;
}
