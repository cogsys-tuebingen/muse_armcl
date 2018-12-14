#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>


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

    for(const rosbag::MessageInstance &m : view) {
        for(auto &t : topics) {
            if(m.getTopic() == t) {
                std::cout << "[" << t << "]:" <<  m.getTime() << std::endl;
            }
        }
    }
    bag.close();

    return 0;
}
