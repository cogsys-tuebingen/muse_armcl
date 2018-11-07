#include "muse_armcl_node.h"

namespace muse_armcl {
MuseARMCLNode::MuseARMCLNode()
{
}

MuseARMCLNode::~MuseARMCLNode()
{
}

bool MuseARMCLNode::setup()
{
    return true;
}

void MuseARMCLNode::start()
{
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_armcl");

    muse_armcl::MuseARMCLNode node;
    if(node.setup()) {
        ROS_INFO_STREAM("Node is set up and ready to start!");
        node.start();
    } else {
        ROS_ERROR_STREAM("Could not set up the node!");
    }
    return 0;
}
