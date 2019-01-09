#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

#include <muse_armcl/state_space/transform_graph.h>

tf2_msgs::TFMessage tfm;
void tfCb(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    tfm = *msg;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tfg_test_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<tf2_msgs::TFMessage>("/tf", 1, tfCb);
    tf::TransformListener listener;

    ros::Rate rate(10);
    for(std::size_t i = 0; i < 100; ++i){
        ros::spinOnce();
        rate.sleep();
    }

    tf::StampedTransform transform, t2, t3;
    listener.lookupTransform("jaco_link_3", "jaco_link_finger_1",ros::Time(0), transform);
    listener.lookupTransform("jaco_link_1", "jaco_link_5",       ros::Time(0), t2);
    listener.lookupTransform("jaco_link_2", "jaco_link_finger_3",ros::Time(0), t3);



    std::vector<tf::StampedTransform> tf_msgs;
    for(auto t : tfm.transforms){
        tf::StampedTransform tf_t;
        tf::transformStampedMsgToTF(t, tf_t);
        tf_msgs.push_back(tf_t);
    }
    TransformGraph tfg;
    tfg.setup(tf_msgs);

    cslibs_math_3d::Transform3d l3_T_f1, l1_T_l5, l2_T_f3;
    tfg.query("jaco_link_3", "jaco_link_finger_1", l3_T_f1);
    tfg.query("jaco_link_1", "jaco_link_5",        l1_T_l5);
    tfg.query("jaco_link_2", "jaco_link_finger_3", l2_T_f3);


    auto print = [](const tf::StampedTransform& t){
        tf::Matrix3x3 m = t.getBasis();
        double r,p,y;
        m.getRPY(r,p,y);
        ROS_INFO_STREAM("tf:  [" << t.getOrigin().x() << ", "
                        << t.getOrigin().y() << ", "
                        << t.getOrigin().z() << ", "
                        << r << ", "  << p << ", " << y << "]");

    };


    ROS_INFO_STREAM("tfg: " << l3_T_f1);
    print(transform);
    ROS_INFO("===");
    ROS_INFO_STREAM("tfg: " << l1_T_l5);
    print(t2);
    ROS_INFO("===");
    ROS_INFO_STREAM("tfg: " << l2_T_f3);
    print(t3);

    return 0;
}
