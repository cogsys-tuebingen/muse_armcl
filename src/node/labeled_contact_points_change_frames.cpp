#include <ros/ros.h>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>
#include <tf/transform_listener.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "labeled_cp_change_frames");
    std::string path = argv[1];
    std::vector<cslibs_kdl::KDLTransformation> labeled_contact_points;
    if(path != ""){
        cslibs_kdl::load(path, labeled_contact_points);
    }

    std::vector<std::string> old_frames = {"jaco_link_finger_tip_1",
                                           "jaco_link_finger_tip_2",
                                           "jaco_link_finger_tip_3"};
    std::vector<std::string> new_frames = {"jaco_link_finger_1",
                                           "jaco_link_finger_2",
                                           "jaco_link_finger_3"};
    ros::NodeHandle nh;
    old_frames = nh.param<std::vector<std::string>>("old_frames", old_frames);
    new_frames = nh.param<std::vector<std::string>>("new_frames", new_frames);
    if(old_frames.size() != new_frames.size()){
        return 42;
    }
    tf::TransformListener listener;
    for(std::size_t i = 0; i < old_frames.size(); ++i){
        std::string fo = old_frames[i];
        std::string fn = new_frames[i];
        for(cslibs_kdl::KDLTransformation& t : labeled_contact_points){
            if(t.parent == fo){
                tf::StampedTransform trans;
                bool got_tf = false;
                while(!got_tf){
                    try{
                        listener.lookupTransform(fn, fo, ros::Time(0), trans);
                        got_tf = true;
                    } catch( tf::TransformException ex){
                        ROS_WARN("transfrom exception : %s",ex.what());
                        ros::Duration d(0.1);
                        d.sleep();
                    }
                }
                t.parent = fn;
                tf::Transform frame_new;
                tf::TransformKDLToTF(t.frame, frame_new);
                frame_new = trans * frame_new;
                tf::TransformTFToKDL(frame_new, t.frame);
            }
        }
    }

    cslibs_kdl::save(path, labeled_contact_points);



    return 0;
}
