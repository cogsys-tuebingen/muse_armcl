#ifndef CONTACT_SAMPLE_HPP
#define CONTACT_SAMPLE_HPP
#include <cslibs_kdl_data/types.h>
#include <cslibs_kdl_data/stamped_data.h>
#include <cslibs_kdl_data/accelerometer_data.h>
#include <cslibs_kdl_data/joint_state_data.h>
namespace muse_armcl {
struct ContactSample
{
    ContactSample() {}

    cslibs_kdl_data::Header                header;
    cslibs_kdl_data::JointStateDataStamped state;
    cslibs_kdl_data::AccelerometerData     lin_acc;
    cslibs_kdl_data::Vector3Stamped        contact_force;
    int label;

};

}
#endif // CONTACT_SAMPLE_HPP
