#ifndef MSGS_CONVERSION_HPP
#define MSGS_CONVERSION_HPP
#include <sensor_msgs/JointState.h>
#include <jaco2_contact_msgs/Jaco2CollisionSequence.h>
#include <cslibs_kdl_data/types.h>
#include <cslibs_kdl_data/joint_state_data.h>
#include <cslibs_kdl_conversion/cslibs_kdl_conversion.h>
#include <muse_armcl/evaluation/contact_sample.hpp>
namespace muse_armcl {

inline void convert(const jaco2_msgs::Jaco2JointState& in, sensor_msgs::JointState& out)
{
    out.header = in.header;
    out.name = in.name;
    out.position = in.position;
    out.velocity = in.velocity;
    out.effort = in.effort;
}

inline void convert(const jaco2_msgs::Jaco2JointState &in, cslibs_kdl_data::JointStateDataStamped& out)
{
    cslibs_kdl::HeaderConversion::ros2data(in.header, out.header);
    out.gravity = cslibs_kdl_data::Vector3(in.gx, in.gy, in.gz);
    out.names = in.name;
    out.position = in.position;
    out.velocity = in.velocity;
    out.acceleration = in.acceleration;
    out.torque = in.effort;
}

inline void convert(const jaco2_msgs::Jaco2Accelerometers& in, cslibs_kdl_data::AccelerometerData& out)
{
    for(auto i : in.lin_acc){
        cslibs_kdl_data::Vector3Stamped vec;
        cslibs_kdl::Vector3StampedConversion::ros2data(i, vec);
        out.push_back(vec);
    }
}

inline void convert(const jaco2_contact_msgs::Jaco2CollisionSample& msg, cslibs_kdl_data::JointStateDataStamped& out)
{
    out.label = msg.label;
    convert(msg.state, out);
}

inline void convert(const jaco2_contact_msgs::Jaco2CollisionSample& in, muse_armcl::ContactSample& out)
{
    convert(in.state, out.state);
    convert(in.lin_acc, out.lin_acc);
    out.label = in.label;
    cslibs_kdl::Vector3StampedConversion::ros2data(in.contact_force, out.contact_force);
    out.state.label = in.label;
    out.lin_acc.label = in.label;
}

}
#endif // MSGS_CONVERSION_HPP
