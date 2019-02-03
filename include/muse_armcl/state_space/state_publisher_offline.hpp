//#pragma once
// ARMCL
#include <muse_armcl/state_space/state_publisher.h>
#include <muse_armcl/evaluation/confusion_matrix.hpp>
#include <muse_armcl/evaluation/detection_result.hpp>
#include <muse_armcl/evaluation/ground_truth_particle_set_distance.hpp>
#include <muse_armcl/evaluation/contact_evaluation_data.hpp>
#include <muse_armcl/density/sample_density.hpp>
// other
#include <jaco2_contact_msgs/Jaco2CollisionSequence.h>
#include <jaco2_contact_msgs/Jaco2CollisionSample.h>
#include <eigen_conversions/eigen_kdl.h>

namespace muse_armcl {
class EIGEN_ALIGN16 StatePublisherOffline : public StatePublisher
{
public:
    //    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<StatePublisherOffline>;

    using Ptr                = std::shared_ptr<StatePublisherOffline>;

    /// callback_t
    using time_callback_t = cslibs_utility::common::delegate<void(const cslibs_time::Time &t)>;


    ///  weight_iterator_t::notify_touch::template    from<sample_set_t, &sample_set_t::weightStatisticReset>(this)
    void setup(ros::NodeHandle &nh, map_provider_map_t &map_providers, time_callback_t &set_time);

    virtual void publish(const typename sample_set_t::ConstPtr &sample_set) override;
    virtual void publishIntermediate(const typename sample_set_t::ConstPtr &sample_set) override;
    virtual void publishConstant(const typename sample_set_t::ConstPtr &sample_set) override;

    void setData(const ContactSequence& data);

    void reset();

    int getClosetPoint(const std::string& frame_id, const cslibs_math_3d::Vector3d& estimated) const;
    std::string getDiscreteContact(const cslibs_mesh_map::MeshMapTree* map,
                                   const ContactSample& gt,
                                   cslibs_math_3d::Vector3d& position,
                                   cslibs_math_3d::Vector3d& direction) const;
    std::string getDiscreteContact(int label,
                                   cslibs_math_3d::Vector3d& position,
                                   cslibs_math_3d::Vector3d& direction) const;

    void exportResults(const std::string& path);

    void reportLikelyHoodOfGt(const typename sample_set_t::ConstPtr &sample_set,
                              const ContactSample& gt,
                              const cslibs_mesh_map::MeshMapTree* map);

    const cslibs_kdl::KDLTransformation& getLabledPoint(int label) const;

private:
    bool vertex_gt_model_;
    time_callback_t     set_time_;
    const ContactSequence* data_;
    ConfusionMatrix confusion_matrix_;
    int no_collision_label_;
    std::vector<DetectionResult> results_;
    std::vector<GroundTruthParticleSetDistance> gt_likely_hood_;
    std::size_t n_sequences_;
    std::size_t n_samples_;

};
}
