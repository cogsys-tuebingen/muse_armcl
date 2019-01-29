#ifndef CONTACT_POINT_HISTOGRAM_H
#define CONTACT_POINT_HISTOGRAM_H
#include <muse_armcl/density/sample_density.hpp>

#include <unordered_map>
#include <cslibs_math/statistics/weighted_distribution.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math/color/color.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>
namespace muse_armcl {
class EIGEN_ALIGN16 ContactPointHistogram : public muse_armcl::SampleDensity
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct DiscreteCluster
    {
        DiscreteCluster() :
            hits(0),
            dist(std::numeric_limits<double>::max())
        {}
        double hits;
        double dist;
        sample_t const* sample;
    };

    struct DiscreteContactPoint
    {
        DiscreteContactPoint(const cslibs_kdl::KDLTransformation& t):
            name(t.name),
            frame(t.frame)
        {
            std::string name = t.name;
            name.erase(0,1);
            label = std::stoi(name);
        }

        std::string name;
        KDL::Frame frame;
        int label;
    };

    using histogram_t = std::map<int, DiscreteCluster>;
    using Ptr                = std::shared_ptr<ContactPointHistogram>;
    using ConstPtr           = std::shared_ptr<ContactPointHistogram const>;

    inline const static std::string Type()
    {
        return "muse_armcl::ContactPointHistogram";
    }
    std::size_t histogramSize() const override;
    void setup(const map_provider_map_t &map_providers,
               ros::NodeHandle &nh) override;

    void contacts(sample_vector_t &states) const override;

    void getTopLabels(std::vector<std::pair<int,double>>& labels) const;
    void clear() override;

    void insert(const sample_t &sample) override;
    void estimate() override;
protected:
    bool                                                     ignore_func_;
    std::size_t                                              n_contacts_;
    std::map<std::string, std::vector<DiscreteContactPoint>> labeled_contact_points_;
    histogram_t                                              histo_;
    MeshMapProvider::Ptr                                     map_provider_;
    std::map<double, std::vector<int>>                       ranked_labels_;

};
}
#endif // CONTACT_POINT_HISTOGRAM_H
