#ifndef CONTACT_POINT_HISTOGRAM_MIN_H
#define CONTACT_POINT_HISTOGRAM_MIN_H
#include <muse_armcl/density/contact_point_histogram.h>
namespace muse_armcl {
class EIGEN_ALIGN16 ContactPointHistogramMin : public muse_armcl::ContactPointHistogram
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ContactPointHistogramMin>;

    using Ptr                = std::shared_ptr<ContactPointHistogramMin>;
    using ConstPtr           = std::shared_ptr<ContactPointHistogramMin const>;

    inline const static std::string Type()
    {
        return "muse_armcl::ContactPointHistogramMin";
    }

    void setup(const map_provider_map_t &map_providers,
               ros::NodeHandle &nh) override;
    void insert(const sample_t &sample) override;
protected:
    bool restrict_neigbours_;
    double scale_pos_;
    double scale_dir_;
    std::map<std::string, std::vector<std::string>> search_links_;

    void setupSearchLinks();
};
}
#endif // CONTACT_POINT_HISTOGRAM_MIN_H
