#ifndef CONTACT_POINT_HISTOGRAM_MIN_H
#define CONTACT_POINT_HISTOGRAM_MIN_H
#include <muse_armcl/density/contact_point_histogram.h>
namespace muse_armcl {
class EIGEN_ALIGN16 ContactPointHistogramMin : public muse_armcl::ContactPointHistogram
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr                = std::shared_ptr<ContactPointHistogramMin>;
    using ConstPtr           = std::shared_ptr<ContactPointHistogramMin const>;

    inline const static std::string Type()
    {
        return "muse_armcl::ContactPointHistogramMin";
    }

    void insert(const sample_t &sample) override;
};
}
#endif // CONTACT_POINT_HISTOGRAM_MIN_H
