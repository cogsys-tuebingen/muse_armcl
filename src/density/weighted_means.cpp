#include <muse_armcl/density/sample_density.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 WeightedMeans : public SampleDensity
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<WeightedMeans>;

    virtual void setup(const map_provider_map_t &map_providers,
                       ros::NodeHandle &nh)
    {

    }

    virtual void clear()
    {

    }

    virtual void insert(const sample_t &sample)
    {

    }

    virtual void estimate()
    {

    }

    virtual std::size_t histogramSize() const
    {

    }

    virtual void contacts(std::vector<sample_t> &states) const
    {

    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::WeightedMeans, muse_armcl::SampleDensity)
