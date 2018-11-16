#ifndef MUSE_ARMCL_INDEXATION_HPP
#define MUSE_ARMCL_INDEXATION_HPP

#include <cslibs_math_3d/linear/vector.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 Indexation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Indexation>;
    using resolution_t = std::array<double, 3>;
    using index_t      = std::array<int, 3>;
    using position_t   = cslibs_math_3d::Vector3d;

    inline Indexation() :
        resolution_inv_({0.0, 0.0, 0.0})
    {
    }
    inline Indexation(const resolution_t &resolution) :
        resolution_inv_({1.0 / resolution[0], 1.0 / resolution[1], 1.0 / resolution[2]})
    {
    }
    inline Indexation(const double &resolution) :
        resolution_inv_({1.0 / resolution, 1.0 / resolution, 1.0 / resolution})
    {
    }

    inline index_t create(const position_t &pos) const
    {
        return index_t({{static_cast<int>(std::floor(pos(0) * resolution_inv_[0])),
                         static_cast<int>(std::floor(pos(1) * resolution_inv_[1])),
                         static_cast<int>(std::floor(pos(2) * resolution_inv_[2]))}});
    }

private:
    resolution_t resolution_inv_;
};
}

#endif // MUSE_ARMCL_INDEXATION_HPP
