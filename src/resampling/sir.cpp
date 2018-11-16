#include <muse_armcl/resampling/resampling.hpp>

namespace muse_armcl {
class EIGEN_ALIGN16 SIR : public Resampling
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<SIR>;
protected:
    virtual void doSetup(ros::NodeHandle &nh) override
    {
    }

    void doApply(sample_set_t &sample_set)
    {
        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        const std::size_t size = p_t_1.size();
        assert(size != 0);

        cslibs_math::random::Uniform<1> rng(0.0, 1.0);
        double U = rng.get() / static_cast<double>(size);
        double Q = 0.0;
        std::size_t i = 0, j = 0, k = 0;

        typename sample_set_t::sample_insertion_t  i_p_t = sample_set.getInsertion();
        auto insert = [&i_p_t, &p_t_1, &i](const std::size_t& idx) {
            i_p_t.insert(p_t_1[idx]);
            ++i;
        };

        while (U < 1.0) {
            if (Q > U) {
                U += 1.0 / static_cast<double>(size);
                if (k >= size || i >= size) {
                    while (i < size)
                        insert(size - 1);
                    return;
                }
                insert(k);
            }
            else {
                ++j;
                k = j;
                if (j >= size) {
                    while (i < size)
                        insert(size - 1);
                    return;
                }
                Q += p_t_1[j].weight;
                if (j == size) {
                    while (i < size)
                        insert(k - 1);
                    return;
                }
            }
        }
        while (i < size) {
            if (k >= size) {
                k = size-1;
                insert(k);
            }
        }
    }

    void doApplyRecovery(sample_set_t &sample_set)
    {
        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        const std::size_t size = p_t_1.size();
        assert(size != 0);

        cslibs_math::random::Uniform<1> rng(0.0, 1.0);
        double U = rng.get() / static_cast<double>(size);
        double Q = 0.0;
        std::size_t i = 0, j = 0, k = 0;

        typename sample_set_t::sample_insertion_t  i_p_t = sample_set.getInsertion();
        auto insert = [this, &rng, &i_p_t, &p_t_1, &i](const std::size_t& idx) {
            const double recovery_probability = rng.get();
            if(recovery_probability < recovery_random_pose_probability_) {
                StateSpaceDescription::sample_t sample;
                uniform_pose_sampler_->apply(sample);
                i_p_t.insert(sample);
            } else
                i_p_t.insert(p_t_1[idx]);
            ++i;
        };

        while (U < 1.0) {
            if (Q > U) {
                U += 1.0 / static_cast<double>(size);
                if (k >= size || i >= size) {
                    while (i < size)
                        insert(size - 1);
                    return;
                }
                insert(k);
            }
            else {
                ++j;
                k = j;
                if (j >= size) {
                    while (i < size)
                        insert(size - 1);
                    return;
                }
                Q += p_t_1[j].weight;
                if (j == size) {
                    while (i < size)
                        insert(k - 1);
                    return;
                }
            }
        }
        while (i < size) {
            if (k >= size) {
                k = size-1;
                insert(k);
            }
        }
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::SIR, muse_armcl::Resampling)
