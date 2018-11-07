#ifndef MUSE_ARMCL_MESH_MAP_PROVIDER_HPP
#define MUSE_ARMCL_MESH_MAP_PROVIDER_HPP

#include <muse_smc/state_space/state_space_provider.hpp>
#include <muse_armcl/state_space/mesh_map.h>

#include <cslibs_plugins/plugin.hpp>

namespace muse_armcl {
class MeshMapProvider : public muse_smc::StateSpaceProvider<StateSpaceDescription>,
                        public cslibs_plugins::Plugin
{
public:
    using Ptr      = std::shared_ptr<MeshMapProvider>;
    using ConstPtr = std::shared_ptr<MeshMapProvider const>;

    virtual ~MeshMapProvider() = default;

    inline const static std::string Type()
    {
        return "muse_armcl::MeshMapProvider";
    }

    virtual inline const std::string getName() const override
    {
        return cslibs_plugins::Plugin::getName();
    }

    virtual void setup(ros::NodeHandle &nh) = 0;
};
}

#endif // MUSE_ARMCL_MESH_MAP_PROVIDER_HPP
