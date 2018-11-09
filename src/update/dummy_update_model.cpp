#include <muse_armcl/update/update_model.hpp>
#include <muse_armcl/state_space/mesh_map.h>

namespace muse_armcl {
class DummyUpdateModel : public UpdateModel
{
public:
    virtual void apply(const typename data_t::ConstPtr          &data,
                       const typename state_space_t::ConstPtr   &ss,
                       typename sample_set_t::weight_iterator_t  set) override
    {
        if (!ss->isType<MeshMap>()) /// || !data->isType<YourType>()
            return;

        /// cast map to specific type
        using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
        using mesh_map_t      = cslibs_mesh_map::MeshMap;
        const mesh_map_tree_t::Ptr &map = ss->as<MeshMap>().data();

        /// cast data to specific type
        // const YourType &your_data = data->as<YourType>();

        for(auto it = set.begin() ; it != set.end() ; ++it) {
            /// access particle
            const state_t& state = it.state();

            /// TODO: implement your model here, i.e. estimate a weight for each particle
            const double weight = 1.0;

            /// apply estimated weight on particle
            *it *= weight;
        }
    }

    virtual void setup(ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        /// TODO: load parameters with nh.param(param_name(<name>), <default>);
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::DummyUpdateModel, muse_armcl::UpdateModel)
