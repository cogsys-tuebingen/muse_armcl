#include <cslibs_plugins_data/data_provider.hpp>

namespace muse_armcl {
class DummyDataProvider : public cslibs_plugins_data::DataProvider
{
protected:
    class DummyData : public cslibs_plugins_data::Data
    {
    public:
        using Ptr = std::shared_ptr<DummyData>;
        using ConstPtr = std::shared_ptr<const DummyData>;
        DummyData() : Data("") { }
    };

    std::thread thread_;

    virtual void doSetup(ros::NodeHandle &nh) override
    {
        auto send_dummy_data = [this]() {
            while (!data_received_.isEnabled()) { }
            data_received_(DummyData::Ptr(new DummyData()));
        };
        thread_ = std::thread(send_dummy_data);
    }
};
}

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_armcl::DummyDataProvider, cslibs_plugins_data::DataProvider)
