#ifndef MUSE_ARMCL_NODE_H
#define MUSE_ARMCL_NODE_H

#include <muse_smc/smc/smc.hpp>
#include <muse_smc/update/update_relay.hpp>
#include <muse_smc/prediction/prediction_relay.hpp>

#include <muse_armcl/state_space/mesh_map_provider.hpp>
#include <muse_armcl/prediction/prediction_model.hpp>
#include <muse_armcl/update/update_model.hpp>

#include <muse_armcl/sampling/normal_sampling.hpp>
#include <muse_armcl/sampling/uniform_sampling.hpp>
#include <muse_armcl/resampling/resampling.hpp>
#include <muse_armcl/scheduling/scheduler.hpp>

#include <muse_armcl/samples/sample_density.hpp>
#include <muse_armcl/state_space/state_publisher.h>

#include <cslibs_plugins/plugin_loader.hpp>
#include <cslibs_plugins/plugin_factory.hpp>
#include <cslibs_plugins_data/data_provider.hpp>

#include <cslibs_math_ros/tf/tf_listener.hpp>

#include <ros/ros.h>

namespace muse_armcl {
class MuseARMCLNode
{
public:
    MuseARMCLNode();
    ~MuseARMCLNode();

    inline bool setup();
    inline void start();

private:
    using data_t                 = cslibs_plugins_data::Data;
    using data_provider_t        = cslibs_plugins_data::DataProvider;
    using tf_listener_t          = cslibs_math_ros::tf::TFListener;

    using map_provider_map_t     = std::map<std::string, MeshMapProvider::Ptr>;
    using data_provider_map_t    = std::map<std::string, typename data_provider_t::Ptr>;
    using update_model_map_t     = std::map<std::string, UpdateModel::Ptr>;

    using update_relay_t         = muse_smc::UpdateRelay<StateSpaceDescription, data_t, data_provider_t>;
    using prediction_relay_t     = muse_smc::PredictionRelay<StateSpaceDescription, data_t, data_provider_t>;
    using smc_t                  = muse_smc::SMC<StateSpaceDescription, data_t>;
    using sample_set_t           = muse_smc::SampleSet<StateSpaceDescription>;
    using prediction_integrals_t = muse_smc::PredictionIntegrals<StateSpaceDescription, data_t>;

    using update_model_mapping_t = update_relay_t::map_t;

    ros::NodeHandle             nh_private_;
    ros::NodeHandle             nh_public_;

    /// map providers & data providers
    tf_listener_t::Ptr          tf_provider_frontend_;  /// for data providers and data conversion
    tf_listener_t::Ptr          tf_provider_backend_;   /// for the backend (the particle filter and the sensor updates)
    map_provider_map_t          map_providers_;
    data_provider_map_t         data_providers_;

    /// particle filter
    smc_t::Ptr                  particle_filter_;
    prediction_integrals_t::Ptr prediction_integrals_;
    sample_set_t::Ptr           sample_set_;

    /// prediction & update
    update_model_map_t          update_models_;
    PredictionModel::Ptr        prediction_model_;
    update_relay_t::Ptr         update_forwarder_;
    prediction_relay_t::Ptr     predicition_forwarder_;

    /// sampling & resampling
    UniformSampling::Ptr        uniform_sampling_;
    NormalSampling::Ptr         normal_sampling_;
    Resampling::Ptr             resampling_;
    Scheduler::Ptr              scheduler_;

    /// density & publishing
    SampleDensity::Ptr          sample_density_;
    StatePublisher::Ptr         state_publisher_;

    bool getUpdateModelProviderMapping(update_model_mapping_t &update_mapping);
    bool getPredictionProvider(data_provider_t::Ptr &prediction_provider);
};
}

#endif // MUSE_ARMCL_NODE_H
