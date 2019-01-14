#include "muse_armcl_offline_node.h"

#include <muse_armcl/prediction/prediction_integral.hpp>
#include <muse_armcl/update/joint_state_provider.h>
#include <muse_armcl/evaluation/data_set_loader.hpp>
#include <muse_armcl/evaluation/msgs_conversion.hpp>

#include <tf/transform_broadcaster.h>
#include <jaco2_contact_msgs/Jaco2CollisionSequence.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <tf/tf.h>

namespace muse_armcl {
MuseARMCLOfflineNode::MuseARMCLOfflineNode() :
    nh_private_("~"),
    tf_provider_frontend_(new tf_listener_t),
    tf_provider_backend_(new tf_listener_t)
{
}

MuseARMCLOfflineNode::~MuseARMCLOfflineNode()
{
    for (auto &d : data_providers_) {
        d.second->disable();
    }
}

bool MuseARMCLOfflineNode::setup()
{
    /// load all muse_armcl plugins
    cslibs_plugins::PluginLoader loader("muse_armcl", nh_private_);

    /// load dataset from bagfile
    std::string bag_filename;
    nh_private_.getParam("bag_filename",          bag_filename);
    nh_private_.getParam("bag_joint_state_topic", bag_joint_state_topic_);
    nh_private_.getParam("bag_tf_topic",          bag_tf_topic_);
    std::shared_ptr<rosbag::Bag> bag;
    try {
        //        bag_.reset(new rosbag::Bag(bag_filename, rosbag::bagmode::Read));
        bag.reset(new rosbag::Bag(bag_filename, rosbag::bagmode::Read));
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("Could not load bagfile " + bag_filename + "!");
        return false;
    }
    data_set_.reset(new muse_armcl::DataSet);
    if(!muse_armcl::DataSetLoader::loadFromBag(*bag, bag_tf_topic_, bag_joint_state_topic_, *data_set_)){
        ROS_ERROR_STREAM("Could not load data from bagfile " + bag_filename + "!");
        return false;
    }
    bag->close();
    ROS_INFO_STREAM("Data successfully loaded! " << data_set_->size() << " sequences.");


    {   /// Update Models
        loader.load<UpdateModel, /*tf_provider_t::Ptr, */ros::NodeHandle&>(
                    update_models_, /*tf_provider_backend_, */nh_private_);
        if(update_models_.empty()) {
            ROS_ERROR_STREAM("No update model functions were found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        std::string update_model_list = "[";
        for (auto &e : update_models_) {
            update_model_list += e.first + ",";
        }
        update_model_list.at(update_model_list.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded update function models.");
        ROS_INFO_STREAM(update_model_list);
    }
    {   /// Prediction Model
        loader.load<PredictionModel, /*tf_provider_t::Ptr, */ros::NodeHandle&>(
                    prediction_model_, /*tf_provider_backend_, */nh_private_);
        if (!prediction_model_) {
            ROS_ERROR_STREAM("No prediction model functions was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        ROS_INFO_STREAM("Loaded prediction function model.");
        ROS_INFO_STREAM("[" << prediction_model_->getName() << "]");
    }
    {   /// Map Providers
        loader.load<MeshMapProvider, tf_provider_t::Ptr, ros::NodeHandle&>(
                    map_providers_, tf_provider_frontend_, nh_private_);
        if (map_providers_.empty()) {
            ROS_ERROR_STREAM("No map provider was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        std::string map_provider_list = "[";
        for (auto &e : map_providers_) {
            map_provider_list += e.first + ",";
        }
        map_provider_list.at(map_provider_list.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded map providers.");
        ROS_INFO_STREAM(map_provider_list);
    }
    {   /// Data Providers
        loader.load<data_provider_t, tf_provider_t::Ptr, ros::NodeHandle&>(
                    data_providers_, tf_provider_frontend_, nh_private_);
        if (data_providers_.empty()) {
            ROS_ERROR_STREAM("No data provider was found!");
            return false;
        }
        std::string data_provider_list = "[";
        for (auto &e : data_providers_) {
            data_provider_list += e.first + ",";
        }
        data_provider_list.at(data_provider_list.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded data providers.");
        ROS_INFO_STREAM(data_provider_list);
    }
    { /// Sampling Algorithms
        loader.load<UniformSampling, map_provider_map_t, ros::NodeHandle&>(
                    uniform_sampling_, map_providers_, nh_private_);
        if (!uniform_sampling_) {
            ROS_ERROR_STREAM("No uniform sampling function was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        ROS_INFO_STREAM("Loaded uniform sampler.");
        ROS_INFO_STREAM("[" << uniform_sampling_->getName() << "]");
        loader.load<NormalSampling, map_provider_map_t, ros::NodeHandle&>(
                    normal_sampling_, map_providers_, nh_private_);
        if (!normal_sampling_) {
            ROS_ERROR_STREAM("No gaussian sampling function was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        ROS_INFO_STREAM("Loaded gaussian sampler.");
        ROS_INFO_STREAM("[" << normal_sampling_->getName() << "]");
        loader.load<Resampling, UniformSampling::Ptr, NormalSampling::Ptr, ros::NodeHandle&>(
                    resampling_, uniform_sampling_, normal_sampling_, nh_private_);
        if (!resampling_) {
            ROS_ERROR_STREAM("No resampling function was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        ROS_INFO_STREAM("Loaded resampling algorithm.");
        ROS_INFO_STREAM("[" << resampling_->getName() << "]");
    }
    { /// Density Estimation
        loader.load<SampleDensity, map_provider_map_t, ros::NodeHandle&>(
                    sample_density_, map_providers_, nh_private_);
        if (!sample_density_) {
            ROS_ERROR_STREAM("No sample density estimation function was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }

        ROS_INFO_STREAM("Loaded density estimation function.");
        ROS_INFO_STREAM("[" << sample_density_->getName() << "]");
    }
    { /// Scheduling
        loader.load<Scheduler, const update_model_map_t&, ros::NodeHandle&>(
                    scheduler_, update_models_, nh_private_);
        if (!scheduler_) {
            ROS_ERROR_STREAM("No scheduler was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }

        ROS_INFO_STREAM("Loaded scheduler.");
        ROS_INFO_STREAM("[" << scheduler_->getName() << "]");
    }


    /// results file
    results_file_base_name_ = nh_private_.param<std::string>("results_base_file","/tmp/armcl_res_");

    /// set up the particle filter
    {
        auto param_name = [](const std::string &param){return "particle_filter/" + param;};

        prediction_integrals_.reset(new prediction_integrals_t(PredictionIntegral::Ptr(new PredictionIntegral())));
        for (const auto &u : update_models_) {
            prediction_integrals_->set(PredictionIntegral::Ptr(new PredictionIntegral),
                                       u.second->getId());
        }

        /// parameters
        const std::string world_frame                   = nh_private_.param<std::string>(param_name("world_frame"), "/world");
        const std::size_t sample_size                   = nh_private_.param<int>(param_name("sample_size"), 0);
        const std::size_t minimum_sample_size           = sample_size == 0 ? nh_private_.param<int>(param_name("minimum_sample_size"), 0) : sample_size;
        const std::size_t maximum_sample_size           = sample_size == 0 ? nh_private_.param<int>(param_name("maximum_sample_size"), 0) : sample_size;
        const bool        reset_weights_after_insertion = nh_private_.param<bool>(param_name("reset_weights_after_insertion"), false);
        const bool        reset_weights_to_one          = nh_private_.param<bool>(param_name("reset_weights_to_one"), false);
        const bool        enable_lag_correction         = nh_private_.param<bool>(param_name("enable_lag_correction"), false);

        if (minimum_sample_size == 0) {
            ROS_ERROR_STREAM("Minimum sample size cannot be zero!");
            return false;
        }
        if (maximum_sample_size < minimum_sample_size) {
            ROS_ERROR_STREAM("Maximum sample size cannot be smaller than minimum sample size!");
            return false;
        }

        sample_set_.reset(new sample_set_t(world_frame,
                                           cslibs_time::Time(ros::Time::now().toNSec()),
                                           minimum_sample_size,
                                           maximum_sample_size,
                                           sample_density_,
                                           reset_weights_after_insertion,
                                           reset_weights_to_one));
        state_publisher_.reset(new StatePublisherOffline);
        StatePublisherOffline::time_callback_t time_callback =  StatePublisherOffline::time_callback_t::from<MuseARMCLOfflineNode, &MuseARMCLOfflineNode::setFilterTime>(this);
        state_publisher_->setup(nh_private_,
                                map_providers_,
                                time_callback);

        const bool reset_all_model_accumulators_on_update = false;
        const bool reset_all_model_accumulators_on_resampling = false;


        particle_filter_.reset(new smc_t);
        particle_filter_->setup(sample_set_,
                                uniform_sampling_,
                                normal_sampling_,
                                resampling_,
                                state_publisher_,
                                prediction_integrals_,
                                scheduler_,
                                reset_all_model_accumulators_on_update,
                                reset_all_model_accumulators_on_resampling,
                                enable_lag_correction && update_models_.size() > 1);

        /// prepare reset function to trigger uniform initialization
        particle_filter_reset_ = [this](const time_t& time) {
            particle_filter_->requestUniformInitialization(time);
        };
        for (auto &model : update_models_) {
            model.second->setResetFunction(particle_filter_reset_);
        }
    }

    predicition_forwarder_.reset(new prediction_relay_t(particle_filter_));
    data_provider_t::Ptr prediction_provider;
    MeshMapProvider::Ptr map_provider;
    if (!getPredictionDataProvider(prediction_provider) || !getPredictionMapProvider(map_provider)) {
        ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
        return false;
    }
    predicition_forwarder_->relay(prediction_model_, prediction_provider, map_provider);

    update_forwarder_.reset(new update_relay_t(particle_filter_/*, true*/));
    update_model_mapping_t update_mapping;
    if (!getUpdateModelProviderMapping(update_mapping)) {
        ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
        return false;
    }
    update_forwarder_->relay(update_mapping);

    return true;
}

void MuseARMCLOfflineNode::start()
{

    for (auto &d : data_providers_) {
        d.second->enable();
    }

    std::shared_ptr<std::vector<tf::StampedTransform>>   tf_transforms;
    std::size_t nv = data_set_->size();
    std::size_t count = 0;


    for(ContactEvaluationSample& seq : *data_set_){

        tf_transforms = std::make_shared<std::vector<tf::StampedTransform>>(seq.transforms);

        /// init map provider ...
        ROS_INFO_STREAM("Setting tf!");
        for(auto map : map_providers_){
            if(!map.second->initializeTF(*tf_transforms)){
                ROS_ERROR_STREAM("Couldn't set initial tf transfroms!");
                return;
            }
        }

        /// seq data
        if(seq.data.size() == 0) {
            ROS_ERROR_STREAM("Cannot work with an empty sequence!");
            continue;
        }

        state_publisher_->setData(seq.data);

        if (!particle_filter_->start()) {
            ROS_ERROR_STREAM("Couldn't start the filter!");
            return;
        }

        particle_filter_->requestUniformInitialization(time_t(seq.data.getMinTime()));

        /// itearte the sequence to test and tick the tf broadcaster
        std::size_t index = 0;
        for(ContactSample &s : seq.data) {
            for(auto &d : data_providers_) {
                if(d.second->isType<JointStateProvider>()) {
                    JointStateProvider &j = d.second->as<JointStateProvider>();
                    sensor_msgs::JointState::Ptr js(new sensor_msgs::JointState);
                    cslibs_kdl::JointStateStampedConversion::data2ros(s.state, *js);
                    /// use time stamp from map here
                    js->header.stamp.fromNSec(seq.data.getTimeFromIndex(index));
                    j.callback(js);
                 }
            }
            ++index;
        }
        ROS_INFO_STREAM("Send " << seq.data.size() << " messages");
        ros::Time expected;
        expected.fromNSec(seq.data.getMaxTime());
        ros::Time start;
        start.fromNSec(seq.data.getMinTime());
        ROS_INFO_STREAM(start << " to " << expected);

        double node_rate = nh_private_.param<double>("node_rate", 60.0);
        while(ros::ok()) {
            {
                std::unique_lock<std::mutex> l(filter_time_mutex_);
                if(expected == filter_time_) {
                    ROS_INFO_STREAM("Filter time reached.");
                    break;
                }
            }
            ros::spinOnce();
            if(node_rate > 0.0) {
                ros::WallRate(node_rate).sleep();
            }
        }

        state_publisher_->reset();
        state_publisher_->exportResults(results_file_base_name_);
        ROS_INFO_STREAM("processed: " << ++count << " of " << nv << " messages.");

        particle_filter_->end();
    }
}

bool MuseARMCLOfflineNode::getPredictionDataProvider(data_provider_t::Ptr &prediction_provider)
{
    const std::string param_name    = prediction_model_->getName() + "/data_provider";
    const std::string provider_name = nh_private_.param<std::string>(param_name, "");

    if (data_providers_.find(provider_name) == data_providers_.end()) {
        std::cerr << "[MuseARMCLOfflineNode]: Could not find data provider '" << provider_name
                  << "' for prediction model '" << prediction_model_->getName()
                  << "' !" << "\n";
        return false;
    }
    prediction_provider = data_providers_.at(provider_name);
    return true;
}

bool MuseARMCLOfflineNode::getPredictionMapProvider(MeshMapProvider::Ptr &map_provider)
{
    const std::string provider_name  = nh_private_.param<std::string>("map", ""); /// toplevel parameter

    if (map_providers_.find(provider_name) == map_providers_.end()) {
        std::cerr << "[MuseARMCLOfflineNode]: Could not find map provider '" << provider_name
                  << "' for prediction model '" << prediction_model_->getName()
                  << "' !" << "\n";
        return false;
    }
    map_provider = map_providers_.at(provider_name);
    return true;
}

bool MuseARMCLOfflineNode::getUpdateModelProviderMapping(update_model_mapping_t &update_mapping)
{
    for (auto &u : update_models_) {
        const std::string update_model_name = u.first;
        UpdateModel::Ptr update_model       = u.second;

        const std::string map_provider_param  = update_model_name + "/map_provider";
        const std::string data_provider_param = update_model_name + "/data_provider";
        const std::string map_provider_name   = nh_private_.param<std::string>(map_provider_param, "");
        const std::string data_provider_name  = nh_private_.param<std::string>(data_provider_param, "");
        if (map_provider_name == "") {
            ROS_ERROR_STREAM("No map provider name can be resolved for update model '" << update_model_name << "'!");
            return false;
        }
        if (data_provider_name == "") {
            ROS_ERROR_STREAM("No data provider name can be resolved for update model '" << update_model_name << "'!");
            return false;
        }
        if (map_providers_.find(map_provider_name) == map_providers_.end()) {
            ROS_ERROR_STREAM("Map Provider '" << map_provider_name << "' cannot be found for update model '" << update_model_name << "' !");
            return false;
        }
        if (data_providers_.find(data_provider_name) == data_providers_.end()) {
            ROS_ERROR_STREAM("Data Provider '" << data_provider_name << "' cannot be found for update model '" << update_model_name << "' !");
            return false;
        }

        MeshMapProvider::Ptr map_provider = map_providers_.at(map_provider_name);
        data_provider_t::Ptr data_provider = data_providers_.at(data_provider_name);
        update_mapping[update_model] = std::make_pair(data_provider, map_provider);
    }
    return true;
}

void MuseARMCLOfflineNode::setFilterTime(const cslibs_time::Time &t)
{
    std::unique_lock<std::mutex> l(filter_time_mutex_);
    uint64_t nsecs = static_cast<uint64_t>(t.nanoseconds());
    filter_time_ = ros::Time().fromNSec(nsecs);
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_armcl");

    muse_armcl::MuseARMCLOfflineNode node;
    if (node.setup()) {
        ROS_INFO_STREAM("Node is set up and ready to start!");
        node.start();
    } else {
        ROS_ERROR_STREAM("Could not set up the node!");
    }
    return 0;
}
