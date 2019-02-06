#include <muse_armcl/state_space/state_publisher_offline.hpp>

using namespace muse_armcl;

void StatePublisherOffline::setup(ros::NodeHandle &nh, map_provider_map_t &map_providers, time_callback_t &set_time)
{
    n_sequences_ = 0;
    n_samples_ = 0;
    StatePublisher::setup(nh, map_providers);

    no_collision_label_ = nh.param<int>("no_collision_label", -1);
    vertex_gt_model_ = nh.param<bool>("vertex_gt_model", false);
    create_confusion_matrix_ = nh.param<bool>("create_confusion_matrix", true);
    use_force_threshold_ = nh.param<bool>("use_force_threshold", false);
    force_threshold_ = nh.param<double>("force_threshold", 0.1);
    set_time_ = set_time;
}

void StatePublisherOffline::publish(const typename sample_set_t::ConstPtr &sample_set)
{
    //        std::cout << "after resampling" << std::endl;

    StatePublisher::publish(sample_set);

    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    using mesh_map_tree_t = cslibs_mesh_map::MeshMapTree;
    using mesh_map_tree_node_t = cslibs_mesh_map::MeshMapTreeNode;

    if (!map_provider_) {
        std::cerr << "[StatePublisherOffline]: I have no map."  << std::endl;
        return;
    }
    if(!data_) {
        std::cerr << "[StatePublisherOffline]: I have no data." << std::endl;
        return;
    }

    //        std::cout << "evaluate" << std::endl;

    /// get the map
    const muse_smc::StateSpace<StateSpaceDescription>::ConstPtr ss = map_provider_->getStateSpace();
    if (!ss->isType<MeshMap>())
        return;
    const mesh_map_tree_t* map = ss->as<MeshMap>().data();

    // ground truth data
    uint64_t nsecs = static_cast<uint64_t>(sample_set->getStamp().nanoseconds());
    if(!data_->contains(nsecs)){
        return;
    }
    const ContactSample& gt = data_->at(nsecs);
    /*std::size_t sample_id = */data_->getID(nsecs);
    if(gt.state.torque.empty()) {
        std::cout << nsecs << std::endl;
        throw std::runtime_error("Empty data recieved");
    }

    double tau_norm =  gt.state.norm(cslibs_kdl_data::JointStateData::DataType::JOINT_TORQUE);
    cslibs_math_3d::Vector3d actual_pos;
    cslibs_math_3d::Vector3d actual_dir;

    DetectionResult event;
    event.true_point = gt.label;

    if(gt.label != no_collision_label_ && gt.label > 0){
        std::string actual_frame = getDiscreteContact(map, gt, actual_pos, actual_dir);
        auto node = map->getNode(actual_frame);
        if(node && vertex_gt_model_){
            std::size_t map_id = node->mapId();
            event.true_point = static_cast<int>(map_id) * 100000 + gt.label;

            cslibs_math_3d::Vector3d z(0,0,1);
            cslibs_math_3d::Vector3d  axis = z.cross(actual_dir);
            double alpha = std::acos(z.dot(actual_dir));
            cslibs_math_3d::Vector3d dir_local(gt.contact_force(0),
                                               gt.contact_force(1),
                                               gt.contact_force(2));
            dir_local = dir_local.normalized();
            cslibs_math_3d::Quaternion q(alpha, axis);
            actual_dir = q*dir_local;
        } else{
            std::cerr << "could not  set label probably." << std::endl;
        }
        cslibs_math_3d::Transform3d baseTactual = map->getTranformToBase(actual_frame);
        actual_pos = baseTactual * actual_pos;
        actual_dir = baseTactual * actual_dir;
    }


    event.contact_force_true = gt.contact_force.norm();
    event.error_dist = std::numeric_limits<double>::infinity();
    event.error_ori = std::numeric_limits<double>::infinity();
    event.contact_force = 0;
    event.phi = 0;
    event.s = 0;
    event.closest_point = no_collision_label_;

    auto distanceMetric = [actual_pos, actual_dir](const cslibs_math_3d::Vector3d& p,
            const cslibs_math_3d::Vector3d& v,
            double& e_dist,
            double& e_ori)
    {
        e_dist = (p - actual_pos).length();
        double tmp = v.dot(actual_dir) / v.length() / actual_dir.length();
        e_ori = std::acos(tmp);
    };
    ++n_samples_;
    /// density estimation
    ContactPointHistogram::ConstPtr histogram = std::dynamic_pointer_cast<ContactPointHistogram const>(sample_set->getDensity());
    if(histogram && !labeled_contact_points_.empty()){
        std::vector<std::pair<int,double>> labels;
        if (tau_norm > no_contact_torque_threshold_){
            histogram->getTopLabels(labels);
            if(!labels.empty()){
                if(!use_force_threshold_ || (std::fabs(labels.front().second) > force_threshold_)){
                    event.closest_point = labels.front().first;
                    event.contact_force = labels.front().second;
                    cslibs_math_3d::Vector3d detected_pos;
                    cslibs_math_3d::Vector3d detected_dir;
                    std::string detected_frame = getDiscreteContact(event.closest_point, detected_pos, detected_dir);
                    cslibs_math_3d::Transform3d baseTdetect = map->getTranformToBase(detected_frame);
                    detected_pos = baseTdetect * detected_pos;
                    detected_pos = baseTdetect * detected_pos;
                    distanceMetric(detected_pos, detected_dir, event.error_dist, event.error_ori);
                }
            }
        }
    } else {
        /// density estimation
        SampleDensity::ConstPtr density = std::dynamic_pointer_cast<SampleDensity const>(sample_set->getDensity());
        std::vector<StateSpaceDescription::sample_t, StateSpaceDescription::sample_t::allocator_t> states;
        density->contacts(states);

        for (const StateSpaceDescription::sample_t& p : states) {
            const mesh_map_tree_node_t* p_map = map->getNode(p.state.map_id);
            cslibs_math_3d::Transform3d baseTpred= map->getTranformToBase(p_map->frameId());
            if (p_map && (tau_norm > no_contact_torque_threshold_)) {
                if(use_force_threshold_ && (std::fabs(p.state.force) < force_threshold_)){
                    continue;
                }
                std::string link = p_map->frameId();
                cslibs_math_3d::Vector3d point = baseTpred * p.state.getPosition(p_map->map);
                cslibs_math_3d::Vector3d direction = baseTpred * p.state.getDirection(p_map->map);
                int prediction = -1;
                if(vertex_gt_model_){
                    std::size_t vid = p.state.s > 0.5 ? p.state.goal_vertex.idx() : p.state.active_vertex.idx();
                    prediction = p_map->mapId() * 100000 + vid;
                } else {
                    prediction = getClosetPoint(link, point);
                }
                double dist_error;
                double angle;
                distanceMetric(point, direction, dist_error, angle);
                if(dist_error < event.error_dist){
                    event.contact_force = p.state.force;
                    event.closest_point = prediction;
                    event.error_dist = dist_error;
                    event.error_ori = angle;
                }
            }
        }
    }


    //        std::cout << "[" << nsecs << "]" << "(" << sample_id << ")" << " torque res: " << tau_norm << " gt label: "<< gt.label
    //                  << " detected: " << event.closest_point << std::endl;
    if(create_confusion_matrix_){
        confusion_matrix_.reportClassification(gt.label, event.closest_point);
    }
    results_.push_back(event);

    if(tau_norm > no_contact_torque_threshold_){
        reportLikelyHoodOfGt(sample_set, gt, map);
    }

    set_time_(sample_set->getStamp());

}

void StatePublisherOffline::publishIntermediate(const typename sample_set_t::ConstPtr &sample_set)
{
    //        std::cout << "intermediate" << "\n";
    //        StatePublisher::publishIntermediate(sample_set);
    publish(sample_set);
    //        set_time_(sample_set->getStamp());
}

void StatePublisherOffline::publishConstant(const typename sample_set_t::ConstPtr &sample_set)
{
    //        std::cout << "constant" << "\n";
    //        StatePublisher::publishConstant(sample_set);
    publish(sample_set);
    //        set_time_(sample_set->getStamp());
}

void StatePublisherOffline::setData(const ContactSequence& data)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    data_ = &data;
}

void StatePublisherOffline::reset()
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    data_ = nullptr;
}

int StatePublisherOffline::getClosetPoint(const std::string& frame_id, const cslibs_math_3d::Vector3d& estimated) const
{
    KDL::Vector pos (estimated(0), estimated(1), estimated(2));
    std::pair<int,double> min;
    min.first = -1;
    min.second = std::numeric_limits<double>::infinity();
    for(const auto& t : labeled_contact_points_){
        if(t.second.parent == frame_id){
            int label = t.first;
            double dist = (t.second.frame.p - pos).Norm();
            if(dist < min.second){
                min.first = label;
                min.second = dist;
            }
        }
    }
    //        std::cout << "get point" << std::endl;
    return min.first;
}

std::string StatePublisherOffline::getDiscreteContact(const cslibs_mesh_map::MeshMapTree* map,
                                                      const ContactSample &gt,
                                                      cslibs_math_3d::Vector3d& position,
                                                      cslibs_math_3d::Vector3d& direction) const
{   
    if(vertex_gt_model_ && gt.label > 0){
        std::size_t vertex_id = static_cast<std::size_t>(gt.label);
        const cslibs_mesh_map::MeshMapTreeNode* gt_node = map->getNode(gt.contact_force.frameId());
        if(gt_node){
            cslibs_mesh_map::MeshMap::VertexHandle vh = gt_node->map.vertexHandle(vertex_id);
            position = gt_node->map.getPoint(vh);
            direction = gt_node->map.getNormal(vh) * (-1.0);
            return gt_node->frameId();
        }
        std::cerr << "Did not find ground truth point "<< gt.label << " frame_id: " << gt.contact_force.frameId() << std::endl;
        //        return map->front()->frameId();
        throw std::runtime_error("Did not find ground truth point");
    } else {
        return getDiscreteContact(gt.label, position, direction);
    }
}

std::string StatePublisherOffline::getDiscreteContact(int label,
                                                      cslibs_math_3d::Vector3d& position,
                                                      cslibs_math_3d::Vector3d& direction) const
{
    try{
        const cslibs_kdl::KDLTransformation& t = labeled_contact_points_.at(label);


        position(0) = t.frame.p.x();
        position(1) = t.frame.p.y();
        position(2) = t.frame.p.z();
        KDL::Vector dir = t.frame.M * KDL::Vector(-1,0,0);
        direction(0) = dir.x();
        direction(1) = dir.y();
        direction(2) = dir.z();
        return t.parent;
    } catch(const std::out_of_range& ex){
        std::cerr << ex.what() << "labled_contact_points at label " << label << ". #(labled contact points)" << labeled_contact_points_.size();
        throw ex;
    }
}

void StatePublisherOffline::exportResults(const std::string& path)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    std::string file_cm = path + "_confusion_matrix.csv";
    std::string file_ds = path + "_detection_results.csv";
    std::string file_gt = path + "_gt_likely_hood.csv";
    std::string file_p  = path + "_process_progress.txt";
    if(create_confusion_matrix_){
        confusion_matrix_.exportCsv(file_cm);
    }
    save(results_, file_ds);
    save(gt_likely_hood_, file_gt);
    ++n_sequences_;
    std::ofstream of(file_p);
    of << "Processed sequences: " << n_sequences_ << std::endl;
    of << "Processed samples:   " << n_samples_ << std::endl;
    of.close();

}

void StatePublisherOffline::reportLikelyHoodOfGt(const typename sample_set_t::ConstPtr &sample_set,
                                                 const ContactSample& gt,
                                                 const cslibs_mesh_map::MeshMapTree* map)
{
    if(gt.label == no_collision_label_){
        return;
    }
    //    const cslibs_kdl::KDLTransformation& true_contact_point = getLabledPoint(gt.label);
    //    const KDL::Vector& pos_t = true_contact_point.frame.p;
    //    KDL::Vector direction = true_contact_point.frame.M * KDL::Vector(-1,0,0);
    cslibs_math_3d::Vector3d true_point;
    cslibs_math_3d::Vector3d true_dir;
    std::string true_point_frame_id = getDiscreteContact(map, gt, true_point, true_dir);
    cslibs_math_3d::Transform3d b_T_cp = map->getTranformToBase(true_point_frame_id);
    true_point = b_T_cp * true_point;
    true_dir = b_T_cp * true_dir;

    GroundTruthParticleSetDistance d;
    d.distance    = std::numeric_limits<double>::max();
    d.angle       = std::numeric_limits<double>::max();
    d.likely_hood = 0;
    d.contact_force = 0;
    d.contact_force_true = gt.contact_force.norm();
    d.link = 0;
    d.true_point = gt.label;
    auto gt_map = map->getNode(true_point_frame_id);
    if(gt_map){
        d.link = static_cast<int>(gt_map->mapId());
    }
    for (const StateSpaceDescription::sample_t& p : sample_set->getSamples()) {
        const cslibs_mesh_map::MeshMapTreeNode* p_map = map->getNode(p.state.map_id);
        if (p_map) {
            cslibs_math_3d::Transform3d T = map->getTranformToBase(p_map->map.frame_id_);
            cslibs_math_3d::Vector3d pos = p.state.getPosition(p_map->map);
            pos = T * pos;
            double dist = (true_point - pos).length2();
            if(dist < d.distance){
                cslibs_math_3d::Vector3d dir = p.state.getDirection(p_map->map);
                d.distance = dist;
                d.likely_hood = p.state.last_update;
                d.contact_force = p.state.force;
                d.angle = cslibs_math::linear::angle(true_dir, dir);
                d.link = static_cast<int>(p.state.map_id);
            }
        }
    }
    d.distance = std::sqrt(d.distance);
    gt_likely_hood_.emplace_back(d);

}

const cslibs_kdl::KDLTransformation& StatePublisherOffline::getLabledPoint(int label) const
{
    return labeled_contact_points_.at(label);
}
