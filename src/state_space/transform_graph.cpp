#include <muse_armcl/state_space/transform_graph.h>

#include <queue>

#include <cslibs_math_ros/tf/conversion_3d.hpp>

TransformGraph::TransformGraph() :
    tree_(new Node)
{
}


bool TransformGraph::setup(const std::vector<tf::StampedTransform> &transforms)
{
    /// find root
    std::map<std::string, std::deque<tf::StampedTransform>> transform_map;
    std::map<std::string, std::size_t> appearance_as_child;
    std::set<std::string> children;

    auto add = [&transform_map, &children, &appearance_as_child](const tf::StampedTransform &t){
        transform_map[t.frame_id_].push_back(t);
        ++appearance_as_child[t.child_frame_id_];
        children.insert(t.child_frame_id_);
    };
    std::for_each(transforms.begin(), transforms.end(), add);

    std::string root = "";
    for(const auto &e : transform_map) {
        if(appearance_as_child[e.first] == 0) {
            if(root != "") {
                std::cerr << "Multiple roots found, therefore unconnected graph! \n";
                return false;
            }
            root = e.first;
        }
    }

    tree_->frame_id = root;
    std::queue<Node::Ptr> node_queue;
    node_queue.push(tree_);
    while(!node_queue.empty()) {
        Node::Ptr node = node_queue.front();
        node_queue.pop();
        std::deque<tf::StampedTransform> children = transform_map[node->frame_id];
        while(!children.empty()) {
            tf::StampedTransform ct = children.front();
            children.pop_front();

            Node::Ptr child_node(new Node);
            child_node->parent = node;
            child_node->frame_id = ct.child_frame_id_;
            child_node->transform_to_parent = cslibs_math_ros::tf::conversion_3d::from(ct);
            node->children[ct.child_frame_id_] = child_node;

            if(transform_map.find(ct.child_frame_id_) != transform_map.end())
                node_queue.push(child_node);
        }
        transform_map.erase(node->frame_id);
    }

    return true;
}

bool TransformGraph::query(const std::string &target, const std::string &source, cslibs_math_3d::Transform3d &t_T_s)
{
    auto find = [this](const std::string &name)
    {
        std::queue<Node::Ptr> node_queue;
        node_queue.push(tree_);
        while(!node_queue.empty()) {
            Node::Ptr node = node_queue.front();
            node_queue.pop();
            if(node->frame_id == name)
                return node;
            for(auto &c : node->children)
                node_queue.push(c.second);
        }
        return Node::Ptr();
    };

    auto backtrace = [](const Node::Ptr &n)
    {
        Node::Ptr node = n;
        cslibs_math_3d::Transform3d t;
        while(node->parent) {
            t = node->transform_to_parent * t;
            node = node->parent;
        }
        return t;
    };

    /// find source
    Node::Ptr node_src = find(source);
    if(!node_src) {
        std::cerr << "Source not found! \n";
        return false;
    }

    /// target
    Node::Ptr node_tgt = find(target);
    if(!node_tgt) {
        std::cerr << "Target not found! \n";
        return false;
    }

    cslibs_math_3d::Transform3d r_T_s = backtrace(node_src);
    cslibs_math_3d::Transform3d r_T_t = backtrace(node_tgt);

    t_T_s = r_T_t.inverse() * r_T_s;

    return true;
}
