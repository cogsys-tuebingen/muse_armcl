#ifndef TRANSFORM_GRAPH_H
#define TRANSFORM_GRAPH_H

#include <tf/tf.h>
#include <memory>
#include <cslibs_math_3d/linear/transform.hpp>
#include <unordered_map>

class TransformGraph
{
public:
    TransformGraph();

    bool setup(const std::vector<tf::StampedTransform> &transforms);
    bool query(const std::string &target, const std::string &source, cslibs_math_3d::Transform3d &t_T_s);

private:
    struct EIGEN_ALIGN16 Node {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Ptr = std::shared_ptr<Node>;

        Ptr                         parent;
        std::map<std::string, Ptr>  children;
        std::string                 frame_id;
        cslibs_math_3d::Transform3d transform_to_parent;

        Node() = default;

        virtual ~Node()
        {
        }
    };

    Node::Ptr tree_;

};

#endif // TRANSFORM_GRAPH_H
