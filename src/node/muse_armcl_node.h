#ifndef MUSE_ARMCL_NODE_H
#define MUSE_ARMCL_NODE_H

#include <ros/ros.h>

namespace muse_armcl {
class MuseARMCLNode
{
public:
    MuseARMCLNode();
    ~MuseARMCLNode();

    inline bool setup();
    inline void start();
};
}

#endif // MUSE_ARMCL_NODE_H
