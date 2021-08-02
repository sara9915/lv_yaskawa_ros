#include "sun_robot_ros/FkineNodelet.h"
#include "sun_yaskawa_nodes/Yaskawa_FkineNode.h"

namespace sun_yaskawa_nodes {
class Yaskawa_FkineNodelet : public sun::FkineNodelet<sun::Yaskawa_FkineNode> {
};
} // namespace sun_yaskawa_nodes