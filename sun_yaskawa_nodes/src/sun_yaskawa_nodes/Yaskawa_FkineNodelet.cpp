// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include "sun_yaskawa_nodes/Yaskawa_FkineNodelet.h"

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(sun_yaskawa_nodes::Yaskawa_FkineNodelet, nodelet::Nodelet)
