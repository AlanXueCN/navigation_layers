#include<clearance_layer/clearance_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(clearance_layer::ClearanceLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace clearance_layer
{

ClearanceLayer::ClearanceLayer() {}


} // end namespace
