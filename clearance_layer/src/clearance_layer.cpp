#include<clearance_layer/clearance_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(clearance_layer::ClearanceLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace clearance_layer
{

ClearanceLayer::ClearanceLayer() {}

unsigned char ClearanceLayer::computeCost(double distance) const
{
    unsigned char cost = 0;
    if (distance == 0)
        cost = costmap_2d::LETHAL_OBSTACLE;
    else if (distance * resolution_ <= inscribed_radius_)
        cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    else
    {
        double euclidean_distance = distance * resolution_;
        double factor = weight_ / 100. * (1.0 - (euclidean_distance - inscribed_radius_) / (inflation_radius_ - inscribed_radius_));
        cost = (unsigned char)((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
}

} // end namespace
