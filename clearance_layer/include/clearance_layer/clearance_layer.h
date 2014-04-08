#ifndef CLEARANCE_LAYER_H_
#define CLEARANCE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace clearance_layer
{

class ClearanceLayer : public costmap_2d::InflationLayer
{
public:
  ClearanceLayer();
  
  virtual unsigned char computeCost(double distance) const
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
      //double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost = (unsigned char)((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }
  
};
}
#endif

