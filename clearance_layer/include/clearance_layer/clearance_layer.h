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
  
  virtual unsigned char computeCost(double distance) const;

  
};
}
#endif

