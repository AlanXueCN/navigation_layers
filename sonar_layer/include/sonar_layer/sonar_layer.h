#ifndef SONAR_LAYER_H_
#define SONAR_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <sensor_msgs/Range.h>
//#include <sonar_layer/SonarLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/costmap_2d_publisher.h>

namespace sonar_layer
{

class SonarLayer : public costmap_2d::CostmapLayer
{
public:
  SonarLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  void incomingRange(const sensor_msgs::RangeConstPtr& range);
  
  double gamma(double theta);
  double delta(double phi);
  double sensor_model(double r, double phi, double theta);
  
  void update_cell(double ox, double oy, double ot, double r, double nx, double ny);
  
  double to_prob(unsigned char c){ return double(c)/costmap_2d::LETHAL_OBSTACLE; }
  unsigned char to_cost(double p){ return (unsigned char)(p*costmap_2d::LETHAL_OBSTACLE); }
    
  std::string global_frame_;
  
  double clear_threshold_, mark_threshold_;

  //in the sonar model, the information content of a sonar reading drops off to 0 at this angle
  double max_angle_;
  double max_marking_angle_;
  //useful sonar reading distance (at this distance, the information used from the sonar reading drops by half)
  double phi_v_;

  //if this is nonzero, don't use any range greater than this to lower cell occupancy probs
  double max_clearing_range_;

  //if this is nonzero, don't use any range greater than this to raise cell occupancy probs
  double max_marking_range_;

  //If true, any cell whose sensor model prob is > 0.5 has its prior reset to 0.5 before updating
  bool reset_prior_for_marking_;

  //If true, any cell whose sensor model prob is < 0.5 has its prior reset to 0.5 before updating
  bool reset_prior_for_clearing_;

  // Flag for when new data is received (and we must do a costmap update)
  bool new_data_received_;

  //If true, the center of the sonar cone at the observed range will be marked lethal (within max_marking_range)
  bool mark_target_lethal_;

  ros::Subscriber range_sub_;
  double min_x_, min_y_, max_x_, max_y_;
  
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  //costmap layer publisher
  costmap_2d::Costmap2DPublisher* publisher_; 
};
}
#endif
