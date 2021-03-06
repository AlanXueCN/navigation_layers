#include<sonar_layer/sonar_layer.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(sonar_layer::SonarLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;

namespace sonar_layer
{

SonarLayer::SonarLayer() {}

void SonarLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = to_cost(0.5);
  phi_v_ = 1.2;
  max_angle_ = 12.5*M_PI/180;

  matchSize();
  min_x_ = min_y_ = -std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::max();

  std::string topic;
  nh.param("topic", topic, std::string("/sonar"));
  
  nh.param("clear_threshold", clear_threshold_, .2);
  nh.param("mark_threshold", mark_threshold_, .8);

  range_sub_ = nh.subscribe(topic, 100, &SonarLayer::incomingRange, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SonarLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  global_frame_ = layered_costmap_->getGlobalFrameID();
}


double SonarLayer::gamma(double theta)
{
    if(fabs(theta)>max_angle_)
        return 0.0;
    else
        return 1 - pow(theta/max_angle_, 2);
}

double SonarLayer::delta(double phi)
{
    return 1 - (1+tanh(2*(phi-phi_v_)))/2;
}

void SonarLayer::get_deltas(double angle, double *dx, double *dy)
{
    double ta = tan(angle);
    if(ta==0)
        *dx = 0;
    else
        *dx = resolution_ / ta;
    
    *dx = copysign(*dx, cos(angle));
    *dy = copysign(resolution_, sin(angle));
}

double SonarLayer::sensor_model(double r, double phi, double theta)
{
    double lbda = delta(phi)*gamma(theta);
    
    double delta = resolution_;
    
    if(phi >= 0.0 and phi < r - 2 * delta * r)
        return (1- lbda) * (0.5);
    else if(phi < r - delta * r)
        return lbda* 0.5 * pow((phi - (r - 2*delta*r))/(delta*r), 2)+(1-lbda)*.5;
    else if(phi < r + delta * r){
        double J = (r-phi)/(delta*r);
        return lbda * ((1-(0.5)*pow(J,2)) -0.5) + 0.5;
    }
    else
        return 0.5;
}


void SonarLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if(enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
  }
}

void SonarLayer::incomingRange(const sensor_msgs::RangeConstPtr& range)
{
  double r = range->range;
  if(r<range->min_range || r>range->max_range)
    return;
  max_angle_ = range->field_of_view/2;
  
  geometry_msgs::PointStamped in, out;
  in.header.stamp = range->header.stamp;
  in.header.frame_id = range->header.frame_id;  

  if(!tf_->waitForTransform(global_frame_, in.header.frame_id,
        in.header.stamp, ros::Duration(0.1)) ) {
     ROS_ERROR("Sonar layer can't transform from %s to %s at %f",
        global_frame_.c_str(), in.header.frame_id.c_str(),
        in.header.stamp.toSec());
     return;
  }
  
  tf_->transformPoint (global_frame_, in, out);
  
  double ox = out.point.x, oy = out.point.y;
  
  in.point.x = r;
  
  tf_->transformPoint(global_frame_, in, out);
  
  double tx = out.point.x, ty = out.point.y;
  
  // calculate target props
  double dx = tx-ox, dy = ty-oy,
        theta = atan2(dy,dx), d = sqrt(dx*dx+dy*dy);
  
  // Integer Bounds of Update
  int bx0, by0, bx1, by1;
  
  // Bounds includes the origin
  worldToMapNoBounds(ox, oy, bx0, by0);
  bx1 = bx0;
  by1 = by0;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update Map with Target Point
  unsigned int aa, ab;
  if(worldToMap(tx, ty, aa, ab)){
    setCost(aa, ab, 233);
    touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
  }
  
  double mx, my;
  int a, b;
  
  // Update left side of sonar cone
  mx = ox + cos(theta-max_angle_) * d * 1.2;
  my = oy + sin(theta-max_angle_) * d * 1.2;  
  worldToMapNoBounds(mx, my, a, b);
  bx0 = std::min(bx0, a);
  bx1 = std::max(bx1, a);
  by0 = std::min(by0, b);
  by1 = std::max(by1, b);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);
  
  // Update right side of sonar cone
  mx = ox + cos(theta+max_angle_) * d * 1.2;
  my = oy + sin(theta+max_angle_) * d * 1.2;
  
  worldToMapNoBounds(mx, my, a, b);
  bx0 = std::min(bx0, a);
  bx1 = std::max(bx1, a);
  by0 = std::min(by0, b);
  by1 = std::max(by1, b);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);
  
  // Limit Bounds to Grid  
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min((int)size_x_, bx1);
  by1 = std::min((int)size_y_, by1);
  
  for(unsigned int x=bx0; x<(unsigned int)bx1; x++){
    for(unsigned int y=by0; y<(unsigned int)by1; y++){
      double wx, wy;
      mapToWorld(x,y,wx,wy);
      update_cell(ox, oy, theta, r, wx, wy);
    }
  } 
  
  current_ = false;
}

void SonarLayer::update_cell(double ox, double oy, double ot, double r, double nx, double ny)
{
  unsigned int x, y;
  if(worldToMap(nx, ny, x, y)){
    double dx = nx-ox, dy = ny-oy;
    double theta = atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);
    double phi = sqrt(dx*dx+dy*dy);
    double sensor = sensor_model(r,phi,theta);
    double prior = to_prob(getCost(x,y));
    double prob_occ = sensor * prior;
    double prob_not = (1 - sensor) * (1 - prior);
    double new_prob = prob_occ/(prob_occ+prob_not);
    
    //ROS_INFO("%f %f | %f %f = %f", dx, dy, theta, phi, sensor);
    //ROS_INFO("%f | %f %f | %f", prior, prob_occ, prob_not, new_prob);
      unsigned char c = to_cost(new_prob);
      setCost(x,y,c);
  }
}

void SonarLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
 if (layered_costmap_->isRolling())
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  if (current_)
    return;

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);
  
  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::min();
  current_ = true;
}

void SonarLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = to_cost(clear_threshold_), mark = to_cost(mark_threshold_);

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      unsigned char prob = costmap_[it];
      unsigned char current;
      if(prob>mark)
        current = costmap_2d::LETHAL_OBSTACLE;
      else if(prob<clear)
        current = costmap_2d::FREE_SPACE;
      else{
        it++;
        continue;
      }
      
      unsigned char old_cost = master_array[it];
      
      if (old_cost == NO_INFORMATION || old_cost < current)
        master_array[it] = current;
      it++;
    }
  }
}

} // end namespace
