#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {
  }

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  sub1= nh.subscribe("/set_obstacle_points", 1, & SimpleLayer::openpoints,this);
  ros::spinOnce();
  current_ = true;
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  // mark_x_ = origin_x + cos(origin_yaw);
  // mark_y_ = origin_y + sin(origin_yaw);
  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::openpoints(const geometry_msgs::PoseStamped &pose){
  ROS_INFO("i got the pose message");
  if(pose.header.frame_id=="obstacle"){
    pose_update = pose;
    std::cout<<pose_update.header.frame_id<<std::endl;
    mark_x_ = pose_update.pose.position.x;
    mark_y_ = pose_update.pose.position.y;
  }
  if(pose.header.frame_id=="obstacle_2"){
    pose_update = pose;
    std::cout<<pose_update.header.frame_id<<std::endl;
    mark_x_1 = pose_update.pose.position.x;
    mark_y_1 = pose_update.pose.position.y;
  }
  if(pose.header.frame_id=="obstacle_3"){
    pose_update = pose;
    std::cout<<pose_update.header.frame_id<<std::endl;
    mark_x_2 = pose_update.pose.position.x;
    mark_y_2 = pose_update.pose.position.y;
  }

  

}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
  if(master_grid.worldToMap(mark_x_1, mark_y_1, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
  if(master_grid.worldToMap(mark_x_2, mark_y_2, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }

  std::cout<<"world obs1:   "<<pose_update.pose.position.x<<"   "<<pose_update.pose.position.y<<std::endl;
  std::cout<<"map obs1:   "<<mx<<"   "<<my<<std::endl;
  
}
} // end namespace









