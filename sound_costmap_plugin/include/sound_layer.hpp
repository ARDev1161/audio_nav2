#pragma once

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "odas_classifier_msgs/msg/sound_source_classification.hpp"

namespace sound_costmap_plugin
{

class SoundLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SoundLayer();
  virtual void onInitialize() override;
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw,
                            double *min_x, double *min_y,
                            double *max_x, double *max_y) override;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                           int min_i, int min_j, int max_i, int max_j) override;
  virtual void reset() override;

  virtual bool isClearable() {return false;}  // depends on your logic

private:
  void soundCallback(const odas_classifier_msgs::msg::SoundSourceClassification::SharedPtr msg);

  rclcpp::Subscription<odas_classifier_msgs::msg::SoundSourceClassification>::SharedPtr sound_sub_;

  // Храним текущие опасные точки
  struct SoundPoint {
    double x;
    double y;
    double radius;
    bool avoid;
  };
  std::vector<SoundPoint> points_;

  double lethal_radius_; // param for how big area to mark as lethal
  rclcpp::Node::SharedPtr node_;
};

} // namespace sound_costmap_plugin
