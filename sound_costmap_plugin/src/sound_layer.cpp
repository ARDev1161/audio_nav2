#include "sound_layer.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(sound_costmap_plugin::SoundLayer, nav2_costmap_2d::Layer)

namespace sound_costmap_plugin
{

SoundLayer::SoundLayer()
{
}

void SoundLayer::onInitialize()
{
  // Нужно, чтобы layer имел rclcpp::Node (через protected переменную node_ или свой)
  node_ = rclcpp::Node::make_shared("sound_layer_node");

  declareParameter("lethal_radius", rclcpp::ParameterValue(1.0));
  node_->get_parameter_or<double>("lethal_radius", lethal_radius_, 1.0);

  sound_sub_ = node_->create_subscription<odas_classifier_msgs::msg::SoundSourceClassification>(
    "/sound_sources", 10,
    std::bind(&SoundLayer::soundCallback, this, std::placeholders::_1)
  );

  current_ = true; // costmap2d layer flag
}

void SoundLayer::soundCallback(const odas_classifier_msgs::msg::SoundSourceClassification::SharedPtr msg)
{
  // Пример: если msg->label в ["Explosion","Gunshot"] => avoid
  bool is_avoid = false;
  if(msg->label == "Explosion" || msg->label == "Gunshot") {
    is_avoid = true;
  }

  // Пример: радиус опасной зоны (может зависеть от энергии)
  double radius = lethal_radius_;

  SoundPoint sp;
  sp.x = msg->x; // координаты в той же системе, что costmap?
  sp.y = msg->y;
  sp.radius = radius;
  sp.avoid = is_avoid;

  // Добавляем в вектор
  // (для реальной системы, возможно, обновляете существующий, храните временные метки и т.д.)
  points_.push_back(sp);
}

void SoundLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                              double *min_x, double *min_y,
                              double *max_x, double *max_y)
{
  // Расширяем границы обновления costmap, чтобы учесть новые точки
  for (auto &p : points_) {
    *min_x = std::min(*min_x, p.x - p.radius);
    *min_y = std::min(*min_y, p.y - p.radius);
    *max_x = std::max(*max_x, p.x + p.radius);
    *max_y = std::max(*max_y, p.y + p.radius);
  }
}

void SoundLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                             int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  // Пройти по всем точкам, где avoid == true => ставим LETHAL
  for (auto &p : points_) {
    if (!p.avoid) {
      continue;
    }

    // Преобразовать (p.x, p.y) в индексы costmap
    unsigned int mx, my;
    if (master_grid.worldToMap(p.x, p.y, mx, my)) {
      // Обойти окрестности по радиусу
      double rr = p.radius * p.radius;
      int r = static_cast<int>(p.radius / master_grid.getResolution());
      for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
          int nx = mx + dx;
          int ny = my + dy;
          if (nx < 0 || ny < 0) continue;
          if ((unsigned int)nx >= master_grid.getSizeInCellsX() ||
              (unsigned int)ny >= master_grid.getSizeInCellsY()) continue;
          double dist = dx * dx + dy * dy;
          if (dist <= rr) {
            // Устанавливаем стоимость
            master_grid.setCost(nx, ny, nav2_costmap_2d::LETHAL_OBSTACLE);
          }
        }
      }
    }
  }
}

void SoundLayer::reset()
{
  // Сбрасываем данные
  points_.clear();
  current_ = true;
}

} // namespace sound_costmap_plugin
