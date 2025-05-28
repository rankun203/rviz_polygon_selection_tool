#pragma once

#include <rviz_common/panel.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/client.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <memory>

namespace rviz_polygon_selection_tool
{

class MacadamiaFarmPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  MacadamiaFarmPanel(QWidget* parent = nullptr);
  virtual ~MacadamiaFarmPanel();

  // Override Panel methods
  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

  // Set the polygon data to display
  void setPolygonData(const std::vector<geometry_msgs::msg::PolygonStamped>& polygons);

public Q_SLOTS:
  void updateTopic();
  void publishPolygons();
  void updatePolygonInfo();
  void checkNav2Status();
  void checkMapStatus();

private:
  // Calculate the area of a polygon
  double calculatePolygonArea(const geometry_msgs::msg::PolygonStamped& polygon);
  
  // Update polygon data from the tool
  void updateFromTool();

  // UI elements
  QLineEdit* topic_editor_;
  QPushButton* publish_button_;
  QLabel* navigation_status_label_;
  QLabel* localization_status_label_;
  QLabel* eta_label_;
  QLabel* distance_remaining_label_;
  QLabel* time_taken_label_;
  QLabel* polygon_area_label_;
  QPushButton* pause_button_;
  QPushButton* reset_button_;

  // ROS elements
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
  rclcpp::TimerBase::SharedPtr nav2_check_timer_;
  rclcpp::TimerBase::SharedPtr map_check_timer_;
  std::vector<geometry_msgs::msg::PolygonStamped> polygons_;
  std::string topic_;
};

} // namespace rviz_polygon_selection_tool
