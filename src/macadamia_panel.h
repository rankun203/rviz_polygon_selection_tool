#pragma once

#include <rviz_common/panel.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/subscription.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <std_msgs/msg/string.hpp>
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
  void handleFarmingTaskControl();
  void updateFarmingUIState();
  void checkNav2Status();
  void checkMapStatus();
  void startTask();
  void onFarmingStatusUpdate(const std_msgs::msg::String::SharedPtr msg);

private:
  // Calculate the area of a polygon
  double calculatePolygonArea(const geometry_msgs::msg::PolygonStamped& polygon);
  
  // Update polygon data from the tool
  void updateFromTool();
  
  // Topic name for publishing polygon selections
  std::string topic_;
  
  // Current farming status
  std::string farming_status_;

  // UI elements
  QLineEdit* topic_editor_;
  QPushButton* publish_button_;
  QLabel* navigation_status_label_;
  QLabel* localization_status_label_;
  QLabel* eta_label_;
  QLabel* distance_remaining_label_;
  QLabel* time_taken_label_;
  QLabel* polygon_area_label_;
  QLabel* task_status_label_;
  QPushButton* pause_button_;
  QPushButton* reset_button_;

  // ROS elements
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr farming_action_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr farming_status_sub_;
  rclcpp::TimerBase::SharedPtr nav2_check_timer_;
  rclcpp::TimerBase::SharedPtr map_check_timer_;
  std::vector<geometry_msgs::msg::PolygonStamped> polygons_;
};

} // namespace rviz_polygon_selection_tool
