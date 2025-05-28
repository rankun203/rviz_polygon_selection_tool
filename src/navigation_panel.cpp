#include "navigation_panel.h"
#include "rviz_polygon_selection_tool.h"

#include <QGridLayout>
#include <QGroupBox>
#include <QTimer>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>

namespace rviz_polygon_selection_tool
{

MacadamiaFarmPanel::MacadamiaFarmPanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , topic_("/polygon_selection")
{
  // Create the main layout
  QVBoxLayout* main_layout = new QVBoxLayout;
  setLayout(main_layout);

  // Create a group box for the macadamia farm panel
  QGroupBox* navigation_group = new QGroupBox("Macadamia Farm");
  main_layout->addWidget(navigation_group);

  QGridLayout* grid_layout = new QGridLayout;
  navigation_group->setLayout(grid_layout);

  // Add status labels
  int row = 0;
  grid_layout->addWidget(new QLabel("Navigation:"), row, 0);
  navigation_status_label_ = new QLabel("active");
  navigation_status_label_->setStyleSheet("color: green; font-weight: bold;");
  grid_layout->addWidget(navigation_status_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Localization:"), row, 0);
  localization_status_label_ = new QLabel("inactive");
  localization_status_label_->setStyleSheet("color: gray;");
  grid_layout->addWidget(localization_status_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Feedback:"), row, 0);
  feedback_status_label_ = new QLabel("unknown");
  grid_layout->addWidget(feedback_status_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Poses remaining:"), row, 0);
  poses_remaining_label_ = new QLabel("0");
  grid_layout->addWidget(poses_remaining_label_, row++, 1);

  grid_layout->addWidget(new QLabel("ETA:"), row, 0);
  eta_label_ = new QLabel("0 s");
  grid_layout->addWidget(eta_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Distance remaining:"), row, 0);
  distance_remaining_label_ = new QLabel("0.00 m");
  grid_layout->addWidget(distance_remaining_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Time taken:"), row, 0);
  time_taken_label_ = new QLabel("0 s");
  grid_layout->addWidget(time_taken_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Recoveries:"), row, 0);
  recoveries_label_ = new QLabel("0");
  grid_layout->addWidget(recoveries_label_, row++, 1);

  // Add polygon area information
  grid_layout->addWidget(new QLabel("Polygon Area:"), row, 0);
  polygon_area_label_ = new QLabel("0.00 m²");
  grid_layout->addWidget(polygon_area_label_, row++, 1);

  // Add buttons
  start_task_button_ = new QPushButton("Start Task");
  start_task_button_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px 16px;");
  grid_layout->addWidget(start_task_button_, row++, 0, 1, 2);

  pause_button_ = new QPushButton("Pause");
  grid_layout->addWidget(pause_button_, row++, 0, 1, 2);

  reset_button_ = new QPushButton("Reset");
  grid_layout->addWidget(reset_button_, row++, 0, 1, 2);

  // Add topic selection and publish button
  QHBoxLayout* topic_layout = new QHBoxLayout;
  main_layout->addLayout(topic_layout);

  topic_layout->addWidget(new QLabel("Topic:"));
  topic_editor_ = new QLineEdit(QString::fromStdString(topic_));
  topic_layout->addWidget(topic_editor_);

  publish_button_ = new QPushButton("Publish Polygons");
  publish_button_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px 16px;");
  main_layout->addWidget(publish_button_);

  // Connect signals and slots
  connect(topic_editor_, &QLineEdit::editingFinished, this, &MacadamiaFarmPanel::updateTopic);
  connect(publish_button_, &QPushButton::clicked, this, &MacadamiaFarmPanel::publishPolygons);
  connect(start_task_button_, &QPushButton::clicked, this, &MacadamiaFarmPanel::startTask);
  
  // Create a timer to update the polygon information periodically
  QTimer* update_timer = new QTimer(this);
  connect(update_timer, &QTimer::timeout, this, &MacadamiaFarmPanel::updateFromTool);
  update_timer->start(500); // Update every 500ms
}

MacadamiaFarmPanel::~MacadamiaFarmPanel()
{
}

void MacadamiaFarmPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  polygon_publisher_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_, 10);
  
  // Initial update from the tool
  updateFromTool();
}

void MacadamiaFarmPanel::updateFromTool()
{
  // Find the PolygonSelectionTool
  PolygonSelectionTool* tool = nullptr;
  if (getDisplayContext() && getDisplayContext()->getToolManager())
  {
    for (int i = 0; i < getDisplayContext()->getToolManager()->numTools(); i++)
    {
      auto current_tool = dynamic_cast<PolygonSelectionTool*>(getDisplayContext()->getToolManager()->getTool(i));
      if (current_tool)
      {
        tool = current_tool;
        break;
      }
    }
  }
  
  // If we found the tool, update our polygon data
  if (tool)
  {
    setPolygonData(tool->getPolygonData());
  }
}

void MacadamiaFarmPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic))
  {
    topic_editor_->setText(topic);
    updateTopic();
  }
}

void MacadamiaFarmPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_editor_->text());
}

void MacadamiaFarmPanel::updateTopic()
{
  topic_ = topic_editor_->text().toStdString();
  
  // Recreate the publisher with the new topic
  if (getDisplayContext())
  {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    polygon_publisher_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_, 10);
  }
}

void MacadamiaFarmPanel::setPolygonData(const std::vector<geometry_msgs::msg::PolygonStamped>& polygons)
{
  polygons_ = polygons;
  updatePolygonInfo();
}

void MacadamiaFarmPanel::updatePolygonInfo()
{
  double total_area = 0.0;
  
  for (const auto& polygon : polygons_)
  {
    if (polygon.polygon.points.size() >= 3)
    {
      total_area += calculatePolygonArea(polygon);
    }
  }
  
  // Update the area label
  polygon_area_label_->setText(QString::number(total_area, 'f', 2) + " m²");
}

void MacadamiaFarmPanel::publishPolygons()
{
  if (!polygon_publisher_ || polygons_.empty())
  {
    return;
  }

  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  
  // Publish all valid polygons
  for (const auto& polygon : polygons_)
  {
    // Skip selections with fewer than 3 points
    if (polygon.polygon.points.size() < 3)
      continue;

    geometry_msgs::msg::PolygonStamped msg = polygon;
    msg.header.stamp = node->now();
    
    polygon_publisher_->publish(msg);
  }
}

void MacadamiaFarmPanel::startTask()
{
  // This method will publish the polygons and potentially trigger other actions
  publishPolygons();
  
  // Update the UI to indicate the task has started
  navigation_status_label_->setText("active");
  navigation_status_label_->setStyleSheet("color: green; font-weight: bold;");
  
  // You could add additional functionality here, such as:
  // - Starting a timer
  // - Enabling/disabling certain UI elements
  // - Sending additional ROS messages
}

double MacadamiaFarmPanel::calculatePolygonArea(const geometry_msgs::msg::PolygonStamped& polygon)
{
  // Calculate the area of a polygon using the Shoelace formula
  double area = 0.0;
  size_t n = polygon.polygon.points.size();
  
  for (size_t i = 0; i < n; i++)
  {
    size_t j = (i + 1) % n;
    area += polygon.polygon.points[i].x * polygon.polygon.points[j].y;
    area -= polygon.polygon.points[j].x * polygon.polygon.points[i].y;
  }
  
  area = std::abs(area) / 2.0;
  return area;
}

} // namespace rviz_polygon_selection_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_selection_tool::MacadamiaFarmPanel, rviz_common::Panel)
