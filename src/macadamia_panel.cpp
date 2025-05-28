#include "macadamia_panel.h"
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
  , topic_("/publish_farming_area")
  , farming_status_("loading")
{
  // Create the main layout
  QVBoxLayout* main_layout = new QVBoxLayout;
  setLayout(main_layout);

  // Create a group box for the macadamia farm panel
  QGroupBox* navigation_group = new QGroupBox("Farming Robot Status");
  main_layout->addWidget(navigation_group);

  QGridLayout* grid_layout = new QGridLayout;
  navigation_group->setLayout(grid_layout);

  // Add status labels
  int row = 0;
  grid_layout->addWidget(new QLabel("Navigation (Nav 2):"), row, 0);
  navigation_status_label_ = new QLabel("active");
  navigation_status_label_->setStyleSheet("color: green; font-weight: bold;");
  grid_layout->addWidget(navigation_status_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Localization (SLAM Toolbox):"), row, 0);
  localization_status_label_ = new QLabel("inactive");
  localization_status_label_->setStyleSheet("color: gray;");
  grid_layout->addWidget(localization_status_label_, row++, 1);

  grid_layout->addWidget(new QLabel("ETA:"), row, 0);
  eta_label_ = new QLabel("0 s");
  grid_layout->addWidget(eta_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Distance remaining:"), row, 0);
  distance_remaining_label_ = new QLabel("0.00 m");
  grid_layout->addWidget(distance_remaining_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Time taken:"), row, 0);
  time_taken_label_ = new QLabel("0 s");
  grid_layout->addWidget(time_taken_label_, row++, 1);

  // Add task area information
  grid_layout->addWidget(new QLabel("Task Area:"), row, 0);
  polygon_area_label_ = new QLabel("Please draw the task area first");
  polygon_area_label_->setStyleSheet("color: #666666; font-style: italic;");
  grid_layout->addWidget(polygon_area_label_, row++, 1);

  grid_layout->addWidget(new QLabel("Task Status:"), row, 0);
  task_status_label_ = new QLabel("loading");
  task_status_label_->setStyleSheet("color: #666666;");
  grid_layout->addWidget(task_status_label_, row++, 1);

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

  publish_button_ = new QPushButton("Start Collecting Macadamia");
  publish_button_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px 16px;");
  main_layout->addWidget(publish_button_);

  // Connect signals and slots
  connect(topic_editor_, &QLineEdit::editingFinished, this, &MacadamiaFarmPanel::updateTopic);
  connect(publish_button_, &QPushButton::clicked, this, &MacadamiaFarmPanel::handleFarmingTaskControl);
  
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
  
  // Create publisher for farming action
  farming_action_pub_ = node->create_publisher<std_msgs::msg::String>("/farming_action", 10);
  
  // Subscribe to farming status topic
  farming_status_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/farming_status",
    10,
    std::bind(&MacadamiaFarmPanel::onFarmingStatusUpdate, this, std::placeholders::_1));
  
  // Set up timers for checking Nav2 and map status
  nav2_check_timer_ = node->create_wall_timer(
    std::chrono::seconds(10), // check every 10 seconds
    std::bind(&MacadamiaFarmPanel::checkNav2Status, this));
    
  map_check_timer_ = node->create_wall_timer(
    std::chrono::seconds(10), // check every 10 seconds
    std::bind(&MacadamiaFarmPanel::checkMapStatus, this));
  
  // Initial update from the tool
  updateFromTool();
}

void MacadamiaFarmPanel::onFarmingStatusUpdate(const std_msgs::msg::String::SharedPtr msg)
{
  farming_status_ = msg->data;
  task_status_label_->setText(QString::fromStdString(farming_status_));
  
  // Update button appearance based on farming status
  if (farming_status_ == "loading") {
    publish_button_->setText("System Loading");
    publish_button_->setStyleSheet("background-color: #cccccc; color: #666666; font-weight: bold; padding: 8px 16px;");
    publish_button_->setEnabled(false);
  } else if (farming_status_ == "idle" || farming_status_ == "task_done") {
    // Check if we have a valid polygon
    bool has_valid_polygon = false;
    for (const auto& polygon : polygons_) {
      if (polygon.polygon.points.size() >= 3) {
        has_valid_polygon = true;
        break;
      }
    }
    
    if (has_valid_polygon) {
      publish_button_->setText("Start Collecting Macadamia");
      publish_button_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px 16px;");
      publish_button_->setEnabled(true);
    } else {
      publish_button_->setText("Start Collecting Macadamia");
      publish_button_->setStyleSheet("background-color: #cccccc; color: #666666; font-weight: bold; padding: 8px 16px;");
      publish_button_->setEnabled(false);
    }
  } else if (farming_status_ == "task_accepted" || farming_status_ == "task_started" || farming_status_ == "task_failed") {
    publish_button_->setText("Cancel");
    publish_button_->setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 8px 16px;");
    publish_button_->setEnabled(true);
  }
}

void MacadamiaFarmPanel::checkNav2Status()
{
  // Check if Nav2 is available by checking for the existence of the global costmap topic
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  
  // Check for the global costmap topic
  std::string nav2_topic = "/global_costmap/costmap";
  
  // Check if the topic exists and has publishers
  bool nav2_available = false;
  
  // Get the list of topics
  auto topic_names_and_types = node->get_topic_names_and_types();
  
  for (const auto& topic_pair : topic_names_and_types) {
    const std::string& topic_name = topic_pair.first;
    if (topic_name.find(nav2_topic) != std::string::npos) {
      // Check if there are publishers for this topic
      size_t publisher_count = node->count_publishers(topic_name);
      if (publisher_count > 0) {
        nav2_available = true;
        break;
      }
    }
  }
  
  if (nav2_available) {
    navigation_status_label_->setText("active");
    navigation_status_label_->setStyleSheet("color: green; font-weight: bold;");
  } else {
    navigation_status_label_->setText("inactive");
    navigation_status_label_->setStyleSheet("color: gray;");
  }
}

void MacadamiaFarmPanel::checkMapStatus()
{
  // Check for map availability by checking for the existence of the map topic
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  
  // Check for the map topic
  std::string map_topic = "/map";
  
  // Check if the topic exists and has publishers
  bool map_available = false;
  
  // Get the list of topics
  auto topic_names_and_types = node->get_topic_names_and_types();
  
  for (const auto& topic_pair : topic_names_and_types) {
    const std::string& topic_name = topic_pair.first;
    if (topic_name.find(map_topic) != std::string::npos) {
      // Check if there are publishers for this topic
      size_t publisher_count = node->count_publishers(topic_name);
      if (publisher_count > 0) {
        map_available = true;
        break;
      }
    }
  }
  
  if (map_available) {
    localization_status_label_->setText("active");
    localization_status_label_->setStyleSheet("color: green; font-weight: bold;");
  } else {
    localization_status_label_->setText("inactive");
    localization_status_label_->setStyleSheet("color: gray;");
  }
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
  updateFarmingUIState();
}

void MacadamiaFarmPanel::updateFarmingUIState()
{
  double total_area = 0.0;
  bool has_valid_polygon = false;
  
  for (const auto& polygon : polygons_)
  {
    if (polygon.polygon.points.size() >= 3)
    {
      total_area += calculatePolygonArea(polygon);
      has_valid_polygon = true;
    }
  }
  
  // Update the area label
  if (has_valid_polygon) {
    polygon_area_label_->setText(QString::number(total_area, 'f', 2) + " m²");
    polygon_area_label_->setStyleSheet("color: black;");
  } else {
    polygon_area_label_->setText("Please draw the task area first");
    polygon_area_label_->setStyleSheet("color: #666666; font-style: italic;");
  }
  
  // Update button based on farming status and polygon validity
  if (farming_status_ == "loading") {
    publish_button_->setText("System Loading");
    publish_button_->setStyleSheet("background-color: #cccccc; color: #666666; font-weight: bold; padding: 8px 16px;");
    publish_button_->setEnabled(false);
  } else if (farming_status_ == "idle" || farming_status_ == "task_done") {
    publish_button_->setText("Start Collecting Macadamia");
    if (has_valid_polygon) {
      publish_button_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px 16px;");
      publish_button_->setEnabled(true);
    } else {
      publish_button_->setStyleSheet("background-color: #cccccc; color: #666666; font-weight: bold; padding: 8px 16px;");
      publish_button_->setEnabled(false);
    }
  } else if (farming_status_ == "task_accepted" || farming_status_ == "task_started" || farming_status_ == "task_failed") {
    publish_button_->setText("Cancel");
    publish_button_->setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 8px 16px;");
    publish_button_->setEnabled(true);
  }
}

void MacadamiaFarmPanel::handleFarmingTaskControl()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  
  // Check if we're in a task active state (task_accepted, task_started, task_failed)
  if (farming_status_ == "task_accepted" || farming_status_ == "task_started" || farming_status_ == "task_failed") {
    // Send a cancel command to the robot
    std_msgs::msg::String action_msg;
    action_msg.data = "cancel";
    farming_action_pub_->publish(action_msg);
    
    // Note: The actual status will be updated when we receive a new status message from the robot
    // We don't update farming_status_ here as it should be updated via the subscription callback
    
    return;
  }
  
  // Normal publish mode - start collecting macadamia
  if (!polygon_publisher_ || polygons_.empty())
  {
    return;
  }
  
  // First publish all valid polygons to define the farming area
  for (const auto& polygon : polygons_)
  {
    // Skip selections with fewer than 3 points
    if (polygon.polygon.points.size() < 3)
      continue;

    geometry_msgs::msg::PolygonStamped msg = polygon;
    msg.header.stamp = node->now();
    
    polygon_publisher_->publish(msg);
  }
  
  // Then send a start command to the robot
  std_msgs::msg::String action_msg;
  action_msg.data = "start";
  farming_action_pub_->publish(action_msg);
  
  // Note: The actual status will be updated when we receive a new status message from the robot
  // We don't update farming_status_ here as it should be updated via the subscription callback
}

void MacadamiaFarmPanel::startTask()
{
  // This method will handle the farming task control and potentially trigger other actions
  handleFarmingTaskControl();
  
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
