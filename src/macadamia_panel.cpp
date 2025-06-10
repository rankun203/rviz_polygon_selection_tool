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
  , farming_status_("inactive")
  , task_area_confirmed_(false)
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
  task_status_label_ = new QLabel("inactive");
  task_status_label_->setStyleSheet("color: gray;");
  grid_layout->addWidget(task_status_label_, row++, 1);

  // Add grid size configuration
  grid_layout->addWidget(new QLabel("Cell Size:"), row, 0);
  QHBoxLayout* grid_size_layout = new QHBoxLayout;
  grid_size_editor_ = new QLineEdit("0.5");
  grid_size_editor_->setMaximumWidth(80);
  grid_size_label_ = new QLabel("m");
  grid_size_layout->addWidget(grid_size_editor_);
  grid_size_layout->addWidget(grid_size_label_);
  grid_size_layout->addStretch();
  
  QWidget* grid_size_widget = new QWidget;
  grid_size_widget->setLayout(grid_size_layout);
  grid_layout->addWidget(grid_size_widget, row++, 1);

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
  connect(grid_size_editor_, &QLineEdit::editingFinished, this, &MacadamiaFarmPanel::onGridSizeChanged);
  
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
  
  // Create publisher for farming configuration
  farming_config_pub_ = node->create_publisher<std_msgs::msg::String>("/farming_config", 10);
  
  // Subscribe to farming status topic
  farming_status_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/farming_status",
    10,
    std::bind(&MacadamiaFarmPanel::onFarmingStatusUpdate, this, std::placeholders::_1));
  
  // Set up timers for checking Nav2, map, and farming status
  nav2_check_timer_ = node->create_wall_timer(
    std::chrono::seconds(10), // check every 10 seconds
    std::bind(&MacadamiaFarmPanel::checkNav2Status, this));
    
  map_check_timer_ = node->create_wall_timer(
    std::chrono::seconds(10), // check every 10 seconds
    std::bind(&MacadamiaFarmPanel::checkMapStatus, this));
    
  farming_status_check_timer_ = node->create_wall_timer(
    std::chrono::seconds(10), // check every 10 seconds
    std::bind(&MacadamiaFarmPanel::checkFarmingStatus, this));
  
  // Initial update from the tool
  updateFromTool();
}

// Helper method to check if a topic exists and has publishers
bool MacadamiaFarmPanel::isTopicAvailable(const std::string& topic_name, bool exact_match)
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  auto topic_names_and_types = node->get_topic_names_and_types();
  
  for (const auto& topic_pair : topic_names_and_types) {
    const std::string& current_topic = topic_pair.first;
    bool match = exact_match ? 
                 (current_topic == topic_name) : 
                 (current_topic.find(topic_name) != std::string::npos);
    
    if (match) {
      // Check if there are publishers for this topic
      size_t publisher_count = node->count_publishers(current_topic);
      if (publisher_count > 0) {
        return true;
      }
    }
  }
  
  return false;
}

void MacadamiaFarmPanel::checkFarmingStatus()
{
  // Check if the farming_status topic exists
  std::string farming_status_topic = "/farming_status";
  bool farming_status_available = isTopicAvailable(farming_status_topic, true);
  
  // If farming_status topic is not available, set task status to inactive
  if (!farming_status_available) {
    farming_status_ = "inactive";
    task_status_label_->setText("inactive");
    task_status_label_->setStyleSheet("color: gray;");
  }
  // If farming_status topic is available and current status is inactive, 
  // change to loading until we receive an actual status update
  else if (farming_status_ == "inactive") {
    farming_status_ = "loading";
    task_status_label_->setText("loading");
    task_status_label_->setStyleSheet("color: #666666;");
  }
}

void MacadamiaFarmPanel::onFarmingStatusUpdate(const std_msgs::msg::String::SharedPtr msg)
{
  // Only update if we received a valid message
  farming_status_ = msg->data;
  task_status_label_->setText(QString::fromStdString(farming_status_));
  
  // Update button appearance based on farming status
  if (farming_status_ == "inactive") {
    publish_button_->setText("System Inactive");
    publish_button_->setStyleSheet("background-color: #cccccc; color: #666666; font-weight: bold; padding: 8px 16px;");
    publish_button_->setEnabled(false);
  } else if (farming_status_ == "loading") {
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
  std::string nav2_topic = "/global_costmap/costmap";
  bool nav2_available = isTopicAvailable(nav2_topic, false);
  
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
  std::string map_topic = "/map";
  bool map_available = isTopicAvailable(map_topic, false);
  
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
  // Only update polygon data if task area hasn't been confirmed yet
  if (!task_area_confirmed_) {
    polygons_ = polygons;
  }
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
    polygon_area_label_->setText(QString::number(total_area, 'f', 2) + " m² (not confirmed)");
    polygon_area_label_->setStyleSheet("color: #666666; font-style: italic;");
  } else if (task_area_confirmed_) {
    polygon_area_label_->setText("Task area confirmed");
    polygon_area_label_->setStyleSheet("color: green; font-weight: bold;");
  } else {
    polygon_area_label_->setText("Please draw the task area first");
    polygon_area_label_->setStyleSheet("color: #666666; font-style: italic;");
  }
  
  // Update button based on farming status and polygon validity
  if (farming_status_ == "inactive") {
    publish_button_->setText("System Inactive");
    publish_button_->setStyleSheet("background-color: #cccccc; color: #666666; font-weight: bold; padding: 8px 16px;");
    publish_button_->setEnabled(false);
  } else if (farming_status_ == "loading") {
    publish_button_->setText("System Loading");
    publish_button_->setStyleSheet("background-color: #cccccc; color: #666666; font-weight: bold; padding: 8px 16px;");
    publish_button_->setEnabled(false);
  } else if (farming_status_ == "idle" || farming_status_ == "task_done") {
    // Check if we have an unconfirmed polygon selection
    if (has_valid_polygon) {
      publish_button_->setText("Confirm Task Area");
      publish_button_->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 8px 16px;");
      publish_button_->setEnabled(true);
    } else if (task_area_confirmed_) {
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
  
  // Check if we have a valid polygon to confirm
  bool has_valid_polygon = false;
  for (const auto& polygon : polygons_) {
    if (polygon.polygon.points.size() >= 3) {
      has_valid_polygon = true;
      break;
    }
  }
  
  // Step 1: Confirm task area (publish polygon and clear selection)
  if (has_valid_polygon && !task_area_confirmed_) {
    if (!polygon_publisher_) {
      return;
    }
    
    // Publish all valid polygons to define the farming area
    for (const auto& polygon : polygons_) {
      // Skip selections with fewer than 3 points
      if (polygon.polygon.points.size() < 3)
        continue;

      geometry_msgs::msg::PolygonStamped msg = polygon;
      msg.header.stamp = node->now();
      
      polygon_publisher_->publish(msg);
    }
    
    // Mark task area as confirmed and clear the polygon selection
    task_area_confirmed_ = true;
    
    // Clear local polygons (the UI will show confirmed state)
    polygons_.clear();
    
    // Update UI state
    updateFarmingUIState();
    
    return;
  }
  
  // Step 2: Start collecting macadamia (task area already confirmed)
  if (task_area_confirmed_ && !has_valid_polygon) {
    // Send a start command to the robot
    std_msgs::msg::String action_msg;
    action_msg.data = "start";
    farming_action_pub_->publish(action_msg);
    
    // Note: The actual status will be updated when we receive a new status message from the robot
    // We don't update farming_status_ here as it should be updated via the subscription callback
  }
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

void MacadamiaFarmPanel::onGridSizeChanged()
{
  // Get the grid size value from the text editor
  QString grid_size_text = grid_size_editor_->text();
  bool conversion_ok = false;
  double grid_size_value = grid_size_text.toDouble(&conversion_ok);
  
  // Validate the input
  if (!conversion_ok || grid_size_value <= 0.0) {
    // Invalid input, reset to default
    grid_size_editor_->setText("0.5");
    grid_size_value = 0.5;
  }
  
  // Publish the configuration update
  if (farming_config_pub_) {
    std_msgs::msg::String config_msg;
    config_msg.data = "grid_size=" + std::to_string(grid_size_value);
    farming_config_pub_->publish(config_msg);
  }
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
