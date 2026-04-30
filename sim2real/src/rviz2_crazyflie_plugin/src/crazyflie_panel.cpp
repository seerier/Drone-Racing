#include <crazyflie_panel.hpp>

namespace rviz2_crazyflie_plugin
{

CrazyfliePanel::CrazyfliePanel(QWidget* parent)
: rviz_common::Panel(parent),
  last_radius_(0.0), last_frequency_(0.0), last_duration_(0.0), last_direction_(false), last_plane_(0)
{
  // Create ROS2 node
  node_ = std::make_shared<rclcpp::Node>("crazyflie_panel");

  drone_name_ = new QLineEdit("crazy_jirl_01");
  QHBoxLayout *drone_name_layout = new QHBoxLayout();
  drone_name_layout->addWidget(new QLabel("Drone Name:"));
  drone_name_layout->addWidget(drone_name_);

  land_button_ = new QPushButton("Land");
  takeoff_button_ = new QPushButton("Takeoff");
  setpoint_button_ = new QPushButton("Set Setpoint");

  connect(land_button_, &QPushButton::clicked, this, &CrazyfliePanel::callLand);
  connect(takeoff_button_, &QPushButton::clicked, this, &CrazyfliePanel::callTakeoff);
  connect(setpoint_button_, &QPushButton::clicked, this, &CrazyfliePanel::callUpdateSetpoint);

  // Input Setpoint
  x_input_ = new QLineEdit();
  y_input_ = new QLineEdit();
  z_input_ = new QLineEdit();
  yaw_input_ = new QLineEdit();
  global_checkbox_ = new QCheckBox("Global");
  global_checkbox_->setChecked(false);

  QHBoxLayout *setpoint_layout = new QHBoxLayout();
  setpoint_layout->addWidget(new QLabel("X:"));
  setpoint_layout->addWidget(x_input_);
  setpoint_layout->addWidget(new QLabel("Y:"));
  setpoint_layout->addWidget(y_input_);
  setpoint_layout->addWidget(new QLabel("Z:"));
  setpoint_layout->addWidget(z_input_);
  setpoint_layout->addWidget(new QLabel("Yaw:"));
  setpoint_layout->addWidget(yaw_input_);
  setpoint_layout->addWidget(global_checkbox_);

  // Trajectory selector
  trajectory_selector_ = new QComboBox();
  trajectory_selector_->addItem("Select Trajectory");
  trajectory_selector_->addItem("Circle");

  connect(trajectory_selector_, QOverload<int>::of(&QComboBox::activated), this, &CrazyfliePanel::handleTrajectorySelection);

  // Status label
  status_label_ = new QLabel("Status: Ready");
  status_label_->setStyleSheet("QLabel { color: green ; }");

  // Main layout
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addLayout(drone_name_layout);
  main_layout->addWidget(takeoff_button_);
  main_layout->addWidget(land_button_);
  main_layout->addSpacing(10);
  main_layout->addLayout(setpoint_layout);
  main_layout->addWidget(setpoint_button_);
  main_layout->addSpacing(10);
  main_layout->addWidget(trajectory_selector_);
  main_layout->addSpacing(10);
  main_layout->addWidget(status_label_);

  setLayout(main_layout);
}

void CrazyfliePanel::callLand() {
  std::string ns = "/" + drone_name_->text().toStdString();
  auto client = node_->create_client<std_srvs::srv::Trigger>(ns + "/land");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  bool success = false;

  if (client->wait_for_service(std::chrono::seconds(2))) {
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      success = response->success;
    }
  }
  handleServiceResponse(success, "Land");
}

void CrazyfliePanel::callTakeoff() {
  std::string ns = "/" + drone_name_->text().toStdString();
  auto client = node_->create_client<std_srvs::srv::Trigger>(ns + "/takeoff");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  bool success = false;

  if (client->wait_for_service(std::chrono::seconds(2))) {
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      success = response->success;
    }
  }
  handleServiceResponse(success, "Takeoff");
}

void CrazyfliePanel::callUpdateSetpoint() {
  std::string ns = "/" + drone_name_->text().toStdString();
  auto client = node_->create_client<UpdateSetpoint>(ns + "/update_setpoint");
  auto request = std::make_shared<UpdateSetpoint::Request>();
  bool success = false;

  request->x = x_input_->text().toDouble();
  request->y = y_input_->text().toDouble();
  request->z = z_input_->text().toDouble();
  request->yaw = yaw_input_->text().toDouble();
  request->is_global = global_checkbox_->isChecked();

  if (client->wait_for_service(std::chrono::seconds(2))) {
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      success = response->success;
    }
  }
  handleServiceResponse(success, "UpdateSetpoint");
}

void CrazyfliePanel::handleTrajectorySelection(int index) {
  if (trajectory_selector_->itemText(index) == "Circle") {
    openCircleConfig();
  }
}

void CrazyfliePanel::openCircleConfig() {
  circle_config_dialog_ = new QDialog(this);
  circle_config_dialog_->setWindowTitle("Circle Trajectory Configuration");

  radius_input_ = new QLineEdit();
  frequency_input_ = new QLineEdit();
  duration_input_ = new QLineEdit();
  move_yaw_checkbox_ = new QCheckBox("Move Yaw");
  direction_checkbox_ = new QCheckBox("Clockwise (CW)");
  plane_selector_ = new QComboBox();
  plane_selector_->addItem("XY");
  plane_selector_->addItem("XZ");
  plane_selector_->addItem("YZ");

  // Restore last circle settings
  radius_input_->setText(QString::number(last_radius_));
  frequency_input_->setText(QString::number(last_frequency_));
  duration_input_->setText(QString::number(last_duration_));
  direction_checkbox_->setChecked(last_direction_);
  plane_selector_->setCurrentIndex(last_plane_);

  confirm_circle_button_ = new QPushButton("Start Circle");
  connect(confirm_circle_button_, &QPushButton::clicked, this, &CrazyfliePanel::callCircleTrajectory);

  QFormLayout *form_layout = new QFormLayout();
  form_layout->addRow("Radius:", radius_input_);
  form_layout->addRow("Frequency:", frequency_input_);
  form_layout->addRow("Duration:", duration_input_);
  form_layout->addRow(move_yaw_checkbox_);
  form_layout->addRow(direction_checkbox_);
  form_layout->addRow("Plane:", plane_selector_);
  form_layout->addRow(confirm_circle_button_);

  QVBoxLayout *dialog_layout = new QVBoxLayout();
  dialog_layout->addLayout(form_layout);
  circle_config_dialog_->setLayout(dialog_layout);
  circle_config_dialog_->exec();
}

void CrazyfliePanel::callCircleTrajectory() {
  std::string ns = "/" + drone_name_->text().toStdString();
  auto client = node_->create_client<StartTrajectory>(ns + "/trajectory");
  auto request = std::make_shared<StartTrajectory::Request>();
  bool success = false;

  request->trajectory_type = StartTrajectory::Request::CIRCLE;
  request->radius = radius_input_->text().toDouble();
  request->freq = frequency_input_->text().toDouble();
  request->duration = duration_input_->text().toDouble();
  request->direction = direction_checkbox_->isChecked() ? StartTrajectory::Request::DIR_CCW
                                                        : StartTrajectory::Request::DIR_CW;
  request->plane = static_cast<int8_t>(plane_selector_->currentIndex());

  if (client->wait_for_service(std::chrono::seconds(2))) {
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      success = response->success;
    }
  }
  handleServiceResponse(success, "Circle Trajectory");

  // Read circle settings
  last_radius_ = radius_input_->text().toDouble();
  last_frequency_ = frequency_input_->text().toDouble();
  last_duration_ = duration_input_->text().toDouble();
  last_direction_ = direction_checkbox_->isChecked();
  last_plane_ = plane_selector_->currentIndex();

  circle_config_dialog_->close();
}

void CrazyfliePanel::handleServiceResponse(bool success, const std::string &service_name) {
  if (success) {
    status_label_->setText(QString::fromStdString("Status: " + service_name + " OK"));
    status_label_->setStyleSheet("QLabel { color: green; }");
  } else {
    status_label_->setText(QString::fromStdString("Status: " + service_name + " FAILED"));
    status_label_->setStyleSheet("QLabel { color: red; }");
  }
}

}  // namespace rviz2_crazyflie_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_crazyflie_plugin::CrazyfliePanel, rviz_common::Panel)
