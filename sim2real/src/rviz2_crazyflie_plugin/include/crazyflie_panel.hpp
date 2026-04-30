#ifndef RVIZ2_CRAZYFLIE_PLUGIN__CRAZYFLIE_PANEL_HPP
#define RVIZ2_CRAZYFLIE_PLUGIN__CRAZYFLIE_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <jirl_interfaces/srv/update_setpoint.hpp>
#include <jirl_interfaces/srv/start_trajectory.hpp>

#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>
#include <QDialog>

#include <QWidget>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>

#define LINE std::cout << __PRETTY_FUNCTION__ << " - Line: " << __LINE__ << std::endl;

using namespace jirl_interfaces::srv;

namespace rviz2_crazyflie_plugin
{

class CrazyfliePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  CrazyfliePanel(QWidget* parent = nullptr);

private Q_SLOTS:
  void callLand();
  void callTakeoff();
  void callUpdateSetpoint();
  void handleTrajectorySelection(int index);
  void openCircleConfig();
  void callCircleTrajectory();
  void handleServiceResponse(bool success, const std::string &service_name);
  void restoreLastCircleSettings();

private:
  rclcpp::Node::SharedPtr node_;

  QLineEdit *drone_name_;

  // Main buttons
  QPushButton *land_button_;
  QPushButton *takeoff_button_;
  QPushButton *setpoint_button_;

  // Input Setpoint
  QLineEdit *x_input_;
  QLineEdit *y_input_;
  QLineEdit *z_input_;
  QLineEdit *yaw_input_;
  QCheckBox *global_checkbox_;

  QComboBox *trajectory_selector_;

  QLineEdit *radius_input_;
  QLineEdit *frequency_input_;
  QLineEdit *duration_input_;
  QCheckBox *move_yaw_checkbox_;
  QCheckBox *direction_checkbox_;
  QComboBox *plane_selector_;
  QPushButton *confirm_circle_button_;
  QDialog *circle_config_dialog_;

  QLabel *status_label_;

  float last_radius_;
  float last_frequency_;
  float last_duration_;
  bool last_direction_;
  int last_plane_;
};

} // namespace rviz2_crazyflie_plugin

#endif  // RVIZ2_CRAZYFLIE_PLUGIN__CRAZYFLIE_PANEL_HPP
