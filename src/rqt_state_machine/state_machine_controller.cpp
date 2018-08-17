#include "rqt_state_machine/state_machine_controller.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMessageBox>

namespace rqt_state_machine {

StateMachineController::StateMachineController()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("StateMachineController");
}

void StateMachineController::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  connect(ui_.startSlam, SIGNAL(clicked()), this, SLOT(onSlamStart()));
  connect(ui_.stopSlam, SIGNAL(clicked()), this, SLOT(onSlamStop()));
  connect(ui_.startRecordPath, SIGNAL(clicked()), this, SLOT(onSlamRecordPathStart()));
  connect(ui_.stopRecordPath, SIGNAL(clicked()), this, SLOT(onSlamRecordPathStop()));
  connect(ui_.saveMapPath, SIGNAL(clicked()), this, SLOT(onSlamSaveMapPath()));
  connect(ui_.switchMapping, SIGNAL(clicked()), this, SLOT(onSlamSwitchToMapping()));
  connect(ui_.switchLocalization, SIGNAL(clicked()), this, SLOT(onSlamSwitchToLocalization()));
  connect(ui_.resetMapping, SIGNAL(clicked()), this, SLOT(onSlamResetMapping()));

  connect(ui_.startFreespace, SIGNAL(clicked()), this, SLOT(onFreespaceStart()));
  connect(ui_.stopFreespace, SIGNAL(clicked()), this, SLOT(onFreespaceStop()));
}

void StateMachineController::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void StateMachineController::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void StateMachineController::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

// slam state control functions
void StateMachineController::onSlamStart()
{
  orb_slam_2_ros::SlamControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 1;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "start", "Failed to start slam!");
  }
  else
    QMessageBox::warning(widget_, "start", "Failed to call start freespace service!");

  return;
}

void StateMachineController::onSlamStop()
{
  orb_slam_2_ros::SlamControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 0;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "stop", "Failed to stop slam!");
  }
  else
    QMessageBox::warning(widget_, "stop", "Failed to call stop freespace service!");

  return;
}

void StateMachineController::onSlamRecordPathStart()
{
  orb_slam_2_ros::SlamControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 3;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "record path", "Failed to start recording path!");
  }
  else
    QMessageBox::warning(widget_, "record path", "Failed to call start recording path service!");

  return;
}

void StateMachineController::onSlamRecordPathStop()
{
  orb_slam_2_ros::SlamControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 4;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "record path", "Failed to stop recording path!");
  }
  else
    QMessageBox::warning(widget_, "record path", "Failed to call stop recording path service!");

  return;
}

void StateMachineController::onSlamSaveMapPath()
{
  orb_slam_2_ros::SlamControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 2;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "save map and path", "Failed to save map and path!");
  }
  else
    QMessageBox::warning(widget_, "save map and path", "Failed to call save map and path service!");

  return;
}

void StateMachineController::onSlamSwitchToMapping()
{
  orb_slam_2_ros::SlamControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 5;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "switch to mapping", "Failed to switch to mapping!");
  }
  else
    QMessageBox::warning(widget_, "switch to mapping", "Failed to call switch to mapping service!");

  return;
}

void StateMachineController::onSlamSwitchToLocalization()
{
  orb_slam_2_ros::SlamControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 6;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "switch to localization", "Failed to switch to localization!");
  }
  else
    QMessageBox::warning(widget_, "switch to localization", "Failed to call switch to localization service!");

  return;
}

void StateMachineController::onSlamResetMapping()
{
  orb_slam_2_ros::SlamControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 7;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "reset mapping", "Failed to reset mapping!");
  }
  else
    QMessageBox::warning(widget_, "reset mapping", "Failed to call reset mapping service!");

  return;
}

// freespace state control functions
void StateMachineController::onFreespaceStart()
{
  freespace_ros::FreespaceControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 1;

  if (ros::service::call("freespace_state_control", srv))
  {
    QMessageBox::information(widget_, "start", "Succeed to start freespace!");
  }
  else
  {
    QMessageBox::warning(widget_, "start", "Failed to start freespace!");
    return;
  }
}

void StateMachineController::onFreespaceStop()
{
  freespace_ros::FreespaceControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 0;

  if (ros::service::call("freespace_state_control", srv))
  {
    QMessageBox::information(widget_, "stop", "Succeed to stop freespace!");
  }
  else
  {
    QMessageBox::warning(widget_, "stop", "Failed to stop freespace!");
    return;
  }
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_state_machine, StateMachineController, rqt_state_machine::StateMachineController, rqt_gui_cpp::Plugin)
