#ifndef STATE_MACHINE_CONTROLLER_H
#define STATE_MACHINE_CONTROLLER_H
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_state_machine_controller.h>
#include <QWidget>

// service server for each module
//#include <freespace_ros/FreespaceControl.h>
//#include <orb_slam_2_ros/SlamControl.h>

namespace rqt_state_machine
{

class StateMachineController : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  StateMachineController();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);
protected slots:
  // slam state control functions
  virtual void onSlamStart();
  virtual void onSlamStop();
  virtual void onSlamRecordPathStart();
  virtual void onSlamRecordPathStop();
  virtual void onSlamSaveMapPath();
  virtual void onSlamSwitchToMapping();
  virtual void onSlamSwitchToLocalization();
  virtual void onSlamResetMapping();

  // freespace state control functions
  virtual void onFreespaceStart();
  virtual void onFreespaceStop();

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  Ui::StateMachineControllerWidget ui_;
  QWidget* widget_;

};
} // namespace

#endif // STATE_MACHINE_CONTROLLER_H
