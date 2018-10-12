#ifndef STATE_MACHINE_CONTROLLER_H
#define STATE_MACHINE_CONTROLLER_H
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_state_machine_controller.h>
#include <QWidget>

// service server for each module
#include <freespace_ros/FreespaceControl.h>
#include <orb_slam_2_ros/SlamControl.h>
#include <vehicle_control/VehicleControl.h>
#include <parkinglot_msgs/ParkingLotDetectionStatusStamped.h>
#include <parkinglot_msgs/ParkingLotDetectionCtrlStamped.h>

namespace rqt_state_machine
{

class StateMachineStatus
{
public:
  enum class Deepps
  {
    RUNNING,      //!< Module is started and running.
    IDLE,         //!< Module is waiting or stopped.
    ERROR         //!< Some error occured.
  };
  enum class ParkingPlanning
  {
    RUNNING,      //!< Module is started and running.
    IDLE,         //!< Module is waiting or stopped.
    ERROR,        //!< Some error occured.
    TRACKING      //!< Tracking current parking lot.
  };
};

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
  virtual void onSlamRecordPathInLocalizationStart();
  virtual void onSlamRecordPathInLocalizationStop();

  // freespace state control functions
  virtual void onFreespaceStart();
  virtual void onFreespaceStop();

  // vehicle control state control functions
  virtual void onVehicleControlEnable();
  virtual void onVehicleControlDisable();

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  // initialize status of different modules
  void initStateMachineStatus();

  void parkinglotStatusCB(
      const parkinglot_msgs::ParkingLotDetectionStatusStamped::ConstPtr msg);
  void parkinglotCtrlCB(
      const parkinglot_msgs::ParkingLotDetectionCtrlStamped::ConstPtr msg);

  Ui::StateMachineControllerWidget ui_;
  QWidget* widget_;

  // ros topics
  ros::NodeHandle nh_;
  ros::Subscriber parkinglot_status_sub_;
  ros::Subscriber parkinglot_ctrl_sub_;

  // state machine
  // -- parking planning
  StateMachineStatus::Deepps deepps_status_;
  StateMachineStatus::ParkingPlanning parking_status_;

};
} // namespace

#endif // STATE_MACHINE_CONTROLLER_H
