#ifndef STATE_MACHINE_CONTROLLER_H
#define STATE_MACHINE_CONTROLLER_H
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_state_machine_controller.h>
#include <QWidget>

// service server for each module
//#include <freespace_ros/FreespaceControl.h>
#include <state_machine_msgs/ActionControl.h>
#include <parkinglot_msgs/ParkingLotDetectionStatusStamped.h>
#include <parkinglot_msgs/ParkingLotDetectionCtrlStamped.h>
#include <rqt_state_machine/StateFeedback.h>

namespace rqt_state_machine
{

class StateMachineStatus
{
public:
  enum class Slam
  {
    RUNNING,      //!< Module is started and running.
    IDLE,         //!< Module is waiting or stopped.
    ERROR         //!< Some error occured.
  };
  enum class Navigation
  {
    RUNNING,      //!< Module is started and running.
    IDLE,         //!< Module is waiting or stopped.
    ERROR         //!< Some error occured.
  };
  enum class VehicleControl
  {
    RUNNING,      //!< Module is started and running.
    IDLE,         //!< Module is waiting or stopped.
    ERROR         //!< Some error occured.
  };
  enum class Freespace
  {
    RUNNING,      //!< Module is started and running.
    IDLE,         //!< Module is waiting or stopped.
    ERROR         //!< Some error occured.
  };
  enum class Ssd
  {
    RUNNING,      //!< Module is started and running.
    IDLE,         //!< Module is waiting or stopped.
    ERROR         //!< Some error occured.
  };
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
  virtual void onSlamRecordDeeppsStartPos();

  // freespace state control functions
  virtual void onFreespaceStart();
  virtual void onFreespaceStop();

  // navigation state control functions
  virtual void onNavigationStart();
  virtual void onNavigationStop();

  // vehicle control state control functions
  virtual void onVehicleControlEnable();
  virtual void onVehicleControlDisable();
  virtual void onVehicleControlEStop();

  // deepps state control function
  virtual void onDeeppsStart();
  virtual void onDeeppsStop();

  // state machine
  // -- initialize status of different modules
  virtual void initStateMachineStatus();
  // -- start running
  virtual void startStateMachine();
  // -- stop running
  virtual void stopStateMachine();

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  // state machine
  // -- update states
  bool updateStateMachineStates(StateFeedback::Request &req, StateFeedback::Response &res);

  void parkinglotStatusCB(
      const parkinglot_msgs::ParkingLotDetectionStatusStamped::ConstPtr msg);
  void parkinglotCtrlCB(
      const parkinglot_msgs::ParkingLotDetectionCtrlStamped::ConstPtr msg);

  // update module status in UI
  void updateSlamStatusUI();
  void updateNaviStatusUI();
  void updateCtrlStatusUI();
  void updateFreespaceStatusUI();
  void updateSsdStatusUI();
  void updateDeeppsStatusUI();
  void updateParkingStatusUI();

  Ui::StateMachineControllerWidget ui_;
  QWidget* widget_;

  // ros topics
  ros::NodeHandle nh_;
  ros::Subscriber parkinglot_status_sub_;
  ros::Subscriber parkinglot_ctrl_sub_;

  // ros service
  ros::ServiceServer state_feedback_service_;

  // state machine
  // -- module status
  StateMachineStatus::Slam slam_status_;
  StateMachineStatus::Navigation navi_status_;
  StateMachineStatus::VehicleControl vehicle_ctrl_status_;
  StateMachineStatus::Freespace freespace_status_;
  StateMachineStatus::Ssd ssd_status_;
  StateMachineStatus::Deepps deepps_status_;
  StateMachineStatus::ParkingPlanning parking_status_;
};
} // namespace

#endif // STATE_MACHINE_CONTROLLER_H
