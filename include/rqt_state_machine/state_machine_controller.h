#ifndef STATE_MACHINE_CONTROLLER_H
#define STATE_MACHINE_CONTROLLER_H
#include <math.h>

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_state_machine_controller.h>
#include <geometry_msgs/Twist.h>
#include <QWidget>
#include <QTimer>
#include <QFileDialog>
#include <QKeyEvent>

// service server for each module
#include <state_machine_msgs/ActionControl.h>
#include <parkinglot_msgs/ParkingLotDetectionStatusStamped.h>
#include <parkinglot_msgs/ParkingLotDetectionCtrlStamped.h>
#include <state_machine_msgs/StateFeedback.h>

// lcm
#include <lcm/lcm-cpp.hpp>
#include <vel_conversion/ackermann_cmd.hpp>
#include <vel_conversion/ackermann_odom.hpp>
#include <sensor/sonar.hpp>
#include <sensor/imu.hpp>
#include <sensor/egomotion_fusion.hpp>
#include <sensor/esr.hpp>
#include <sensor/esr_track.hpp>
#include <parkinglot/OFILM_CTRL.hpp>

namespace rqt_state_machine
{

static const char* PACKAGE_BRINGUP = "saic_bringup";
static const char* PACKAGE_SLAM = "orb_slam_2_ros";
static const char* PACKAGE_FREESPACE = "freespace_ros";
static const char* PACKAGE_NAVIGATION = "navigation_launch";
static const char* PACKAGE_DEEPPS = "deepps_ros";

static const char* SLAM_NODE_NAME = "orb_slam_2_ros_node";
static const char* FREESPACE_NODE_NAME = "freespace_node";

static const char* LCM_CHANNEL_ACKERMANN_CMD = "ACKERMANN_CMD";
static const char* LCM_CHANNEL_ACKERMANN_ODOM = "ACKERMANN_ODOM";
static const char* LCM_CHANNEL_SENSOR_SONAR = "SENSOR_SONAR";
static const char* LCM_CHANNEL_SENSOR_IMU = "SENSOR_IMU";
static const char* LCM_CHANNEL_SENSOR_EGOMOTION = "SENSOR_EGOMOTION";
static const char* LCM_CHANNEL_SENSOR_ESR_FRONT = "SENSOR_ESR";
static const char* LCM_CHANNEL_PARKINGLOT_CTRL = "OFILM_CTRL";

class StateMachineStatus
{
public:
  enum class Slam
  {
    RUNNING, //!< Module is started and running.
    IDLE,    //!< Module is waiting or stopped.
    ERROR    //!< Some error occured.
  };
  enum class Navigation
  {
    RUNNING, //!< Module is started and running.
    IDLE,    //!< Module is waiting or stopped.
    ERROR    //!< Some error occured.
  };
  enum class VehicleControl
  {
    RUNNING, //!< Module is started and running.
    IDLE,    //!< Module is waiting or stopped.
    ERROR    //!< Some error occured.
  };
  enum class Freespace
  {
    RUNNING, //!< Module is started and running.
    IDLE,    //!< Module is waiting or stopped.
    ERROR    //!< Some error occured.
  };
  enum class Ssd
  {
    RUNNING, //!< Module is started and running.
    IDLE,    //!< Module is waiting or stopped.
    ERROR    //!< Some error occured.
  };
  enum class Deepps
  {
    RUNNING, //!< Module is started and running.
    IDLE,    //!< Module is waiting or stopped.
    ERROR    //!< Some error occured.
  };
  enum class ParkingPlanning
  {
    RUNNING, //!< Module is started and running.
    IDLE,    //!< Module is waiting or stopped.
    ERROR,   //!< Some error occured.
    TRACKING //!< Tracking current parking lot.
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
  bool eventFilter(QObject* target, QEvent* event);

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
  virtual void onSlamRemoveDefaultMapFile();

  // freespace state control functions
  virtual void onFreespaceStart();
  virtual void onFreespaceStop();
  virtual void onFreespaceTestObstacleEnable();
  virtual void onFreespaceTestObstacleDisable();
  virtual void changeTestObstacleState();
  virtual void updateTestObstacle();
  virtual void onFreespaceSurroundEnable();
  virtual void onFreespaceSurroundDisable();
  virtual void onFreespaceFrontEnable();
  virtual void onFreespaceFrontDisable();

  // navigation & following state control functions
  virtual void onNavigationStart();
  virtual void onNavigationCancel();
  virtual void onFollowingStart();
  virtual void onFollowingStop();

  // vehicle control state control functions
  virtual void onVehicleControlEnable();
  virtual void onVehicleControlDisable();
  virtual void onVehicleControlEStop();
  virtual void onVehicleControlSoftStop();
  virtual void setSliderSpeed(int speed);
  virtual void setDialSteering(int steering);
  virtual void stopKeyboardControl();
  virtual void setKeyboardSteeringZero();
  virtual void setKeyboardSpeedZero();
  virtual void keyboardControlPublish();
  virtual void keyboardControlEnable();
  virtual void onSonarEnable();
  virtual void onSonarDisable();
  virtual void onFrontEsrEnable();
  virtual void onFrontEsrDisable();

  virtual void changeLcmMonitorState();
  virtual void changeLcmAckermannCmdState();
  virtual void changeLcmAckermannOdomState();
  virtual void changeLcmSensorSonarState();
  virtual void changeLcmSensorImuState();
  virtual void changeLcmSensorEgomotionState();
  virtual void changeLcmSensorEsrFrontState();
  virtual void changeLcmParkinglotCtrlState();
  virtual void lcmChecking();
  virtual void resetLcmOutput();

  virtual void updateAckermannCmdLcm(const lcm::ReceiveBuffer* rbuf,
                                     const std::string& chan,
                                     const vel_conversion::ackermann_cmd* msg);
  virtual void
  updateAckermannOdomLcm(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const vel_conversion::ackermann_odom* msg);
  virtual void updateSensorSonarLcm(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const sensor::sonar* msg);
  virtual void updateSensorImuLcm(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const sensor::imu* msg);
  virtual void updateSensorEgomotionLcm(const lcm::ReceiveBuffer* rbuf,
                                        const std::string& chan,
                                        const sensor::egomotion_fusion* msg);
  virtual void updateSensorEsrFrontLcm(const lcm::ReceiveBuffer* rbuf,
                                       const std::string& chan,
                                       const sensor::esr* msg);
  virtual void updateParkinglotCtrlLcm(const lcm::ReceiveBuffer* rbuf,
                                       const std::string& chan,
                                       const OFILM_CTRL* msg);

  // deepps state control function
  virtual void onDeeppsStart();
  virtual void onDeeppsStop();
  virtual void getDeeppsStartPos();
  virtual void enableParkinglot();
  virtual void applyParkinglotId();
  virtual void clearParkinglotId();
  virtual void enableVirtualParkinglot();
  virtual void applyVirtualParkinglot();
  virtual void refreshVirtualParkinglot();
  virtual void enableFreespaceParkinglot();

  // state machine
  // -- initialize status of different modules
  virtual void initStateMachineStatus();
  // -- start running
  virtual void startStateMachine();
  // -- stop running
  virtual void stopStateMachine();

  // state checking for modules
  virtual void stateChecking();

  // reset the ui status of each module
  virtual void resetStatusUI();

  // open and get the paths to launch files of modules
  virtual void getLaunchFilePathBringup();
  virtual void getLaunchFilePathSlam();
  virtual void getLaunchFilePathFreespace();
  virtual void getLaunchFilePathNavigation();
  virtual void getLaunchFilePathDeepps();

  // launch modules
  virtual void launchBringup();
  virtual void launchSlam();
  virtual void launchFreespace();
  virtual void launchNavigation();
  virtual void launchDeepps();
  virtual void launchRviz();
  virtual void configLcmCAN();
  virtual void launchCAN();

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  // state machine
  // -- update states
  bool
  updateStateMachineStates(state_machine_msgs::StateFeedback::Request& req,
                           state_machine_msgs::StateFeedback::Response& res);

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

  // state checking functions for modules
  void checkSlamMode();
  void checkSlamStatus();
  void checkSlamPathRecording();
  void checkSlamMapScale();
  void checkDeeppsStartCondition();

  // initialize lcm
  void initLcm();

  Ui::StateMachineControllerWidget ui_;
  QWidget* widget_;

  // ros topics
  ros::NodeHandle nh_;
  ros::Subscriber parkinglot_status_sub_;
  ros::Subscriber parkinglot_ctrl_sub_;
  ros::Publisher keyboard_control_pub_;
  ros::Publisher navigation_goal_pub_;
  ros::Publisher navigation_cancel_pub_;

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

  // modules related variables
  tf::Point deepps_start_pos_;

  // tf
  tf::TransformListener tf_listener_;

  // Timers
  QTimer stateCheckingTimer_;
  QTimer lcmCheckingTimer_;
  QTimer keyboardControlTimer_;

  // lcm
  std::shared_ptr<lcm::LCM> lcm_;
  lcm::Subscription* ackermann_cmd_lcm_sub_;
  lcm::Subscription* ackermann_odom_lcm_sub_;
  lcm::Subscription* sensor_sonar_lcm_sub_;
  lcm::Subscription* sensor_imu_lcm_sub_;
  lcm::Subscription* sensor_egomotion_lcm_sub_;
  lcm::Subscription* sensor_esr_front_lcm_sub_;
  lcm::Subscription* parkinglot_ctrl_lcm_sub_;
  bool lcmMonitorEnabled_;
};
} // namespace

#endif // STATE_MACHINE_CONTROLLER_H
