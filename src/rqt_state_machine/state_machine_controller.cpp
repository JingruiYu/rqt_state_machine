#include "rqt_state_machine/state_machine_controller.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMessageBox>

namespace rqt_state_machine
{

StateMachineController::StateMachineController()
    : rqt_gui_cpp::Plugin(), widget_(0)
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
  ui_.tabWidget->setCurrentWidget(ui_.tabLaunch);

  ui_.verticalLayoutSpeed->setAlignment(ui_.sliderSpeed, Qt::AlignCenter);

  ui_.radioModeSlam->setEnabled(false);
  ui_.radioModeLocalization->setEnabled(false);
  ui_.radioSlamRunning->setEnabled(false);
  ui_.radioSlamIdle->setEnabled(false);
  ui_.radioTrackingSuccessful->setEnabled(false);
  ui_.radioTrackingFailed->setEnabled(false);
  ui_.radioPathRecording->setEnabled(false);
  ui_.radioPathIdle->setEnabled(false);

  // add widget to the user interface
  context.addWidget(widget_);

  widget_->installEventFilter(this);
  widget_->setFocus();

  connect(ui_.startSlam, SIGNAL(clicked()), this, SLOT(onSlamStart()));
  connect(ui_.stopSlam, SIGNAL(clicked()), this, SLOT(onSlamStop()));
  connect(ui_.startRecordPath, SIGNAL(clicked()), this,
          SLOT(onSlamRecordPathStart()));
  connect(ui_.stopRecordPath, SIGNAL(clicked()), this,
          SLOT(onSlamRecordPathStop()));
  connect(ui_.saveMapPath, SIGNAL(clicked()), this, SLOT(onSlamSaveMapPath()));
  connect(ui_.switchMapping, SIGNAL(clicked()), this,
          SLOT(onSlamSwitchToMapping()));
  connect(ui_.switchLocalization, SIGNAL(clicked()), this,
          SLOT(onSlamSwitchToLocalization()));
  connect(ui_.resetMapping, SIGNAL(clicked()), this,
          SLOT(onSlamResetMapping()));
  connect(ui_.recordDeeppsStartPos, SIGNAL(clicked()), this,
          SLOT(onSlamRecordDeeppsStartPos()));
  connect(ui_.recordDeeppsVirtualParkinglot, SIGNAL(clicked()), this,
          SLOT(onSlamRecordDeeppsVirtualParkinglot()));
  connect(ui_.removeDefaultMapFile, SIGNAL(clicked()), this,
          SLOT(onSlamRemoveDefaultMapFile()));

  connect(ui_.startFreespace, SIGNAL(clicked()), this,
          SLOT(onFreespaceStart()));
  connect(ui_.stopFreespace, SIGNAL(clicked()), this, SLOT(onFreespaceStop()));
  connect(ui_.enableTestObstacle, SIGNAL(stateChanged(int)), this,
          SLOT(changeTestObstacleState()));
  connect(ui_.updateObstacle, SIGNAL(clicked()), this,
          SLOT(updateTestObstacle()));
  connect(ui_.enableSurround, SIGNAL(clicked()), this,
          SLOT(onFreespaceSurroundEnable()));
  connect(ui_.disableSurround, SIGNAL(clicked()), this,
          SLOT(onFreespaceSurroundDisable()));
  connect(ui_.enableFront, SIGNAL(clicked()), this,
          SLOT(onFreespaceFrontEnable()));
  connect(ui_.disableFront, SIGNAL(clicked()), this,
          SLOT(onFreespaceFrontDisable()));

  connect(ui_.enableVehicleControlManually, SIGNAL(clicked()), this,
          SLOT(onVehicleControlEnable()));
  connect(ui_.disableVehicleControlManually, SIGNAL(clicked()), this,
          SLOT(onVehicleControlDisable()));
  connect(ui_.eStopVehicleControlManually, SIGNAL(clicked()), this,
          SLOT(onVehicleControlEStop()));
  connect(ui_.softStopVehicleControlManually, SIGNAL(clicked()), this,
          SLOT(onVehicleControlSoftStop()));
  connect(ui_.sliderSpeed, SIGNAL(valueChanged(int)), this,
          SLOT(setSliderSpeed(int)));
  connect(ui_.dialSteering, SIGNAL(valueChanged(int)), this,
          SLOT(setDialSteering(int)));
  connect(ui_.stopKeyboardControl, SIGNAL(clicked()), this,
          SLOT(stopKeyboardControl()));
  connect(ui_.setKeyboardSpeedZero, SIGNAL(clicked()), this,
          SLOT(setKeyboardSpeedZero()));
  connect(ui_.setKeyboardSteeringZero, SIGNAL(clicked()), this,
          SLOT(setKeyboardSteeringZero()));
  connect(ui_.enableKeyboardControl, SIGNAL(stateChanged(int)), this,
          SLOT(keyboardControlEnable()));
  connect(ui_.enableSonar, SIGNAL(clicked()), this, SLOT(onSonarEnable()));
  connect(ui_.disableSonar, SIGNAL(clicked()), this, SLOT(onSonarDisable()));
  connect(ui_.enableFrontEsr, SIGNAL(clicked()), this,
          SLOT(onFrontEsrEnable()));
  connect(ui_.disableFrontEsr, SIGNAL(clicked()), this,
          SLOT(onFrontEsrDisable()));

  connect(ui_.startNavigation, SIGNAL(clicked()), this,
          SLOT(onNavigationStart()));
  connect(ui_.cancelNavigation, SIGNAL(clicked()), this,
          SLOT(onNavigationCancel()));
  connect(ui_.startFollowing, SIGNAL(clicked()), this,
          SLOT(onFollowingStart()));

  connect(ui_.startDeeppsManually, SIGNAL(clicked()), this,
          SLOT(onDeeppsStart()));
  connect(ui_.stopDeeppsManually, SIGNAL(clicked()), this,
          SLOT(onDeeppsStop()));
  connect(ui_.enableParkinglot, SIGNAL(stateChanged(int)), this,
          SLOT(enableParkinglot()));
  connect(ui_.applyParkinglotId, SIGNAL(clicked()), this,
          SLOT(applyParkinglotId()));
  connect(ui_.clearParkinglotId, SIGNAL(clicked()), this,
          SLOT(clearParkinglotId()));
  connect(ui_.enableVirtualParkinglot, SIGNAL(stateChanged(int)), this,
          SLOT(enableVirtualParkinglot()));
  connect(ui_.applyVirtualParkinglot, SIGNAL(clicked()), this,
          SLOT(applyVirtualParkinglot()));
  connect(ui_.refreshVirtualParkinglot, SIGNAL(clicked()), this,
          SLOT(refreshVirtualParkinglot()));
  connect(ui_.enableFreespaceParkinglot, SIGNAL(stateChanged(int)), this,
          SLOT(enableFreespaceParkinglot()));
  connect(ui_.refreshDeeppsStartPos, SIGNAL(clicked()), this,
          SLOT(getDeeppsStartPos()));

  connect(ui_.startParkingManually, SIGNAL(clicked()), this,
          SLOT(onParkingStart()));
  connect(ui_.stopParkingManually, SIGNAL(clicked()), this,
          SLOT(onParkingStop()));

  connect(ui_.startDeepps, SIGNAL(clicked()), this, SLOT(onDeeppsStart()));

  connect(ui_.startRecordPathInLocalization, SIGNAL(clicked()), this,
          SLOT(onSlamRecordPathInLocalizationStart()));
  connect(ui_.stopRecordPathInLocalization, SIGNAL(clicked()), this,
          SLOT(onSlamRecordPathInLocalizationStop()));

  connect(ui_.startAuto, SIGNAL(clicked()), this, SLOT(startStateMachine()));
  connect(ui_.stopAuto, SIGNAL(clicked()), this, SLOT(stopStateMachine()));
  connect(ui_.statusReset, SIGNAL(clicked()), this, SLOT(resetStatusUI()));

  connect(ui_.launchFileBringup, SIGNAL(clicked()), this,
          SLOT(getLaunchFilePathBringup()));
  connect(ui_.launchFileSlam, SIGNAL(clicked()), this,
          SLOT(getLaunchFilePathSlam()));
  connect(ui_.launchFileFreespace, SIGNAL(clicked()), this,
          SLOT(getLaunchFilePathFreespace()));
  connect(ui_.launchFileNavigation, SIGNAL(clicked()), this,
          SLOT(getLaunchFilePathNavigation()));
  connect(ui_.launchFileDeepps, SIGNAL(clicked()), this,
          SLOT(getLaunchFilePathDeepps()));

  connect(ui_.launchBringup, SIGNAL(clicked()), this, SLOT(launchBringup()));
  connect(ui_.launchSlam, SIGNAL(clicked()), this, SLOT(launchSlam()));
  connect(ui_.launchFreespace, SIGNAL(clicked()), this,
          SLOT(launchFreespace()));
  connect(ui_.launchNavigation, SIGNAL(clicked()), this,
          SLOT(launchNavigation()));
  connect(ui_.launchDeepps, SIGNAL(clicked()), this, SLOT(launchDeepps()));

  connect(ui_.launchRviz, SIGNAL(clicked()), this, SLOT(launchRviz()));
  connect(ui_.configLcmCAN, SIGNAL(clicked()), this, SLOT(configLcmCAN()));
  connect(ui_.launchCAN, SIGNAL(clicked()), this, SLOT(launchCAN()));

  connect(ui_.enableLcmMonitor, SIGNAL(stateChanged(int)), this,
          SLOT(changeLcmMonitorState()));
  connect(ui_.enableLcmAckermannCmd, SIGNAL(stateChanged(int)), this,
          SLOT(changeLcmAckermannCmdState()));
  connect(ui_.enableLcmAckermannOdom, SIGNAL(stateChanged(int)), this,
          SLOT(changeLcmAckermannOdomState()));
  connect(ui_.enableLcmSensorSonar, SIGNAL(stateChanged(int)), this,
          SLOT(changeLcmSensorSonarState()));
  connect(ui_.enableLcmSensorImu, SIGNAL(stateChanged(int)), this,
          SLOT(changeLcmSensorImuState()));
  connect(ui_.enableLcmSensorEgomotion, SIGNAL(stateChanged(int)), this,
          SLOT(changeLcmSensorEgomotionState()));
  connect(ui_.enableLcmSensorEsrFront, SIGNAL(stateChanged(int)), this,
          SLOT(changeLcmSensorEsrFrontState()));
  connect(ui_.enableLcmParkinglotCtrl, SIGNAL(stateChanged(int)), this,
          SLOT(changeLcmParkinglotCtrlState()));
  connect(ui_.resetLcmOutput, SIGNAL(clicked()), this, SLOT(resetLcmOutput()));

  // start periodic state checking
  connect(&stateCheckingTimer_, SIGNAL(timeout()), this, SLOT(stateChecking()));
  stateCheckingTimer_.setInterval(500);
  stateCheckingTimer_.start();

  // set lcm checking
  connect(&lcmCheckingTimer_, SIGNAL(timeout()), this, SLOT(lcmChecking()));
  lcmCheckingTimer_.setInterval(20);

  connect(&keyboardControlTimer_, SIGNAL(timeout()), this,
          SLOT(keyboardControlPublish()));
  keyboardControlTimer_.setInterval(50);

  // subscribe to ros topics
  parkinglot_status_sub_ =
      nh_.subscribe("/deepps/parkinglot_status", 100,
                    &StateMachineController::parkinglotStatusCB, this);
  parkinglot_ctrl_sub_ =
      nh_.subscribe("/deepps/parkinglot_ctrl", 100,
                    &StateMachineController::parkinglotCtrlCB, this);
  virtual_parkinglot_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>(
      "/deepps/virtual_parkinglot", 1, true);

  deepps_start_pos_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "deepps/start_position", 1);

  keyboard_control_pub_ =
      nh_.advertise<geometry_msgs::Twist>("vehicle_cmd_vel", 30);

  navigation_goal_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  navigation_cancel_pub_ =
      nh_.advertise<std_msgs::Empty>("navigation_cancel", 10);

  // Advertising state control service
  state_feedback_service_ = nh_.advertiseService(
      "state_machine_feedback",
      &StateMachineController::updateStateMachineStates, this);

  // initialize status of different modules
  initStateMachineStatus();

  // initialize lcm
  initLcm();
}

void StateMachineController::shutdownPlugin()
{
  // TODO unregister all publishers here

  stateCheckingTimer_.stop();
  disconnect(&stateCheckingTimer_, SIGNAL(timeout()), this,
             SLOT(stateChecking()));
}

void StateMachineController::saveSettings(
    qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void StateMachineController::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

// initialize status of different modules
void StateMachineController::initStateMachineStatus()
{
  // init state machine status of every module
  // -- slam
  slam_status_ = StateMachineStatus::Slam::IDLE;
  ui_.statusSlam->setText("Idle");
  // -- navigation
  navi_status_ = StateMachineStatus::Navigation::IDLE;
  ui_.statusNavi->setText("Idle");
  // -- vehicle control
  vehicle_ctrl_status_ = StateMachineStatus::VehicleControl::IDLE;
  ui_.statusCtrl->setText("Idle");
  // -- freespace
  freespace_status_ = StateMachineStatus::Freespace::IDLE;
  ui_.statusFreespace->setText("Idle");
  // -- ssd
  ssd_status_ = StateMachineStatus::Ssd::IDLE;
  ui_.statusSsd->setText("Idle");
  // -- deepps
  deepps_status_ = StateMachineStatus::Deepps::IDLE;
  ui_.statusDeepps->setText("Idle");
  // -- parking
  parking_status_ = StateMachineStatus::ParkingPlanning::IDLE;
  ui_.statusParking->setText("Idle");
}

// initialize lcm
void StateMachineController::initLcm()
{
  lcm_ = std::shared_ptr<lcm::LCM>(new lcm::LCM());

  if (!lcm_->good())
  {
    QMessageBox::warning(widget_, "LCM", "LCM is not ok!");
  }

  lcmMonitorEnabled_ = false;
}

// start running state machine
void StateMachineController::startStateMachine()
{
  ui_.status->setText("Status: State machine started!");

  // start slam
  onSlamStart();

  // get deepps start pos
  getDeeppsStartPos();

  return;
}

// stop running state machine
void StateMachineController::stopStateMachine()
{
  // stop following
  onFollowingStop();

  // stop slam
  onSlamStop();

  // stop deepps
  onDeeppsStop();

  // stop freespace
  onFreespaceStop();

  ui_.status->setText("Status: State machine stopped!");
}

// update state machine states
bool StateMachineController::updateStateMachineStates(
    state_machine_msgs::StateFeedback::Request& req,
    state_machine_msgs::StateFeedback::Response& res)
{
  switch (req.state.module)
  {
  case 0: // slam
  {
    switch (req.state.state)
    {
    case 0: // tracking status
    {
      if (req.state.data > 0)
        ui_.statusSlam->setText("Tracking succeed");
      else
        ui_.statusSlam->setText("Tracking failed");
      break;
    }
    case 1: // map scale
    {
      if (req.state.data > 0)
        ui_.mapScale->setText(QString::number(req.state.data, 'f', 2));
      else
        ui_.mapScale->setText("N/A");
      break;
    }
    default:
      break;
    }
    break;
  }
  case 1: // freespace
  {
    ui_.status->setText("Status: freespace feedback received");
    break;
  }
  case 2: // deepps
  {
    ui_.status->setText("Status: deepps feedback received");
    break;
  }
  case 3: // ssd
  {
    ui_.status->setText("Status: ssd feedback received");
    break;
  }
  case 4: // vehicle control
  {
    ui_.status->setText("Status: vehicle control feedback received");
    break;
  }
  default:
    break;
  }

  res.feedback = 1;

  return true;
}

// slam state control functions
void StateMachineController::onSlamStart()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 1;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "start", "Failed to start slam!");
    else
      ui_.status->setText("Status: Slam started!");
  }
  else
    QMessageBox::warning(widget_, "start",
                         "Failed to call start slam service!");

  return;
}

void StateMachineController::onSlamStop()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 0;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "stop", "Failed to stop slam!");
    else
      ui_.status->setText("Status: Slam stopped!");
  }
  else
    QMessageBox::warning(widget_, "stop", "Failed to call stop slam service!");

  return;
}

void StateMachineController::onSlamRecordPathStart()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 3;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "record path",
                           "Failed to start recording path!");
    else
      ui_.status->setText("Status: Start to record path!");
  }
  else
    QMessageBox::warning(widget_, "record path",
                         "Failed to call start recording path service!");

  return;
}

void StateMachineController::onSlamRecordPathStop()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 4;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "record path",
                           "Failed to stop recording path!");
    else
      ui_.status->setText("Status: Stop recording path!");
  }
  else
    QMessageBox::warning(widget_, "record path",
                         "Failed to call stop recording path service!");

  return;
}

void StateMachineController::onSlamSaveMapPath()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 2;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "save map and path",
                           "Failed to save map and path!");
    else
      ui_.status->setText("Status: Map and path saved!");
  }
  else
    QMessageBox::warning(widget_, "save map and path",
                         "Failed to call save map and path service!");

  return;
}

void StateMachineController::onSlamSwitchToMapping()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 5;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "switch to mapping",
                           "Failed to switch to mapping!");
    else
      ui_.status->setText("Status: Switch to mapping!");
  }
  else
    QMessageBox::warning(widget_, "switch to mapping",
                         "Failed to call switch to mapping service!");

  return;
}

void StateMachineController::onSlamSwitchToLocalization()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 6;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "switch to localization",
                           "Failed to switch to localization!");
    else
      ui_.status->setText("Status: Switch to localization!");
  }
  else
    QMessageBox::warning(widget_, "switch to localization",
                         "Failed to call switch to localization service!");

  return;
}

void StateMachineController::onSlamResetMapping()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 7;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "reset mapping",
                           "Failed to reset mapping!");
    else
      ui_.status->setText("Status: Reset mapping!");
  }
  else
    QMessageBox::warning(widget_, "reset mapping",
                         "Failed to call reset mapping service!");

  return;
}

void StateMachineController::onSlamRecordPathInLocalizationStart()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 8;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "record path in localization",
                           "Failed to record path in localization!");
    else
      ui_.status->setText("Status: Record path in localization!");
  }
  else
    QMessageBox::warning(widget_, "record path in localization",
                         "Failed to call record path in localization service!");

  return;
}

void StateMachineController::onSlamRecordPathInLocalizationStop()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 9;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "record path in localization",
                           "Failed to stop recording path in localization!");
    else
      ui_.status->setText("Status: Stop recording path in localization!");
  }
  else
    QMessageBox::warning(
        widget_, "record path in localization",
        "Failed to call stop recording path in localization service!");

  return;
}

void StateMachineController::onSlamRecordDeeppsStartPos()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 10;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "record deepps start position",
                           "Failed to record deepps start position!");
    else
      ui_.status->setText("Status: Record deepps start position!");
  }
  else
    QMessageBox::warning(
        widget_, "record deepps start position",
        "Failed to call record deepps start position service!");

  return;
}

void StateMachineController::onSlamRecordDeeppsVirtualParkinglot()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 15;

  if (ros::service::call("slam_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "record deepps virtual parkinglot",
                           "Failed to record deepps virtual parkinglot!");
    else
      ui_.status->setText("Status: Record deepps virtual parkinglot!");
  }
  else
    QMessageBox::warning(
        widget_, "record deepps virtual parkinglot",
        "Failed to call record deepps virtual parkinglot service!");

  return;
}

void StateMachineController::onSlamRemoveDefaultMapFile()
{
  std::string package_path = ros::package::getPath(PACKAGE_SLAM);
  std::string cmd = "rm " + package_path + "/../../ORB_SLAM2/files/1.bin";

  int output = system(cmd.c_str());
  ui_.status->setText("Status: Remove default map file!");
}

// navigation & following state control functions
void StateMachineController::onNavigationStart()
{
  // publish topic to start navigation
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  navigation_goal_pub_.publish(msg);

  navi_status_ = StateMachineStatus::Navigation::RUNNING;
  updateNaviStatusUI();

  ui_.status->setText("Status: Navigation goal sent!");
}

void StateMachineController::onNavigationCancel()
{
  // publish topic to stop navigation
  std_msgs::Empty msg;
  navigation_cancel_pub_.publish(msg);

  navi_status_ = StateMachineStatus::Navigation::IDLE;
  updateNaviStatusUI();

  ui_.status->setText("Status: Navigation cancel sent!");
}

void StateMachineController::onFollowingStart()
{
  // enable vehicle control
  onVehicleControlEnable();

  if (vehicle_ctrl_status_ == StateMachineStatus::VehicleControl::RUNNING)
  { // update navi status after vehicle control enabled
    navi_status_ = StateMachineStatus::Navigation::RUNNING;
    updateNaviStatusUI();
  }

  return;
}

void StateMachineController::onFollowingStop()
{
  // disable vehicle control
  onVehicleControlDisable();

  if (vehicle_ctrl_status_ == StateMachineStatus::VehicleControl::IDLE)
  { // update navi status after vehicle control disabled
    navi_status_ = StateMachineStatus::Navigation::IDLE;
    updateNaviStatusUI();
  }

  return;
}

// freespace state control functions
void StateMachineController::onFreespaceStart()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 1;

  if (ros::service::call("freespace_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "start", "Failed to start freespace!");
    else
    {
      freespace_status_ = StateMachineStatus::Freespace::RUNNING;
      updateFreespaceStatusUI();
      ui_.status->setText("Status: Start freespace detection!");
    }
  }
  else
    QMessageBox::warning(widget_, "start",
                         "Failed to call start freespace service!");
}

void StateMachineController::onFreespaceStop()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 0;

  if (ros::service::call("freespace_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "stop", "Failed to stop freespace!");
    else
    {
      freespace_status_ = StateMachineStatus::Freespace::IDLE;
      updateFreespaceStatusUI();
      ui_.status->setText("Status: Stop freespace detection!");
    }
  }
  else
    QMessageBox::warning(widget_, "stop",
                         "Failed to call stop freespace service!");
}

void StateMachineController::onFreespaceTestObstacleEnable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 2;

  if (ros::service::call("freespace_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "enable",
                           "Failed to Enable Test Obstacle!");
    else
    {
      ui_.status->setText("Status: Enable test obstacle!");
    }
  }
  else
    QMessageBox::warning(widget_, "enable",
                         "Failed to call enable test obstacle service!");
}

void StateMachineController::onFreespaceTestObstacleDisable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 3;

  if (ros::service::call("freespace_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "disalbe",
                           "Failed to Disable Test Obstacle!");
    else
    {
      ui_.status->setText("Status: Disable test obstacle!");
    }
  }
  else
    QMessageBox::warning(widget_, "disalbe",
                         "Failed to call disable test obstacle service!");
}

void StateMachineController::changeTestObstacleState()
{
  if (ui_.enableTestObstacle->isChecked())
  {
    onFreespaceTestObstacleEnable();
    ui_.status->setText("Status: Enable obstacles for testing.");
  }
  else
  {
    onFreespaceTestObstacleDisable();
    ui_.status->setText("Status: Disable obstacles for testing.");
  }
}

void StateMachineController::updateTestObstacle()
{
  double obstacle_x, obstacle_y;
  double obstacle_size, obstacle_resolution;

  if (ui_.inputObstacleX->text().isEmpty() ||
      ui_.inputObstacleY->text().isEmpty() ||
      ui_.inputObstacleSize->text().isEmpty() ||
      ui_.inputObstacleResolution->text().isEmpty())
  {
    ui_.status->setText("Status: Not all params of obstacle are given!");
    return;
  }
  else
  {
    obstacle_x = ui_.inputObstacleX->text().toDouble();
    obstacle_y = ui_.inputObstacleY->text().toDouble();
    obstacle_size = ui_.inputObstacleSize->text().toDouble();
    obstacle_resolution = ui_.inputObstacleResolution->text().toDouble();
  }

  std::string param_obstacle_x =
      FREESPACE_NODE_NAME + std::string("/test_obstacle_x");
  std::string param_obstacle_y =
      FREESPACE_NODE_NAME + std::string("/test_obstacle_y");
  std::string param_obstacle_size =
      FREESPACE_NODE_NAME + std::string("/test_obstacle_size");
  std::string param_obstacle_resolution =
      FREESPACE_NODE_NAME + std::string("/test_obstacle_resolution");

  nh_.setParam(param_obstacle_x, obstacle_x);
  nh_.setParam(param_obstacle_y, obstacle_y);
  nh_.setParam(param_obstacle_size, obstacle_size);
  nh_.setParam(param_obstacle_resolution, obstacle_resolution);

  ui_.status->setText("Status: Test obstacle params are updated!");
}

void StateMachineController::onFreespaceSurroundEnable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 4;

  if (ros::service::call("freespace_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "enable",
                           "Failed to Enable Surround Obstacle!");
    else
    {
      ui_.status->setText("Status: Enable surround obstacle!");
    }
  }
  else
    QMessageBox::warning(widget_, "enable",
                         "Failed to call enable surround obstacle service!");
}

void StateMachineController::onFreespaceSurroundDisable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 5;

  if (ros::service::call("freespace_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "disalbe",
                           "Failed to Disable Surround Obstacle!");
    else
    {
      ui_.status->setText("Status: Disable surround obstacle!");
    }
  }
  else
    QMessageBox::warning(widget_, "disalbe",
                         "Failed to call disable surround obstacle service!");
}

void StateMachineController::onFreespaceFrontEnable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 6;

  if (ros::service::call("freespace_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "enable",
                           "Failed to Enable Front Obstacle!");
    else
    {
      ui_.status->setText("Status: Enable front obstacle!");
    }
  }
  else
    QMessageBox::warning(widget_, "enable",
                         "Failed to call enable front obstacle service!");
}

void StateMachineController::onFreespaceFrontDisable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 1;
  srv.request.action.command = 7;

  if (ros::service::call("freespace_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "disalbe",
                           "Failed to Disable Front Obstacle!");
    else
    {
      ui_.status->setText("Status: Disable front obstacle!");
    }
  }
  else
    QMessageBox::warning(widget_, "disalbe",
                         "Failed to call disable front obstacle service!");
}

// vehicle control state control functions
void StateMachineController::onVehicleControlEnable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 1;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "enable",
                           "Failed to enable vehicle_control!");
    else
    {
      vehicle_ctrl_status_ = StateMachineStatus::VehicleControl::RUNNING;
      updateCtrlStatusUI();
      ui_.status->setText("Status: Enable vehicle control!");
    }
  }
  else
    QMessageBox::warning(widget_, "enable",
                         "Failed to call enable vehicle_control service!");

  return;
}

void StateMachineController::onVehicleControlDisable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 0;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "diable",
                           "Failed to diable vehicle_control!");
    else
    {
      vehicle_ctrl_status_ = StateMachineStatus::VehicleControl::IDLE;
      updateCtrlStatusUI();
      ui_.status->setText("Status: Disable vehicle control!");
    }
  }
  else
    QMessageBox::warning(widget_, "diable",
                         "Failed to call diable vehicle_control service!");

  return;
}

void StateMachineController::onVehicleControlEStop()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 2;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "diable",
                           "Failed to E-Stop vehicle_control!");
    else
    {
      vehicle_ctrl_status_ = StateMachineStatus::VehicleControl::IDLE;
      updateCtrlStatusUI();
      ui_.status->setText("Status: E-Stop vehicle control!");
    }
  }
  else
    QMessageBox::warning(widget_, "diable",
                         "Failed to call E-Stop vehicle_control service!");

  return;
}

void StateMachineController::onVehicleControlSoftStop()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 3;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "diable",
                           "Failed to Soft Stop vehicle_control!");
    else
    {
      vehicle_ctrl_status_ = StateMachineStatus::VehicleControl::IDLE;
      updateCtrlStatusUI();
      ui_.status->setText("Status: Soft Stop vehicle control!");
    }
  }
  else
    QMessageBox::warning(widget_, "diable",
                         "Failed to call Soft Stop vehicle_control service!");

  return;
}

void StateMachineController::onSonarEnable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 4;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "enable", "Failed to enable sonar!");
    else
    {
      vehicle_ctrl_status_ = StateMachineStatus::VehicleControl::IDLE;
      updateCtrlStatusUI();
      ui_.status->setText("Status: Enable sonar!");
    }
  }
  else
    QMessageBox::warning(
        widget_, "enable",
        "Failed to call enable sonar vehicle_control service!");

  return;
}

void StateMachineController::onSonarDisable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 5;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "diable", "Failed to disable sonar!");
    else
    {
      vehicle_ctrl_status_ = StateMachineStatus::VehicleControl::IDLE;
      updateCtrlStatusUI();
      ui_.status->setText("Status: Disable sonar!");
    }
  }
  else
    QMessageBox::warning(
        widget_, "diable",
        "Failed to call disable sonar vehicle_control service!");

  return;
}

void StateMachineController::onFrontEsrEnable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 6;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "enable", "Failed to enable front esr!");
    else
    {
      vehicle_ctrl_status_ = StateMachineStatus::VehicleControl::IDLE;
      updateCtrlStatusUI();
      ui_.status->setText("Status: Enable front esr!");
    }
  }
  else
    QMessageBox::warning(
        widget_, "enable",
        "Failed to call enable front esr vehicle_control service!");

  return;
}

void StateMachineController::onFrontEsrDisable()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 7;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "diable", "Failed to disable front esr!");
    else
    {
      vehicle_ctrl_status_ = StateMachineStatus::VehicleControl::IDLE;
      updateCtrlStatusUI();
      ui_.status->setText("Status: Disable front esr!");
    }
  }
  else
    QMessageBox::warning(
        widget_, "diable",
        "Failed to call disable front esr vehicle_control service!");

  return;
}

void StateMachineController::setSliderSpeed(int speed)
{
  ui_.sliderSpeed->setValue(speed);
  double s = static_cast<double>(speed) / 10.0; // align scale
  ui_.inputKeyboardSpeed->setText(QString::number(s, 'f', 1));
}

void StateMachineController::setDialSteering(int steering)
{
  ui_.dialSteering->setValue(steering);
  double s = static_cast<double>(steering) / -1.0; // align scale and dir
  ui_.inputKeyboardSteering->setText(QString::number(s, 'f', 1));
}

void StateMachineController::stopKeyboardControl()
{
  setKeyboardSpeedZero();
  setKeyboardSteeringZero();
}

void StateMachineController::setKeyboardSpeedZero()
{
  ui_.sliderSpeed->setValue(0.0);
  ui_.inputKeyboardSpeed->setText(QString::number(0.0, 'f', 1));
}

void StateMachineController::setKeyboardSteeringZero()
{
  ui_.dialSteering->setValue(0.0);
  ui_.inputKeyboardSteering->setText(QString::number(0.0, 'f', 1));
}

void StateMachineController::keyboardControlEnable()
{
  if (ui_.enableKeyboardControl->isChecked())
  {
    keyboardControlTimer_.start();
    ui_.status->setText("Status: Enable keyboard control.");
  }
  else
  {
    keyboardControlTimer_.stop();
    ui_.status->setText("Status: Disable keyboard control.");
  }
}

void StateMachineController::keyboardControlPublish()
{
  double speed = static_cast<double>(ui_.sliderSpeed->value()) / 10.0;
  double steering = static_cast<double>(ui_.dialSteering->value()) / -1.0;

  geometry_msgs::Twist cmd_msg;
  cmd_msg.linear.x = speed;
  cmd_msg.angular.z = steering / 180.0 * M_PI;

  keyboard_control_pub_.publish(cmd_msg);
}

void StateMachineController::changeLcmMonitorState()
{
  if (ui_.enableLcmMonitor->isChecked())
  {
    lcmMonitorEnabled_ = true;
    lcmCheckingTimer_.start();
    ui_.status->setText("Status: Enable LCM monitoring.");
  }
  else
  {
    lcmMonitorEnabled_ = false;
    lcmCheckingTimer_.stop();
    ui_.status->setText("Status: Disable LCM monitoring.");
  }
}

void StateMachineController::changeLcmAckermannCmdState()
{
  if (ui_.enableLcmAckermannCmd->isChecked())
  {
    ackermann_cmd_lcm_sub_ =
        lcm_->subscribe(LCM_CHANNEL_ACKERMANN_CMD,
                        &StateMachineController::updateAckermannCmdLcm, this);
    ui_.status->setText("Status: Enable LCM Ackermann Cmd.");
  }
  else
  {
    lcm_->unsubscribe(ackermann_cmd_lcm_sub_);
    ui_.status->setText("Status: Disable LCM Ackermann Cmd.");
  }
}

void StateMachineController::changeLcmAckermannOdomState()
{
  if (ui_.enableLcmAckermannOdom->isChecked())
  {
    ackermann_odom_lcm_sub_ =
        lcm_->subscribe(LCM_CHANNEL_ACKERMANN_ODOM,
                        &StateMachineController::updateAckermannOdomLcm, this);
    ui_.status->setText("Status: Enable LCM Ackermann Odom.");
  }
  else
  {
    lcm_->unsubscribe(ackermann_odom_lcm_sub_);
    ui_.status->setText("Status: Disable LCM Ackermann Odom.");
  }
}

void StateMachineController::changeLcmSensorSonarState()
{
  if (ui_.enableLcmSensorSonar->isChecked())
  {
    sensor_sonar_lcm_sub_ =
        lcm_->subscribe(LCM_CHANNEL_SENSOR_SONAR,
                        &StateMachineController::updateSensorSonarLcm, this);
    ui_.status->setText("Status: Enable LCM Sensor Sonar.");
  }
  else
  {
    lcm_->unsubscribe(sensor_sonar_lcm_sub_);
    ui_.status->setText("Status: Disable LCM Sensor Sonar.");
  }
}

void StateMachineController::changeLcmSensorImuState()
{
  if (ui_.enableLcmSensorImu->isChecked())
  {
    sensor_imu_lcm_sub_ =
        lcm_->subscribe(LCM_CHANNEL_SENSOR_IMU,
                        &StateMachineController::updateSensorImuLcm, this);
    ui_.status->setText("Status: Enable LCM Sensor Imu.");
  }
  else
  {
    lcm_->unsubscribe(sensor_imu_lcm_sub_);
    ui_.status->setText("Status: Disable LCM Sensor Imu.");
  }
}

void StateMachineController::changeLcmSensorEgomotionState()
{
  if (ui_.enableLcmSensorEgomotion->isChecked())
  {
    sensor_egomotion_lcm_sub_ = lcm_->subscribe(
        LCM_CHANNEL_SENSOR_EGOMOTION,
        &StateMachineController::updateSensorEgomotionLcm, this);
    ui_.status->setText("Status: Enable LCM Sensor front ESR.");
  }
  else
  {
    lcm_->unsubscribe(sensor_egomotion_lcm_sub_);
    ui_.status->setText("Status: Disable LCM Sensor front ESR.");
  }
}

void StateMachineController::changeLcmSensorEsrFrontState()
{
  if (ui_.enableLcmSensorEsrFront->isChecked())
  {
    sensor_esr_front_lcm_sub_ =
        lcm_->subscribe(LCM_CHANNEL_SENSOR_ESR_FRONT,
                        &StateMachineController::updateSensorEsrFrontLcm, this);
    ui_.status->setText("Status: Enable LCM Sensor front ESR.");
  }
  else
  {
    lcm_->unsubscribe(sensor_esr_front_lcm_sub_);
    ui_.status->setText("Status: Disable LCM Sensor front ESR.");
  }
}

void StateMachineController::changeLcmParkinglotCtrlState()
{
  if (ui_.enableLcmParkinglotCtrl->isChecked())
  {
    parkinglot_ctrl_lcm_sub_ =
        lcm_->subscribe(LCM_CHANNEL_PARKINGLOT_CTRL,
                        &StateMachineController::updateParkinglotCtrlLcm, this);
    ui_.status->setText("Status: Enable LCM Parkinglot Ctrl.");
  }
  else
  {
    lcm_->unsubscribe(parkinglot_ctrl_lcm_sub_);
    ui_.status->setText("Status: Disable LCM Parkinglot Ctrl.");
  }
}

void StateMachineController::lcmChecking() { lcm_->handleTimeout(10); }

void StateMachineController::resetLcmOutput()
{
  ui_.dataAckermannCmd->setText("");
  ui_.dataAckermannOdom->setText("");
  ui_.dataSensorSonar->setText("");
  ui_.dataSensorImu->setText("");
  ui_.dataSensorEgomotion->setText("");
  ui_.dataSensorEsrFront->setText("");

  return;
}

void StateMachineController::updateAckermannCmdLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const vel_conversion::ackermann_cmd* msg)
{
  QString data;
  data += "enable: " + QString::number(msg->ros_planner_enabled, 'f', 0);
  data += " speed: " + QString::number(msg->speed, 'f', 2);
  data += " angle: " + QString::number(msg->steering_angle, 'f', 2);

  ui_.dataAckermannCmd->setText(data);
}

void StateMachineController::updateAckermannOdomLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const vel_conversion::ackermann_odom* msg)
{
  QString data;
  data += "speed: " + QString::number(msg->speed, 'f', 2);
  data += " left speed: " + QString::number(msg->left_speed, 'f', 2);
  data += " right speed: " + QString::number(msg->right_speed, 'f', 2);
  data += " angle: " + QString::number(msg->steering_angle, 'f', 2);

  ui_.dataAckermannOdom->setText(data);
}

void StateMachineController::updateSensorSonarLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const sensor::sonar* msg)
{
  QString data;
  for (int i = 0; i < 12; i++)
    data += QString::number(msg->measurement[i], 'f', 2) + ' ';

  ui_.dataSensorSonar->setText(data);
}

void StateMachineController::updateSensorImuLcm(const lcm::ReceiveBuffer* rbuf,
                                                const std::string& chan,
                                                const sensor::imu* msg)
{
  QString data;
  data += "acc: ";
  for (int i = 0; i < 3; i++)
    data += QString::number(msg->acc[i], 'f', 2) + ' ';

  data += " gyro: ";
  for (int i = 0; i < 3; i++)
    data += QString::number(msg->gyro[i], 'f', 2) + ' ';

  ui_.dataSensorImu->setText(data);
}

void StateMachineController::updateSensorEgomotionLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const sensor::egomotion_fusion* msg)
{
  QString data;
  data += QString::number(msg->longitude, 'f', 2) + ' ';
  data += QString::number(msg->latitude, 'f', 2) + ' ';
  data += "rpy: ";
  data += QString::number(msg->roll, 'f', 2) + ' ';
  data += QString::number(msg->pitch, 'f', 2) + ' ';
  data += QString::number(msg->yaw, 'f', 2);

  ui_.dataSensorEgomotion->setText(data);
}

void StateMachineController::updateSensorEsrFrontLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const sensor::esr* msg)
{
  QString data;
  for (int i = 0; i < 64; i++)
  {
    if (msg->object[i].status != 0)
    {
      data += QString::number(i) + ":[";
      data += QString::number(msg->object[i].angle, 'f', 2) + ',';
      data += QString::number(msg->object[i].range, 'f', 2) + "] ";
    }
  }

  ui_.dataSensorEsrFront->setText(data);
}

void StateMachineController::updateParkinglotCtrlLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const OFILM_CTRL* msg)
{
  QString data;
  data += "Search:" + QString::number(msg->isOfilmParkingSearchFuncEnable);
  data += " Track:" + QString::number(msg->isOfilmParkinglotTrackingFuncEnable);
  data += " Tracking ID:" + QString::number(msg->isOfilmParkinglotTrackingID);
  data += " Reckon:" + QString::number(msg->isOfilmLocationFuncEnable);

  ui_.dataParkinglotCtrl->setText(data);
}

void StateMachineController::onDeeppsStart()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 2;
  srv.request.action.command = 1;

  if (ros::service::call("deepps_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "start", "Failed to start deepps!");
    else
    {
      deepps_status_ = StateMachineStatus::Deepps::RUNNING;
      updateDeeppsStatusUI();
      ui_.status->setText("Status: Start parking lot detection!");
    }
  }
  else
    QMessageBox::warning(widget_, "start",
                         "Failed to call start deepps service!");

  return;
}

void StateMachineController::onDeeppsStop()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 2;
  srv.request.action.command = 0;

  if (ros::service::call("deepps_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "stop", "Failed to stop deepps!");
    else
    {
      deepps_status_ = StateMachineStatus::Deepps::IDLE;
      updateDeeppsStatusUI();
      ui_.status->setText("Status: Stop parking lot detection!");
    }
  }
  else
    QMessageBox::warning(widget_, "stop",
                         "Failed to call stop deepps service!");

  return;
}

void StateMachineController::enableParkinglot()
{
  if (ui_.enableParkinglot->isChecked())
  {
    ui_.numParkinglot->setEnabled(true);
    ui_.applyParkinglotId->setEnabled(true);
    ui_.clearParkinglotId->setEnabled(true);
  }
  else
  {
    ui_.numParkinglot->setDisabled(true);
    ui_.applyParkinglotId->setDisabled(true);
    ui_.clearParkinglotId->setDisabled(true);
  }

  return;
}

void StateMachineController::applyParkinglotId()
{
  int id = ui_.numParkinglot->value();

  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 2;
  srv.request.action.command = 2;
  srv.request.action.data = id;

  if (ros::service::call("deepps_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "enable parkinglot",
                           "Failed to enable parkinglot!");
    else
    {
      ui_.status->setText("Status: Enable parkinglot!");
    }
  }
  else
    QMessageBox::warning(widget_, "enable parkinglot",
                         "Failed to call enable parkinglot service!");
}

void StateMachineController::clearParkinglotId()
{
  ui_.numParkinglot->setValue(-1);
}

void StateMachineController::getDeeppsStartPos()
{
  // get deepps start position
  double x, y;
  QString deepps_start_pos_string;
  if (nh_.getParam("/orb_slam_2_ros_node/deepps_start_pos_x", x) &&
      nh_.getParam("/orb_slam_2_ros_node/deepps_start_pos_y", y))
  {
    deepps_start_pos_.setX(x);
    deepps_start_pos_.setY(y);

    deepps_start_pos_string = "(" + QString::number(x, 'f', 2) + ", " +
                              QString::number(y, 'f', 2) + ")";

    visualization_msgs::Marker start_pos_marker;
    start_pos_marker.header.stamp = ros::Time::now();
    start_pos_marker.header.frame_id = "map";
    start_pos_marker.type = visualization_msgs::Marker::SPHERE;
    start_pos_marker.id = 0;
    start_pos_marker.pose.position.x = x;
    start_pos_marker.pose.position.y = y;
    start_pos_marker.scale.x = 1.2;
    start_pos_marker.scale.y = 1.2;
    start_pos_marker.scale.z = 1.2;
    start_pos_marker.color.r = 1.0;
    start_pos_marker.color.g = 0.0;
    start_pos_marker.color.b = 1.0;
    start_pos_marker.color.a = 0.8;

    deepps_start_pos_pub_.publish(start_pos_marker);
  }
  else
  {
    ROS_WARN("Failed to get deepps start position!");
    deepps_start_pos_string = "(N/A, N/A)";
  }

  ui_.deeppsStartPos->setText(deepps_start_pos_string);
}

void StateMachineController::enableVirtualParkinglot()
{
  if (ui_.enableVirtualParkinglot->isChecked())
  {
    ui_.applyVirtualParkinglot->setEnabled(true);
    ui_.refreshVirtualParkinglot->setEnabled(true);
  }
  else
  {
    ui_.applyVirtualParkinglot->setDisabled(true);
    ui_.refreshVirtualParkinglot->setDisabled(true);
    ui_.statusVirtualParkinglot->setText("");

    // call deepps to disable virtual parkinglot
    state_machine_msgs::ActionControl srv;
    srv.request.action.module = 2;
    srv.request.action.command = 3;
    srv.request.action.data = 0;

    if (ros::service::call("deepps_state_control", srv))
    {
      if (!srv.response.feedback)
        QMessageBox::warning(widget_, "disable virtual parkinglot",
                             "Failed to disable virtual parkinglot!");
      else
      {
        ui_.status->setText("Status: Disable virtual parkinglot!");
      }
    }
    else
      QMessageBox::warning(
          widget_, "disable virtual parkinglot",
          "Failed to call disable virtual parkinglot service!");
  }

  return;
}

void StateMachineController::applyVirtualParkinglot()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 2;
  srv.request.action.command = 3;
  srv.request.action.data = 1;

  if (ros::service::call("deepps_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "enable virtual parkinglot",
                           "Failed to enable virtual parkinglot!");
    else
    {
      ui_.status->setText("Status: Enable virtual parkinglot!");
    }
  }
  else
    QMessageBox::warning(widget_, "enable virtual parkinglot",
                         "Failed to call enable virtual parkinglot service!");
}

void StateMachineController::refreshVirtualParkinglot()
{
  QString virtual_parkinglot_output;

  double p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y;

  const std::string p1_x_name = SLAM_NODE_NAME + std::string("/parkinglot_x1");
  const std::string p1_y_name = SLAM_NODE_NAME + std::string("/parkinglot_y1");
  const std::string p2_x_name = SLAM_NODE_NAME + std::string("/parkinglot_x2");
  const std::string p2_y_name = SLAM_NODE_NAME + std::string("/parkinglot_y2");
  const std::string p3_x_name = SLAM_NODE_NAME + std::string("/parkinglot_x3");
  const std::string p3_y_name = SLAM_NODE_NAME + std::string("/parkinglot_y3");
  const std::string p4_x_name = SLAM_NODE_NAME + std::string("/parkinglot_x4");
  const std::string p4_y_name = SLAM_NODE_NAME + std::string("/parkinglot_y4");

  if (nh_.getParam(p1_x_name, p1_x) && nh_.getParam(p1_y_name, p1_y) &&
      nh_.getParam(p2_x_name, p2_x) && nh_.getParam(p2_y_name, p2_y) &&
      nh_.getParam(p3_x_name, p3_x) && nh_.getParam(p3_y_name, p3_y) &&
      nh_.getParam(p4_x_name, p4_x) && nh_.getParam(p4_y_name, p4_y))
  {
    virtual_parkinglot_output += "p1: (";
    virtual_parkinglot_output += QString::number(p1_x, 'f', 2) + ", ";
    virtual_parkinglot_output += QString::number(p1_y, 'f', 2) + ") ";
    virtual_parkinglot_output += "p2: (";
    virtual_parkinglot_output += QString::number(p2_x, 'f', 2) + ", ";
    virtual_parkinglot_output += QString::number(p2_y, 'f', 2) + ")\n";
    virtual_parkinglot_output += "p3: (";
    virtual_parkinglot_output += QString::number(p3_x, 'f', 2) + ", ";
    virtual_parkinglot_output += QString::number(p3_y, 'f', 2) + ") ";
    virtual_parkinglot_output += "p4: (";
    virtual_parkinglot_output += QString::number(p4_x, 'f', 2) + ", ";
    virtual_parkinglot_output += QString::number(p4_y, 'f', 2) + ")";

    // publish virtual parkinglot
    geometry_msgs::PolygonStamped virtual_parkinglot_msg;
    virtual_parkinglot_msg.header.frame_id = "map";
    virtual_parkinglot_msg.header.stamp = ros::Time::now();

    geometry_msgs::Point32 lot_p1, lot_p2, lot_p3, lot_p4;
    lot_p1.x = p1_x;
    lot_p1.y = p1_y;
    lot_p2.x = p2_x;
    lot_p2.y = p2_y;
    lot_p3.x = p3_x;
    lot_p3.y = p3_y;
    lot_p4.x = p4_x;
    lot_p4.y = p4_y;

    virtual_parkinglot_msg.polygon.points.push_back(lot_p1);
    virtual_parkinglot_msg.polygon.points.push_back(lot_p2);
    virtual_parkinglot_msg.polygon.points.push_back(lot_p3);
    virtual_parkinglot_msg.polygon.points.push_back(lot_p4);

    virtual_parkinglot_pub_.publish(virtual_parkinglot_msg);
  }
  else
  {
    virtual_parkinglot_output += "p1: N/A, p2: N/A \n";
    virtual_parkinglot_output += "p3: N/A, p4: N/A";
    ui_.status->setText("Status: Cannot load virtual parkinglot.");
  }

  ui_.statusVirtualParkinglot->setText(virtual_parkinglot_output);
}

void StateMachineController::enableFreespaceParkinglot()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 2;
  srv.request.action.command = 4;
  srv.request.action.data = ui_.enableFreespaceParkinglot->isChecked() ? 1 : 0;

  if (ros::service::call("deepps_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "enable virtual parkinglot",
                           "Failed to enable virtual parkinglot!");
    else
    {
      ui_.status->setText("Status: Enable virtual parkinglot!");
    }
  }
  else
    QMessageBox::warning(widget_, "enable virtual parkinglot",
                         "Failed to call enable virtual parkinglot service!");
}

void StateMachineController::onParkingStart()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 8;
  srv.request.action.data = 1;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "start parkinglot tracking",
                           "Failed to start parkinglot tracking!");
    else
    {
      ui_.status->setText("Status: Start parkinglot tracking!");
    }
  }
  else
    QMessageBox::warning(widget_, "start",
                         "Failed to call start parkinglot tracking service!");
}

void StateMachineController::onParkingStop()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 4;
  srv.request.action.command = 8;
  srv.request.action.data = 0;

  if (ros::service::call("vehicle_control_state_control", srv))
  {
    if (!srv.response.feedback)
      QMessageBox::warning(widget_, "stop parkinglot tracking",
                           "Failed to stop parkinglot tracking!");
    else
    {
      ui_.status->setText("Status: Stop parkinglot tracking!");
    }
  }
  else
    QMessageBox::warning(widget_, "stop",
                         "Failed to call stop parkinglot tracking service!");
}

void StateMachineController::parkinglotStatusCB(
    const parkinglot_msgs::ParkingLotDetectionStatusStamped::ConstPtr msg)
{
  ui_.status->setText("Status: parking lot status received!");

  if (msg->ParkinglotSearchEnabled)
    deepps_status_ = StateMachineStatus::Deepps::RUNNING;
  else
    deepps_status_ = StateMachineStatus::Deepps::IDLE;

  // update UI
  updateDeeppsStatusUI();

  return;
}

void StateMachineController::parkinglotCtrlCB(
    const parkinglot_msgs::ParkingLotDetectionCtrlStamped::ConstPtr msg)
{
  ui_.status->setText("Status: parking lot ctrl received!");

  if (msg->isParkinglotTrackingFuncEnable)
    parking_status_ = StateMachineStatus::ParkingPlanning::TRACKING;
  else
    parking_status_ = StateMachineStatus::ParkingPlanning::IDLE;

  // update UI
  updateParkingStatusUI();

  return;
}

void StateMachineController::updateSlamStatusUI()
{
  switch (slam_status_)
  {

  case StateMachineStatus::Slam::RUNNING:
  {
    ui_.statusSlam->setText("Running...");
    break;
  }

  case StateMachineStatus::Slam::IDLE:
  {
    ui_.statusSlam->setText("Idle");
    break;
  }

  case StateMachineStatus::Slam::ERROR:
  {
    ui_.statusSlam->setText("ERROR!");
    break;
  }

  default:
    break;
  }
}

void StateMachineController::updateFreespaceStatusUI()
{
  switch (freespace_status_)
  {

  case StateMachineStatus::Freespace::RUNNING:
  {
    ui_.statusFreespace->setText("Running...");
    break;
  }

  case StateMachineStatus::Freespace::IDLE:
  {
    ui_.statusFreespace->setText("Idle");
    break;
  }

  case StateMachineStatus::Freespace::ERROR:
  {
    ui_.statusFreespace->setText("ERROR!");
    break;
  }

  default:
    break;
  }
}

void StateMachineController::updateNaviStatusUI()
{
  switch (navi_status_)
  {

  case StateMachineStatus::Navigation::RUNNING:
  {
    ui_.statusNavi->setText("Running...");
    break;
  }

  case StateMachineStatus::Navigation::IDLE:
  {
    ui_.statusNavi->setText("Idle");
    break;
  }

  case StateMachineStatus::Navigation::ERROR:
  {
    ui_.statusNavi->setText("ERROR!");
    break;
  }

  default:
    break;
  }
}

void StateMachineController::updateCtrlStatusUI()
{
  switch (vehicle_ctrl_status_)
  {

  case StateMachineStatus::VehicleControl::RUNNING:
  {
    ui_.statusCtrl->setText("Running...");
    break;
  }

  case StateMachineStatus::VehicleControl::IDLE:
  {
    ui_.statusCtrl->setText("Idle");
    break;
  }

  case StateMachineStatus::VehicleControl::ERROR:
  {
    ui_.statusCtrl->setText("ERROR!");
    break;
  }

  default:
    break;
  }
}

void StateMachineController::updateDeeppsStatusUI()
{
  switch (deepps_status_)
  {

  case StateMachineStatus::Deepps::RUNNING:
  {
    ui_.statusDeepps->setText("Running...");
    break;
  }

  case StateMachineStatus::Deepps::IDLE:
  {
    ui_.statusDeepps->setText("Idle");
    break;
  }

  case StateMachineStatus::Deepps::ERROR:
  {
    ui_.statusDeepps->setText("ERROR!");
    break;
  }

  default:
    break;
  }
}

void StateMachineController::updateParkingStatusUI()
{
  switch (parking_status_)
  {

  case StateMachineStatus::ParkingPlanning::RUNNING:
  {
    ui_.statusParking->setText("Running...");
    break;
  }

  case StateMachineStatus::ParkingPlanning::IDLE:
  {
    ui_.statusParking->setText("Idle");
    break;
  }

  case StateMachineStatus::ParkingPlanning::ERROR:
  {
    ui_.statusParking->setText("ERROR!");
    break;
  }

  case StateMachineStatus::ParkingPlanning::TRACKING:
  {
    ui_.statusParking->setText("Tracking...");
    break;
  }

  default:
    break;
  }
}

void StateMachineController::resetStatusUI()
{
  ui_.statusSlam->setText("Idle");
  ui_.statusNavi->setText("Idle");
  ui_.statusCtrl->setText("Idle");
  ui_.statusFreespace->setText("Idle");
  ui_.statusSsd->setText("Idle");
  ui_.statusDeepps->setText("Idle");
  ui_.statusParking->setText("Idle");

  ui_.status->setText("Status: Reset status.");
}

bool StateMachineController::eventFilter(QObject* target, QEvent* event)
{
  if (event->type() == QEvent::KeyPress)
  {
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);

    if (keyEvent->key() == Qt::Key_W)
    {
      int speed = ui_.sliderSpeed->value();
      speed += ui_.sliderSpeed->singleStep();
      ui_.sliderSpeed->setValue(speed);

      double s = static_cast<double>(speed) / 10.0; // align scale
      ui_.inputKeyboardSpeed->setText(QString::number(s, 'f', 1));
    }

    if (keyEvent->key() == Qt::Key_S)
    {
      int speed = ui_.sliderSpeed->value();
      speed -= ui_.sliderSpeed->singleStep();
      ui_.sliderSpeed->setValue(speed);

      double s = static_cast<double>(speed) / 10.0; // align scale
      ui_.inputKeyboardSpeed->setText(QString::number(s, 'f', 1));
    }

    if (keyEvent->key() == Qt::Key_A)
    {
      int steering = ui_.dialSteering->value();
      steering -= ui_.dialSteering->singleStep();
      ui_.dialSteering->setValue(steering);

      double s = static_cast<double>(steering) / -1.0; // align scale and dir
      ui_.inputKeyboardSteering->setText(QString::number(s, 'f', 1));
    }

    if (keyEvent->key() == Qt::Key_D)
    {
      int steering = ui_.dialSteering->value();
      steering += ui_.dialSteering->singleStep();
      ui_.dialSteering->setValue(steering);

      double s = static_cast<double>(steering) / -1.0; // align scale and dir
      ui_.inputKeyboardSteering->setText(QString::number(s, 'f', 1));
    }

    if (keyEvent->key() == Qt::Key_Escape)
      stopKeyboardControl();
  }
  return rqt_gui_cpp::Plugin::eventFilter(target, event);
}

void StateMachineController::stateChecking()
{
  // check slam mode
  checkSlamMode();
  // check slam status
  checkSlamStatus();
  // check slam path recording
  checkSlamPathRecording();
  // check slam map scale
  checkSlamMapScale();

  if (navi_status_ == StateMachineStatus::Navigation::RUNNING &&
      deepps_status_ == StateMachineStatus::Deepps::IDLE)
  {
    // check if time to start deepps
    getDeeppsStartPos();
    checkDeeppsStartCondition();
  }

  return;
}

void StateMachineController::checkSlamStatus()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 12;

  if (ros::service::call("slam_state_control", srv))
  {
    ui_.radioSlamRunning->setEnabled(true);
    ui_.radioSlamIdle->setEnabled(true);

    if (srv.response.feedback == 0)
    {
      ui_.radioSlamRunning->setChecked(false);
      ui_.radioSlamIdle->setChecked(true);

      ui_.radioTrackingSuccessful->setEnabled(false);
      ui_.radioTrackingFailed->setEnabled(false);
    }
    else
    {
      ui_.radioSlamRunning->setChecked(true);
      ui_.radioSlamIdle->setChecked(false);

      ui_.radioTrackingSuccessful->setEnabled(true);
      ui_.radioTrackingFailed->setEnabled(true);

      if (srv.response.feedback == 1)
      {
        ui_.radioTrackingSuccessful->setChecked(true);
        ui_.radioTrackingFailed->setChecked(false);
      }
      else
      {
        ui_.radioTrackingSuccessful->setChecked(false);
        ui_.radioTrackingFailed->setChecked(true);
      }
    }
  }
  else
  {
    ui_.radioSlamRunning->setEnabled(false);
    ui_.radioSlamIdle->setEnabled(false);
    ui_.radioTrackingSuccessful->setEnabled(false);
    ui_.radioTrackingFailed->setEnabled(false);
  }
}

void StateMachineController::checkSlamMode()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 11;

  if (ros::service::call("slam_state_control", srv))
  {
    ui_.radioModeSlam->setEnabled(true);
    ui_.radioModeLocalization->setEnabled(true);

    if (srv.response.feedback == 0)
    {
      ui_.radioModeSlam->setChecked(true);
      ui_.radioModeLocalization->setChecked(false);
    }
    else
    {
      ui_.radioModeSlam->setChecked(false);
      ui_.radioModeLocalization->setChecked(true);
    }
  }
  else
  {
    ui_.radioModeSlam->setEnabled(false);
    ui_.radioModeLocalization->setEnabled(false);
  }
}

void StateMachineController::checkSlamPathRecording()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 13;

  if (ros::service::call("slam_state_control", srv))
  {
    ui_.radioPathRecording->setEnabled(true);
    ui_.radioPathIdle->setEnabled(true);

    if (srv.response.feedback == 1)
    {
      ui_.radioPathRecording->setChecked(true);
      ui_.radioPathIdle->setChecked(false);
    }
    else
    {
      ui_.radioPathRecording->setChecked(false);
      ui_.radioPathIdle->setChecked(true);
    }
  }
  else
  {
    ui_.radioPathRecording->setEnabled(false);
    ui_.radioPathIdle->setEnabled(false);
  }
}

void StateMachineController::checkSlamMapScale()
{
  state_machine_msgs::ActionControl srv;
  srv.request.action.module = 0;
  srv.request.action.command = 14;

  if (!ros::service::call("slam_state_control", srv))
    ui_.mapScale->setText("");
}

void StateMachineController::checkDeeppsStartCondition()
{
  if (navi_status_ == StateMachineStatus::Navigation::RUNNING &&
      deepps_status_ == StateMachineStatus::Deepps::IDLE)
  { // check if deepps start position has been reached

    // get transform of footprint to map
    tf::StampedTransform tf_footprint2map_stamp;
    try
    {
      tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0),
                                   tf_footprint2map_stamp);

      // get current pose
      double x = tf_footprint2map_stamp.getOrigin().getX();
      double y = tf_footprint2map_stamp.getOrigin().getY();

      // tolerance
      double tol = 2.0;

      // current distance
      double dist =
          std::hypot(x - deepps_start_pos_.x(), y - deepps_start_pos_.y());

      ROS_DEBUG("Distance to deepps start pos: %f", dist);
      ui_.deeppsStartPosDist->setText("Dist: " + QString::number(dist, 'f', 2));

      if (dist < tol)
      {
        ui_.status->setText("Status: Deepps start position arrived!");
        // start deepps
        onDeeppsStart();
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN_STREAM("checkDeeppsStartCondition: " << ex.what());
      ui_.deeppsStartPosDist->setText("Dist: N/A");
    }
  }
  else
    ui_.deeppsStartPosDist->setText("Dist: N/A");
}

void StateMachineController::getLaunchFilePathBringup()
{
  std::string package_path = ros::package::getPath(PACKAGE_BRINGUP);
  package_path += "/launch";

  QString file = QFileDialog::getOpenFileName(
      widget_, tr("Open Bringup launch file"),
      QString::fromStdString(package_path), tr("ROS Launch Files (*.launch)"));
  ui_.launchFilePathBringup->setText(file);
  return;
}

void StateMachineController::getLaunchFilePathSlam()
{
  std::string package_path = ros::package::getPath(PACKAGE_SLAM);
  package_path += "/launch";

  QString file = QFileDialog::getOpenFileName(
      widget_, tr("Open Slam launch file"),
      QString::fromStdString(package_path), tr("ROS Launch Files (*.launch)"));
  ui_.launchFilePathSlam->setText(file);
  return;
}

void StateMachineController::getLaunchFilePathFreespace()
{
  std::string package_path = ros::package::getPath(PACKAGE_FREESPACE);
  package_path += "/launch";

  QString file = QFileDialog::getOpenFileName(
      widget_, tr("Open Freespace launch file"),
      QString::fromStdString(package_path), tr("ROS Launch Files (*.launch)"));
  ui_.launchFilePathFreespace->setText(file);
  return;
}

void StateMachineController::getLaunchFilePathNavigation()
{
  std::string package_path = ros::package::getPath(PACKAGE_NAVIGATION);
  package_path += "/launch";

  QString file = QFileDialog::getOpenFileName(
      widget_, tr("Open Navigation launch file"),
      QString::fromStdString(package_path), tr("ROS Launch Files (*.launch)"));
  ui_.launchFilePathNavigation->setText(file);
  return;
}

void StateMachineController::getLaunchFilePathDeepps()
{
  std::string package_path = ros::package::getPath(PACKAGE_DEEPPS);
  package_path += "/launch";

  QString file = QFileDialog::getOpenFileName(
      widget_, tr("Open Deepps launch file"),
      QString::fromStdString(package_path), tr("ROS Launch Files (*.launch)"));
  ui_.launchFilePathDeepps->setText(file);
  return;
}

void StateMachineController::launchBringup()
{
  QString path = ui_.launchFilePathBringup->text();
  if (!path.isEmpty())
  {
    QString cmd = "gnome-terminal -x sh -c 'roslaunch ";
    cmd += path + "'";
    int output = system(cmd.toLocal8Bit().data());
    ui_.status->setText("Status: Bringup launched!");
  }
  return;
}

void StateMachineController::launchSlam()
{
  QString path = ui_.launchFilePathSlam->text();
  if (!path.isEmpty())
  {
    QString cmd = "gnome-terminal -x sh -c 'roslaunch ";
    cmd += path + "'";
    int output = system(cmd.toLocal8Bit().data());
    ui_.status->setText("Status: Slam launched!");
  }
  return;
}

void StateMachineController::launchFreespace()
{
  QString path = ui_.launchFilePathFreespace->text();
  if (!path.isEmpty())
  {
    QString cmd = "gnome-terminal -x sh -c 'roslaunch ";
    cmd += path + "'";
    int output = system(cmd.toLocal8Bit().data());
    ui_.status->setText("Status: Freespace launched!");
  }
  return;
}

void StateMachineController::launchNavigation()
{
  QString path = ui_.launchFilePathNavigation->text();
  if (!path.isEmpty())
  {
    QString cmd = "gnome-terminal -x sh -c 'roslaunch ";
    cmd += path + "'";
    int output = system(cmd.toLocal8Bit().data());
    ui_.status->setText("Status: Navigation launched!");
  }
  return;
}

void StateMachineController::launchDeepps()
{
  QString path = ui_.launchFilePathDeepps->text();
  if (!path.isEmpty())
  {
    QString cmd = "gnome-terminal -x sh -c 'roslaunch ";
    cmd += path + "'";
    int output = system(cmd.toLocal8Bit().data());
    ui_.status->setText("Status: Deepps launched!");
  }
  return;
}

void StateMachineController::launchRviz()
{
  int output = system("rviz&");
  ui_.status->setText("Status: RViz launched!");
}

void StateMachineController::configLcmCAN()
{
  int output = system("cd ~; ./lcm.sh");
  ui_.status->setText("Status: LCM & CAN configed!");
}

void StateMachineController::launchCAN()
{
  int output =
      system("gnome-terminal -x sh -c 'cd ~/HSS/PlanP2P/bin; ./PlanP2P'");
  ui_.status->setText("Status: CAN launched!");
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
PLUGINLIB_DECLARE_CLASS(rqt_state_machine, StateMachineController,
                        rqt_state_machine::StateMachineController,
                        rqt_gui_cpp::Plugin)
