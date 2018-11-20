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
  ui_.tabWidget->setCurrentWidget(ui_.tabSlam);
  // add widget to the user interface
  context.addWidget(widget_);

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

  connect(ui_.startFreespace, SIGNAL(clicked()), this,
          SLOT(onFreespaceStart()));
  connect(ui_.stopFreespace, SIGNAL(clicked()), this, SLOT(onFreespaceStop()));

  connect(ui_.enableVehicleControlManually, SIGNAL(clicked()), this,
          SLOT(onVehicleControlEnable()));
  connect(ui_.disableVehicleControlManually, SIGNAL(clicked()), this,
          SLOT(onVehicleControlDisable()));
  connect(ui_.eStopVehicleControlManually, SIGNAL(clicked()), this,
          SLOT(onVehicleControlEStop()));

  connect(ui_.startFollowing, SIGNAL(clicked()), this,
          SLOT(onVehicleControlEnable()));

  connect(ui_.startDeeppsManually, SIGNAL(clicked()), this,
          SLOT(onDeeppsStart()));
  connect(ui_.stopDeeppsManually, SIGNAL(clicked()), this,
          SLOT(onDeeppsStop()));

  connect(ui_.startDeepps, SIGNAL(clicked()), this, SLOT(onDeeppsStart()));

  connect(ui_.startRecordPathInLocalization, SIGNAL(clicked()), this,
          SLOT(onSlamRecordPathInLocalizationStart()));
  connect(ui_.stopRecordPathInLocalization, SIGNAL(clicked()), this,
          SLOT(onSlamRecordPathInLocalizationStop()));

  connect(ui_.startAuto, SIGNAL(clicked()), this, SLOT(startStateMachine()));
  connect(ui_.stopAuto, SIGNAL(clicked()), this, SLOT(stopStateMachine()));

  // subscribe to ros topics
  parkinglot_status_sub_ =
      nh_.subscribe("/deepps/parkinglot_status", 100,
                    &StateMachineController::parkinglotStatusCB, this);
  parkinglot_ctrl_sub_ =
      nh_.subscribe("/deepps/parkinglot_ctrl", 100,
                    &StateMachineController::parkinglotCtrlCB, this);

  // Advertising state control service
  state_feedback_service_ = nh_.advertiseService(
      "state_machine_feedback",
      &StateMachineController::updateStateMachineStates, this);

  // initialize status of different modules
  initStateMachineStatus();
}

void StateMachineController::shutdownPlugin()
{
  // TODO unregister all publishers here
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

// start running state machine
void StateMachineController::startStateMachine()
{
  ui_.status->setText("Status: State machine started!");

  // start slam
  onSlamStart();
}

// stop running state machine
void StateMachineController::stopStateMachine()
{
  // stop vehicle control
  onVehicleControlDisable();

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
    StateFeedback::Request& req, StateFeedback::Response& res)
{
  switch (req.state.module)
  {
  case 0: // slam
  {
    ui_.status->setText("Status: slam feedback received");
    switch (req.state.state)
    {
    case 0: // tracking status
    {
      int8_t traking_status = req.state.data;
      if (traking_status == 0)
        ui_.statusSlam->setText("Tracking succeed");
      else if (traking_status == 1)
        ui_.statusSlam->setText("Tracking failed");
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
    QMessageBox::warning(widget_, "record deepps start position",
                         "Failed to call record deepps start position service!");

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
                         "Failed to call start freespace service!");
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
