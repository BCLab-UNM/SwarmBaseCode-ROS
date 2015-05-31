#include <rover_motor.h>


// TODO: move these globals to a shared header file
#define JOY_TOTAL_AXES          6
#define JOY_TOTAL_BUTTONS       16

#define JOY_AXIS_LEFT_THUMB_X	0
#define JOY_AXIS_LEFT_THUMB_Y	1
#define JOY_AXIS_RIGHT_THUMB_X	2
#define JOY_AXIS_RIGHT_THUMB_Y	3
#define JOY_AXIS_LEFT_FINGER	4
#define JOY_AXIS_RIGHT_FINGER	5

#define JOY_BTN_LEFT_PAD_RIGHT	0
#define JOY_BTN_LEFT_PAD_LEFT	1
#define JOY_BTN_LEFT_PAD_UP     2
#define JOY_BTN_LEFT_PAD_DOWN	3
#define JOY_BTN_RIGHT_PAD_X     4
#define JOY_BTN_RIGHT_PAD_Y     5
#define JOY_BTN_RIGHT_PAD_A     6
#define JOY_BTN_RIGHT_PAD_B     7
#define JOY_BTN_LEFT_FINGER     8
#define JOY_BTN_RIGHT_FINGER	9
#define JOY_BTN_LEFT_THROTTLE	10
#define JOY_BTN_RIGHT_THROTTLE	11
#define JOY_BTN_BACK            12
#define JOY_BTN_START           13
#define JOY_BTN_LEFT_THUMB      14
#define JOY_BTN_RIGHT_THUMB     15

#define STEER_MODE_TANK		0
#define STEER_MODE_JOYSTICK	1

namespace rover_driver_rqt_motor {

long heartBeat = 0;
QString name = "";
int prevRobotTime = 0;
int currRobotTime = 0;

RoverMotor::RoverMotor() : rqt_gui_cpp::Plugin(), widget(0){
    setObjectName("RobotTeleop");
}

void RoverMotor::initPlugin(qt_gui_cpp::PluginContext& context){
    widget = new QWidget();
    ui.setupUi(widget);

    connect(ui.forwardButton, SIGNAL(pressed()), this, SLOT(moveForward()));
    connect(ui.backButton, SIGNAL(pressed()), this, SLOT(moveBackward()));
    connect(ui.rightButton, SIGNAL(pressed()), this, SLOT(turnRight()));
    connect(ui.leftButton, SIGNAL(pressed()), this, SLOT(turnLeft()));
    connect(ui.stopButton, SIGNAL(pressed()), this, SLOT(allStop()));
    connect(ui.publishButton, SIGNAL(pressed()), this, SLOT(publishName()));

    if (context.serialNumber() > 1){
        widget->setWindowTitle(widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }

    QStringList robots;
    robots << "moe" << "larry" << "curly" << "shemp" << "harve" << "driverStation" << "driver";
    ui.robotList->addItems(robots);

    context.addWidget(widget);
    
    timer = new QTimer(this);
    timer->start(1000);
}

void RoverMotor::shutdownPlugin(){
    whichPublish.shutdown();
    hbPublish.shutdown();
    stickPublish.shutdown();
    steerPublish.shutdown();
}

void RoverMotor::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const{
}

void RoverMotor::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings){
}

void RoverMotor::moveForward(){
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(JOY_TOTAL_AXES);
    joy_msg.buttons.resize(JOY_TOTAL_BUTTONS);
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_X] = 0.0;
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_Y] = (double) ui.speedSlider->value() / 100.0;
    stickPublish.publish(joy_msg);

    ros::spinOnce();
}

void RoverMotor::moveBackward(){
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(JOY_TOTAL_AXES);
    joy_msg.buttons.resize(JOY_TOTAL_BUTTONS);
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_X] = 0.0;
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_Y] = -1 * (double) ui.speedSlider->value() / 100.0;
    stickPublish.publish(joy_msg);

    ros::spinOnce();
}

void RoverMotor::turnRight(){    
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(JOY_TOTAL_AXES);
    joy_msg.buttons.resize(JOY_TOTAL_BUTTONS);
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_X] = -1 * (double) ui.turnSlider->value() / 100.0;
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_Y] = 0.0;
    stickPublish.publish(joy_msg);

    ros::spinOnce();
}

void RoverMotor::turnLeft(){    
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(JOY_TOTAL_AXES);
    joy_msg.buttons.resize(JOY_TOTAL_BUTTONS);
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_X] = (double) ui.turnSlider->value() / 100.0;
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_Y] = 0.0;
    stickPublish.publish(joy_msg);

    ros::spinOnce();
}

void RoverMotor::allStop(){
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(JOY_TOTAL_AXES);
    joy_msg.buttons.resize(JOY_TOTAL_BUTTONS);
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_X] = 0.0;
    joy_msg.axes[JOY_AXIS_LEFT_THUMB_Y] = 0.0;
    stickPublish.publish(joy_msg);

    ros::spinOnce();
}

void RoverMotor::statusUpdate(const std_msgs::String::ConstPtr& msg){
    QString rstatus(msg->data.c_str());
    if(rstatus != "OFFLINE"){
        ui.robotStatus->setText(rstatus);
    }
}

void RoverMotor::haltUpdate(const std_msgs::Bool::ConstPtr& msg){
    bool rhaltflag = msg->data;

    if(rhaltflag) {
        ROS_INFO("GUI: halt flag is TRUE, must turn it off");
        sensor_msgs::Joy joy_msg;
        joy_msg.axes.resize(JOY_TOTAL_AXES);
        joy_msg.buttons.resize(JOY_TOTAL_BUTTONS);

        joy_msg.buttons[JOY_BTN_START] = 1;
        stickPublish.publish(joy_msg);
        ros::spinOnce();
        joy_msg.buttons[JOY_BTN_START] = 0;
        stickPublish.publish(joy_msg);
        ros::spinOnce();

    } else {
        ROS_INFO("GUI: halt flag is FALSE");
    }
}

void RoverMotor::hbUpdate(){
    std_msgs::UInt64 hb;
    hb.data = heartBeat;
    hbPublish.publish(hb);
    ros::spinOnce();
    heartBeat ++;
    prevRobotTime ++;
    if((prevRobotTime-currRobotTime) > 4){
        ui.robotStatus->setText("LOST LINK");
    }
}

void RoverMotor::publishName(){
    QString name = ui.robotList->currentText();
    QString hb = name + "/teleop/heartbeat";
    QString stick = name + "/joystick";
    QString which = name + "/joystick_whom";
    QString steer = name + "/steering";
    hbPublish = teleopNode.advertise<std_msgs::UInt64>(hb.toStdString(), 10, this);
    stickPublish = teleopNode.advertise<sensor_msgs::Joy>(stick.toStdString(), 10, this);
    whichPublish = teleopNode.advertise<std_msgs::String>(which.toStdString(), 10, this);
    steerPublish = teleopNode.advertise<std_msgs::Int16>(steer.toStdString(), 10, this);

    // publish name of robot the joystick is controlling (physical joystick gets to control last plugin running)
    std_msgs::String whom_msg;
    whom_msg.data = name.toStdString();
    whichPublish.publish(whom_msg);
    ros::spinOnce();

    // publish joystick steering mode
    std_msgs::Int16 steer_msg;
    steer_msg.data = STEER_MODE_JOYSTICK;
    steerPublish.publish(steer_msg);
    ros::spinOnce();

    QString sta = name + "Status";
    QString halt = name + "/gui/halt_motion";
    statusSubscribe = teleopNode.subscribe(sta.toStdString(), 10, &RoverMotor::statusUpdate, this);
    haltSubscribe = teleopNode.subscribe(halt.toStdString(), 10, &RoverMotor::haltUpdate, this);

    connect(timer, SIGNAL(timeout()), this, SLOT(hbUpdate()));
    ui.forwardButton->setEnabled(true);
    ui.leftButton->setEnabled(true);
    ui.rightButton->setEnabled(true);
    ui.backButton->setEnabled(true);
    ui.stopButton->setEnabled(true);
    ui.speedSlider->setEnabled(true);
    ui.turnSlider->setEnabled(true);
    ui.robotList->setEnabled(false);
}

}

PLUGINLIB_EXPORT_CLASS(rover_driver_rqt_motor::RoverMotor, rqt_gui_cpp::Plugin)
