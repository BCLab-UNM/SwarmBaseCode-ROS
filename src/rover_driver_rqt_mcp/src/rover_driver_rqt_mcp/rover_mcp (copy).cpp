#include <rover_driver_rqt_mcp/rover_mcp.h>


namespace rover_driver_rqt_mcp {

RoverMcp::RoverMcp() : rqt_gui_cpp::Plugin(), widget(0){
    setObjectName("RobotMCP");
}

void RoverMcp::initPlugin(qt_gui_cpp::PluginContext& context){
    widget = new QWidget();
    ui.setupUi(widget);

    connect(ui.manualButton, SIGNAL(pressed()), this, SLOT(manualMode()));
    connect(ui.autoButton, SIGNAL(pressed()), this, SLOT(autoMode()));
    connect(ui.manualSimButton, SIGNAL(pressed()), this, SLOT(manualSimMode()));
    connect(ui.autoSimButton, SIGNAL(pressed()), this, SLOT(autoSimMode()));
    connect(ui.publishButton, SIGNAL(pressed()), this, SLOT(publishName()));
    connect(ui.parameterButton, SIGNAL(pressed()), this, SLOT(publishParameters()));
    connect(ui.speedSlider, SIGNAL(sliderReleased()), this, SLOT(publishVel()));
    connect(ui.turnSlider, SIGNAL(sliderReleased()), this, SLOT(publishVel()));

    if (context.serialNumber() > 1){
        widget->setWindowTitle(widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }

    QStringList robots;
    robots << "moe" << "larry" << "curly" << "shemp" << "harve" << "driverStation" << "driver";
    ui.robotList->addItems(robots);

    context.addWidget(widget);

    //possible tag counter

}

void RoverMcp::shutdownPlugin(){
    modePublish.shutdown();
    parameterPublish.shutdown();
}

void RoverMcp::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const{
}

void RoverMcp::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings){
}

void RoverMcp::manualMode(){
    operationMode.data = 1;
    modePublish.publish(operationMode);
    //Automatically slide the speed slider to the iAnt speed for the physical robots.
    ui.speedSlider->setSliderPosition(18.0);
    linearSpd.data = (float) ui.speedSlider->value() / 100.0;
    linearPublish.publish(linearSpd);
    ros::spinOnce();
}

void RoverMcp::autoMode(){
    operationMode.data = 3;
    modePublish.publish(operationMode);
    //Automatically slide the speed slider to the iAnt speed for the physical robots.
    ui.speedSlider->setSliderPosition(18.0);
    linearSpd.data = (float) ui.speedSlider->value() / 100.0;
    linearPublish.publish(linearSpd);
    ros::spinOnce();
}

void RoverMcp::manualSimMode(){
    operationMode.data = 0;
    modePublish.publish(operationMode);
    //Automatically slide the speed slider to the iAnt speed for the Gazebo simulation.
    ui.speedSlider->setSliderPosition(35.0);
    linearSpd.data = (float) ui.speedSlider->value() / 100.0;
    linearPublish.publish(linearSpd);
    ros::spinOnce();
}

void RoverMcp::autoSimMode(){
    operationMode.data = 2;
    modePublish.publish(operationMode);
    //Automatically slide the speed slider to the iAnt speed for the Gazebo simulation.
    ui.speedSlider->setSliderPosition(35.0);
    linearSpd.data = (float) ui.speedSlider->value() / 100.0;
    linearPublish.publish(linearSpd);
    ros::spinOnce();
}

void RoverMcp::publishVel(){
    linearSpd.data = (float) ui.speedSlider->value() / 100.0;
    angularSpd.data = (float) ui.turnSlider->value() / 100.0;
    linearPublish.publish(linearSpd);
    angularPublish.publish(angularSpd);
}

void RoverMcp::publishName(){
    QString name = ui.robotList->currentText();
    QString mode = name + "/mode";
    modePublish = mcpNode.advertise<std_msgs::UInt8>(mode.toStdString(), 10, this);
    QString parameters = name +"/parameters";
    parameterPublish = mcpNode.advertise<std_msgs::Float32MultiArray>(parameters.toStdString(), 10, this);
    QString linear = name + "/linearVel";
    QString angular = name + "/angularVel";
    linearPublish = mcpNode.advertise<std_msgs::Float32>(linear.toStdString(), 10, this);
    angularPublish = mcpNode.advertise<std_msgs::Float32>(angular.toStdString(), 10, this);

    operationMode.data = 0;
    modePublish.publish(operationMode);

    linearSpd.data = (float) ui.speedSlider->value() / 100.0;
    angularSpd.data = (float) ui.turnSlider->value() / 100.0;
    linearPublish.publish(linearSpd);
    angularPublish.publish(angularSpd);

    ros::spinOnce();

    ui.manualButton->setEnabled(true);
    ui.autoButton->setEnabled(true);
    ui.manualSimButton->setEnabled(true);
    ui.autoSimButton->setEnabled(true);
    ui.parameterButton->setEnabled(true);
    ui.pheromoneData->setEnabled(true);
    ui.uninformedData->setEnabled(true);
    ui.informedData->setEnabled(true);
    ui.travelData->setEnabled(true);
    ui.searchData->setEnabled(true);
    ui.layingData->setEnabled(true);
    ui.leaveData->setEnabled(true);
    ui.chargeData->setEnabled(true);
    ui.siteData->setEnabled(true);
    ui.followingData->setEnabled(true);
    ui.robotList->setEnabled(false);
    ui.speedSlider->setEnabled(true);
    ui.turnSlider->setEnabled(true);
}

void RoverMcp::publishParameters(){

    QString tPhero = ui.pheromoneData->text();
    parameterArray.data.push_back(tPhero.toFloat());
    QString tUninf = ui.uninformedData->text();
    parameterArray.data.push_back(tUninf.toFloat());
    QString tInfor = ui.informedData->text();
    parameterArray.data.push_back(tInfor.toFloat());
    QString tTrave = ui.travelData->text();
    parameterArray.data.push_back(tTrave.toFloat());
    QString tSearc = ui.searchData->text();
    parameterArray.data.push_back(tSearc.toFloat());
    QString tLayin = ui.layingData->text();
    parameterArray.data.push_back(tLayin.toFloat());
    QString tLeave = ui.leaveData->text();
    parameterArray.data.push_back(tLeave.toFloat());
    QString tCharg = ui.chargeData->text();
    parameterArray.data.push_back(tCharg.toFloat());
    QString tSite = ui.siteData->text();
    parameterArray.data.push_back(tSite.toFloat());
    QString tFoll = ui.followingData->text();
    parameterArray.data.push_back(tFoll.toFloat());

    parameterPublish.publish(parameterArray);    

    ros::spinOnce();

    parameterArray.data.clear();

}


}

PLUGINLIB_EXPORT_CLASS(rover_driver_rqt_mcp::RoverMcp, rqt_gui_cpp::Plugin)
