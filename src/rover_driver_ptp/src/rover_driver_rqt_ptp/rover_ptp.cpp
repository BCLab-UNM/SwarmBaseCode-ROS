#include <rover_driver_rqt_ptp/rover_ptp.h>


namespace rover_driver_rqt_ptp {

RoverPtp::RoverPtp() : rqt_gui_cpp::Plugin(), widget(0){
    setObjectName("RobotPTP");
}

void RoverPtp::initPlugin(qt_gui_cpp::PluginContext& context){
    widget = new QWidget();
    ui.setupUi(widget);

//    connect(ui.manualButton, SIGNAL(pressed()), this, SLOT(manualMode()));
//    connect(ui.autoButton, SIGNAL(pressed()), this, SLOT(autoMode()));
//    connect(ui.manualSimButton, SIGNAL(pressed()), this, SLOT(manualSimMode()));
//    connect(ui.autoSimButton, SIGNAL(pressed()), this, SLOT(autoSimMode()));
    connect(ui.publishButton, SIGNAL(pressed()), this, SLOT(publishName()));
    connect(ui.sendButton, SIGNAL(pressed()), this, SLOT(publishPoint()));

    if (context.serialNumber() > 1){
        widget->setWindowTitle(widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }

    QStringList robots;
    robots << "moe" << "larry" << "curly" << "shemp" << "harve" << "driverStation" << "driver";
    ui.robotList->addItems(robots);

    context.addWidget(widget);
}

void RoverPtp::shutdownPlugin(){
    ptpPublish.shutdown();
    //parameterPublish.shutdown();
}

void RoverPtp::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const{
}

void RoverPtp::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings){
}

//void RoverPtp::manualMode(){
//    operationMode.data = 1;
//    modePublish.publish(operationMode);
//    ros::spinOnce();
//}

//void RoverPtp::autoMode(){
//    operationMode.data = 3;
//    modePublish.publish(operationMode);
//    ros::spinOnce();
//}

//void RoverPtp::manualSimMode(){
//    operationMode.data = 0;
//    modePublish.publish(operationMode);
//    ros::spinOnce();
//}

//void RoverPtp::autoSimMode(){
//    operationMode.data =2;
//    modePublish.publish(operationMode);
//    ros::spinOnce();
//}


void RoverPtp::publishName(){
    QString name = ui.robotList->currentText();
    QString ptp = name + "/ptp";
    ptpPublish = ptpNode.advertise<geometry_msgs::Point>(ptp.toStdString(), 10, this);
    QString mode = name + "/mode";
    modePublish = ptpNode.advertise<std_msgs::UInt8>(mode.toStdString(), 10, this);

    operationMode.data = 4;
    modePublish.publish(operationMode);

    navPoint.x = 0.0;
    navPoint.y = 0.0;
    navPoint.z = 0.0;
    ptpPublish.publish(navPoint);
    ros::spinOnce();

//    ui.manualButton->setEnabled(true);
//    ui.autoButton->setEnabled(true);
//    ui.manualSimButton->setEnabled(true);
//    ui.autoSimButton->setEnabled(true);
//    ui.parameterButton->setEnabled(true);
//    ui.pheromoneData->setEnabled(true);
//    ui.uninformedData->setEnabled(true);
//    ui.informedData->setEnabled(true);
    ui.xCoordinateData->setEnabled(true);
    ui.yCoordinateData->setEnabled(true);
    ui.sendButton->setEnabled(true);
    ui.repeatCheckBox->setEnabled(true);
//    ui.searchData->setEnabled(true);
//    ui.layingData->setEnabled(true);
//    ui.leaveData->setEnabled(true);
//    ui.chargeData->setEnabled(true);
//    ui.siteData->setEnabled(true);
//    ui.robotList->setEnabled(false);
}

void RoverPtp::publishPoint(){

    QString tX = ui.xCoordinateData->text();
    navPoint.x = (tX.toFloat());
    QString tY = ui.yCoordinateData->text();
    navPoint.y = (tY.toFloat());

//    QString tTrave = ui.travelData->text();
//    parameterArray.data.push_back(tTrave.toFloat());
//    QString tSearc = ui.searchData->text();
//    parameterArray.data.push_back(tSearc.toFloat());
//    QString tLayin = ui.layingData->text();
//    parameterArray.data.push_back(tLayin.toFloat());
//    QString tLeave = ui.leaveData->text();
//    parameterArray.data.push_back(tLeave.toFloat());
//    QString tCharg = ui.chargeData->text();
//    parameterArray.data.push_back(tCharg.toFloat());
//    QString tSite = ui.siteData->text();
//    parameterArray.data.push_back(tSite.toFloat());

ptpPublish.publish(navPoint);
ros::spinOnce();

}

}

PLUGINLIB_EXPORT_CLASS(rover_driver_rqt_ptp::RoverPtp, rqt_gui_cpp::Plugin)
