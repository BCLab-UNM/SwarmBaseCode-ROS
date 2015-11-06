// Author: Matthew Fricke
// E-mail: matthew@fricke.co.uk
// Date: 9-16-205
// Purpose: implementation of a simple graphical front end for the UNM-NASA Swarmathon rovers.
// License: GPL3

#include <rover_gui_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <QtXml>
#include <QFile>
#include <QListWidget>
#include <QScrollBar>
#include <QProcess>
#include <QPalette>
#include <QTabBar>
#include <QTabWidget>
#include <QCheckBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QMessageBox>
#include <QProgressDialog>
#include <QStringList>
#include <QLCDNumber>
#include <QComboBox>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

//#include <regex> // For regex expressions

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

using namespace std;

namespace rqt_rover_gui 
{
  RoverGUIPlugin::RoverGUIPlugin() : rqt_gui_cpp::Plugin(), widget(0)
  {
    setObjectName("RoverGUI");
    log_messages = "";
    all_autonomous = false;
    joy_process = NULL;

    arena_height = 20;
    arena_width = 20;

  }

  void RoverGUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    cout << "Rover GUI Starting..." << endl;

    QStringList argv = context.argv();

    widget = new QWidget();

    ui.setupUi(widget);
    
    context.addWidget(widget);

    ui.version_number_label->setText("<font color='white'>0.04 Alpha</font>");

    widget->setWindowTitle("Rover Interface");

    string rover_name_msg = "<font color='white'>Rover: " + selected_rover_name + "</font>";
    QString rover_name_msg_qstr = QString::fromStdString(rover_name_msg);
    ui.rover_name->setText(rover_name_msg_qstr);

    // Setup QT message connections
    connect(ui.rover_list, SIGNAL(currentItemChanged(QListWidgetItem*,QListWidgetItem*)), this, SLOT(currentRoverChangedEventHandler(QListWidgetItem*,QListWidgetItem*)));
    connect(ui.ekf_checkbox, SIGNAL(toggled(bool)), this, SLOT(EKFCheckboxToggledEventHandler(bool)));
    connect(ui.gps_checkbox, SIGNAL(toggled(bool)), this, SLOT(GPSCheckboxToggledEventHandler(bool)));
    connect(ui.encoder_checkbox, SIGNAL(toggled(bool)), this, SLOT(encoderCheckboxToggledEventHandler(bool)));
    connect(ui.autonomous_control_radio_button, SIGNAL(toggled(bool)), this, SLOT(autonomousRadioButtonEventHandler(bool)));
    connect(ui.all_autonomous_control_radio_button, SIGNAL(toggled(bool)), this, SLOT(allAutonomousRadioButtonEventHandler(bool)));
    connect(ui.joystick_control_radio_button, SIGNAL(toggled(bool)), this, SLOT(joystickRadioButtonEventHandler(bool)));
    connect(ui.build_simulation_button, SIGNAL(pressed()), this, SLOT(buildSimulationButtonEventHandler()));
    connect(ui.clear_simulation_button, SIGNAL(pressed()), this, SLOT(clearSimulationButtonEventHandler()));

    // Create a subscriber to listen for joystick events
    joystick_subscriber = nh.subscribe("/joy", 1000, &RoverGUIPlugin::joyEventHandler, this);

    displayLogMessage("Searching for rovers...");

    // Add discovered rovers to the GUI list
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(pollRoversTimerEventHandler()));
    timer->start(5000);

    // Setup the initial display parameters for the map
    ui.map_frame->setDisplayGPSData(ui.gps_checkbox->isChecked());
    ui.map_frame->setDisplayEncoderData(ui.encoder_checkbox->isChecked());
    ui.map_frame->setDisplayEKFData(ui.ekf_checkbox->isChecked());

    ui.joystick_frame->setHidden(false);

    ui.tab_widget->setCurrentIndex(0);

    ui.texture_combobox->setItemData(0, Qt::white, Qt::TextColorRole);

    //QString return_msg = startROSJoyNode();
    //displayLogMessage(return_msg);


  }

  void RoverGUIPlugin::shutdownPlugin()
  {
    sim_creator.stopGazebo();
    stopROSJoyNode();

    //ros::shutdown();
  }

void RoverGUIPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
}

void RoverGUIPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
}


void RoverGUIPlugin::joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    // Set the gui values
    if (joy_msg->axes[4] > 0)
       ui.joy_lcd_forward->display(joy_msg->axes[4]);
    if (joy_msg->axes[4] < 0)
        ui.joy_lcd_back->display(-joy_msg->axes[4]);

    if (joy_msg->axes[3] > 0)
      {
       ui.joy_lcd_left->display(joy_msg->axes[3]);
      }
    if (joy_msg->axes[3] < 0)
      {
        ui.joy_lcd_right->display(-joy_msg->axes[3]);
      }

// Magic axis values in the code below were taken the rover_driver_rqt_motor code /joystick output for default linear and angular velocities.
// Magic indicies are taken from rover_motor.cpp.
// This way the code is consistent with the existing GUI joystick.
// A better way would be to standardize a manual movement control interface and requre all input mechanisms to take input from the user
// and repackage te information according to the interface spec.
    sensor_msgs::Joy standardized_joy_msg;
    standardized_joy_msg.axes.resize(6);

    int x_axis = 0;
    int y_axis = 1;

    if (abs(joy_msg->axes[4]) > 0.05)
    {
      standardized_joy_msg.axes[y_axis] = joy_msg->axes[4];
    }

  if (abs(joy_msg->axes[3]) > 0.05)
    {
      standardized_joy_msg.axes[x_axis] = joy_msg->axes[3];
    }

  joystick_publisher.publish(standardized_joy_msg);

}

void RoverGUIPlugin::EKFEventHandler(const nav_msgs::Odometry::ConstPtr& msg)
{

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    QString x_str; x_str.setNum(x);
    QString y_str; y_str.setNum(y);

   ui.map_frame->addToEKFRoverPath(x,y);
}


void RoverGUIPlugin::encoderEventHandler(const nav_msgs::Odometry::ConstPtr& msg)
{

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    QString x_str; x_str.setNum(x);
    QString y_str; y_str.setNum(y);

   ui.map_frame->addToEncoderRoverPath(x,y);
}


void RoverGUIPlugin::GPSEventHandler(const nav_msgs::Odometry::ConstPtr& msg)
{

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    QString x_str; x_str.setNum(x);
    QString y_str; y_str.setNum(y);

   ui.map_frame->addToGPSRoverPath(x,y);
}

 void RoverGUIPlugin::cameraEventHandler(const sensor_msgs::ImageConstPtr& image)
 {
     cv_bridge::CvImagePtr cv_image_ptr;

     try
     {
        cv_image_ptr = cv_bridge::toCvCopy(image);
     }
     catch (cv_bridge::Exception &e)
     {
         ROS_ERROR("In rover_gui_plugin.cpp: cv_bridge exception: %s", e.what());
     }

     int image_cols = cv_image_ptr->image.cols;
     int image_rows = cv_image_ptr->image.rows;
     int image_step = cv_image_ptr->image.step;

     ostringstream cols_stream, rows_stream;
     cols_stream << image_cols;
     rows_stream << image_rows;

    // ROS_INFO_STREAM("Image received Size:" + rows_stream.str() + "x" + cols_stream.str());

     // Create QImage to hold the image
     //const uchar* image_buffer = (const uchar*)cv_image_ptr->image.data; // extract the raw data
     QImage qimg(&(image->data[0]), image_cols, image_rows, image_step, QImage::Format_RGB888);
     ui.camera_frame->setImage(qimg);
 }

set<string> RoverGUIPlugin::findConnectedRovers()
{
    set<string> rovers;

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    stringstream ss;

   for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
    {
        const ros::master::TopicInfo& info = *it;

        string rover_name;

        std::size_t found = info.name.find("/status");
          if (found!=std::string::npos)
          {
            rover_name = info.name.substr(1,found-1);

            found = rover_name.find("/"); // Eliminate potential names with / in them
            if (found==std::string::npos)
            {
                rovers.insert(rover_name);
            }
        }
    }

    return rovers;
}

void RoverGUIPlugin::currentRoverChangedEventHandler(QListWidgetItem *current, QListWidgetItem *previous)
{
    selected_rover_name = current->text().toStdString();
    string rover_name_msg = "<font color='white'>Rover: " + selected_rover_name + "</font>";
    QString rover_name_msg_qstr = QString::fromStdString(rover_name_msg);
    ui.rover_name->setText(rover_name_msg_qstr);

    setupSubscribers();
    setupPublishers();

    std::map<string, int>::iterator it = rover_control_state.find(selected_rover_name);

    // No entry for this rover name
    if (it == rover_control_state.end())
    {
        // Default to joystick
        ui.joystick_control_radio_button->setChecked(true);
        ui.autonomous_control_radio_button->setChecked(false);
        ui.all_autonomous_control_radio_button->setChecked(false);
        joystickRadioButtonEventHandler(true); // Manually trigger the joystick selected event
        rover_control_state[selected_rover_name]=1;
    }
    else
    {
    int control_state = it->second;

    switch (control_state)
    {
    case 1: ui.joystick_control_radio_button->setChecked(true);
        ui.autonomous_control_radio_button->setChecked(false);
        ui.all_autonomous_control_radio_button->setChecked(false);
        break;
    case 2: ui.joystick_control_radio_button->setChecked(false);
        ui.autonomous_control_radio_button->setChecked(true);
        ui.all_autonomous_control_radio_button->setChecked(false);
        break;
    case 3: ui.joystick_control_radio_button->setChecked(false);
        ui.autonomous_control_radio_button->setChecked(false);
        ui.all_autonomous_control_radio_button->setChecked(true);
        break;
    default:
        displayLogMessage("Unknown control state");
    }
}

    // Clear map
    ui.map_frame->clearMap();

    // Enable control mode radio group now that a rover has been selected
    ui.autonomous_control_radio_button->setEnabled(true);
    ui.joystick_control_radio_button->setEnabled(true);
    ui.all_autonomous_control_radio_button->setEnabled(true);

}

void RoverGUIPlugin::pollRoversTimerEventHandler()
{
    set<string>new_rover_names = findConnectedRovers();

    // Wait for a rover to connect
    if (new_rover_names.empty())
    {
        displayLogMessage("Waiting for rover to connect...");
        ui.map_frame->clearMap();
        selected_rover_name = "";
        // Disable control mode radio group since no rover has been selected
        ui.autonomous_control_radio_button->setEnabled(false);
        ui.joystick_control_radio_button->setEnabled(false);
        ui.all_autonomous_control_radio_button->setEnabled(false);
        return;
    }

    if (new_rover_names == rover_names)
    {
        return;
    }

    rover_names = new_rover_names;

   displayLogMessage("List of connected rovers has changed");

    ui.rover_list->clear();

    for(set<string>::const_iterator i = rover_names.begin(); i != rover_names.end(); ++i)
    {
        QListWidgetItem* new_item = new QListWidgetItem(QString::fromStdString(*i));
        new_item->setForeground(Qt::red);
        ui.rover_list->addItem(new_item);

    }
}

void RoverGUIPlugin::setupPublishers()
{
    // Set the robot to accept manual control. Latch so even if the robot connects later it will get the message.
    string control_mode_topic = "/"+selected_rover_name+"/mode";
    control_mode_publishers[selected_rover_name]=nh.advertise<std_msgs::UInt8>(control_mode_topic, 10, true); // last argument sets latch to true

    string joystick_topic = "/"+selected_rover_name+"/joystick";
    joystick_publisher = nh.advertise<sensor_msgs::Joy>(joystick_topic, 10, this);
}

void RoverGUIPlugin::setupSubscribers()
{
    // Create a subscriber to listen for camera events
    image_transport::ImageTransport it(nh);
    int frame_rate = 1;
    // Theroa codex results in the least information being transmitted
    camera_subscriber = it.subscribe("/"+selected_rover_name+"/camera/image", frame_rate, &RoverGUIPlugin::cameraEventHandler, this);//, image_transport::TransportHints("theora"));

    // Odometry and GPS subscribers
    encoder_subscriber = nh.subscribe("/"+selected_rover_name+"/odom/", 10, &RoverGUIPlugin::encoderEventHandler, this);
    ekf_subscriber = nh.subscribe("/"+selected_rover_name+"/odom/ekf", 10, &RoverGUIPlugin::EKFEventHandler, this);
    gps_subscriber = nh.subscribe("/"+selected_rover_name+"/odom/navsat", 10, &RoverGUIPlugin::GPSEventHandler, this);


    // Ultrasound Subscriptions

    us_center_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarCenter", 10, &RoverGUIPlugin::centerUSEventHandler, this);
    us_left_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarLeft", 10, &RoverGUIPlugin::leftUSEventHandler, this);
    us_right_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarRight", 10, &RoverGUIPlugin::rightUSEventHandler, this);


    // IMU Subscriptions
    imu_subscriber = nh.subscribe("/"+selected_rover_name+"/imu", 10, &RoverGUIPlugin::IMUEventHandler, this);

    // Target detected topic
   // target_detection_subscriber = nh.subscribe("/"+selected_rover_name+"/targets", 10, &RoverGUIPlugin::targetDetectedEventHandler, this);

}

void RoverGUIPlugin::centerUSEventHandler(const sensor_msgs::Range::ConstPtr& msg)
{
    ui.us_frame->setCenterRange(msg->range, msg->min_range, msg->max_range);
 }

void RoverGUIPlugin::rightUSEventHandler(const sensor_msgs::Range::ConstPtr& msg)
{
    ui.us_frame->setRightRange(msg->range, msg->min_range, msg->max_range);
}

void RoverGUIPlugin::leftUSEventHandler(const sensor_msgs::Range::ConstPtr& msg)
{
    ui.us_frame->setLeftRange(msg->range, msg->min_range, msg->max_range);
}

void RoverGUIPlugin::IMUEventHandler(const sensor_msgs::Imu::ConstPtr& msg)
{
    ui.imu_frame->setLinearAcceleration( msg->linear_acceleration.x,
                                         msg->linear_acceleration.y,
                                         msg->linear_acceleration.z );

    ui.imu_frame->setAngularVelocity(    msg->angular_velocity.x,
                                         msg->angular_velocity.y,
                                         msg->angular_velocity.z    );

    ui.imu_frame->setOrientation(        msg->orientation.w,
                                         msg->orientation.x,
                                         msg->orientation.y,
                                         msg->orientation.z        );

}

void RoverGUIPlugin::GPSCheckboxToggledEventHandler(bool checked)
{
    ui.map_frame->setDisplayGPSData(checked);
}

void RoverGUIPlugin::EKFCheckboxToggledEventHandler(bool checked)
{
    ui.map_frame->setDisplayEKFData(checked);
}

void RoverGUIPlugin::encoderCheckboxToggledEventHandler(bool checked)
{
    ui.map_frame->setDisplayEncoderData(checked);
}

// Currently broken. Calling displayLogMessage from the ROS event thread causes a crash or hang
//void RoverGUIPlugin::targetDetectedEventHandler(rover_onboard_target_detection::ATag tagInfo) //rover_onboard_target_detection::ATag msg )
//{
//    // Just let the user know the event happened
//   // displayLogMessage("Tag detected");

//}

void RoverGUIPlugin::displayLogMessage(QString msg)
{
    if (msg.isEmpty()) msg = "Message is empty";
    if (msg == NULL) msg = "Message was a NULL pointer";


    QString new_message = msg+"<br>";
    log_messages = log_messages+new_message;
    ui.log->setText("<font color='white'>"+log_messages+"</font>");

    QScrollBar *sb = ui.log->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void RoverGUIPlugin::autonomousRadioButtonEventHandler(bool marked)
{
    if (!marked) return;

    // Remember that this rover was set to single rover autonomous control
    set<string>::iterator it;
    // And for rovers with all autonomous as their state should now be just autonomous
     for (it = rover_names.begin(); it != rover_names.end(); it++)
     {
         if (rover_control_state[*it]==3) rover_control_state[*it]=2;
     }
    rover_control_state[selected_rover_name] = 2;

    ui.joystick_frame->setHidden(true);
    setupPublishers();

    std_msgs::UInt8 control_mode_msg;
    control_mode_msg.data = 2; // 2 indicates autonomous control
    control_mode_publishers[selected_rover_name].publish(control_mode_msg);
    QString return_msg = stopROSJoyNode();
    displayLogMessage(return_msg);


}

void RoverGUIPlugin::allAutonomousRadioButtonEventHandler(bool marked)
{
    if (!marked) return;

    all_autonomous = true;

    ui.joystick_frame->setHidden(true);

    string remember_selected_rover_name = selected_rover_name;

    set<string>::iterator it;
     for (it = rover_names.begin(); it != rover_names.end(); it++)
     {
         selected_rover_name = *it;
         rover_control_state[*it]=3;
         setupPublishers();
         std_msgs::UInt8 control_mode_msg;
         control_mode_msg.data = 2; // 2 indicates autonomous control
         control_mode_publishers[selected_rover_name].publish(control_mode_msg);

         displayLogMessage(QString::fromStdString(selected_rover_name)+" changed to autonomous control");
     }

     selected_rover_name = remember_selected_rover_name;

    QString return_msg = stopROSJoyNode();
    displayLogMessage(return_msg);
}

void RoverGUIPlugin::joystickRadioButtonEventHandler(bool marked)
{
    if (!marked) return;

    all_autonomous = false;

    // Remember that this rover was set to joystick control
    set<string>::iterator it;
    // And for rovers with all autonomous as their state should now be just autonomous
     for (it = rover_names.begin(); it != rover_names.end(); it++)
     {
         if (rover_control_state[*it]==3) rover_control_state[*it]=2;
     }
    rover_control_state[selected_rover_name] = 1;

    setupPublishers();
    ui.joystick_frame->setHidden(false);

    std_msgs::UInt8 control_mode_msg;
    control_mode_msg.data = 1; // 1 indicates manual control
    control_mode_publishers[selected_rover_name].publish(control_mode_msg);

    QString return_msg = startROSJoyNode();
    displayLogMessage(return_msg);

    displayLogMessage(QString::fromStdString(selected_rover_name)+" changed to joystick control");\
}

void RoverGUIPlugin::buildSimulationButtonEventHandler()
{
    displayLogMessage("Building simulation...");

    QString return_msg;

    return_msg = sim_creator.startGazebo();

    cout << return_msg.toStdString() << endl;
    displayLogMessage(return_msg);

    if (ui.texture_combobox->currentText() == "Gravel")
    {
    displayLogMessage("Adding gravel ground plane...");
    return_msg = sim_creator.addGroundPlane("mars_ground_plane");
    displayLogMessage(return_msg);
    }
    else if (ui.texture_combobox->currentText() == "Concrete")
    {
    displayLogMessage("Adding concrete ground plane...");
    return_msg = sim_creator.addGroundPlane("concrete_ground_plane");
    displayLogMessage(return_msg);
    }
    else if (ui.texture_combobox->currentText() == "Car park")
    {
    displayLogMessage("Adding carpark ground plane...");
    return_msg = sim_creator.addGroundPlane("carpark_ground_plane");
    displayLogMessage(return_msg);
    }
    else
    {
        displayLogMessage("Unknown ground plane...");
    }

    displayLogMessage("Adding rover alpha...");
    return_msg = sim_creator.addRover("alpha", -1, 0, 0);
    displayLogMessage(return_msg);

    displayLogMessage("Adding rover beta...");
    return_msg = sim_creator.addRover("beta", 0, 1, 0);
    displayLogMessage(return_msg);

    displayLogMessage("Adding rover gamma...");
    return_msg = sim_creator.addRover("gamma", 1, 0, 0);
    displayLogMessage(return_msg);


   displayLogMessage("Starting rover node for alpha...");
   return_msg = sim_creator.startRoverNode("alpha");
   displayLogMessage(return_msg);

   displayLogMessage("Starting rover node for beta...");
   return_msg = sim_creator.startRoverNode("beta");
   displayLogMessage(return_msg);

   displayLogMessage("Starting rover node for gamma...");
   return_msg = sim_creator.startRoverNode("gamma");
   displayLogMessage(return_msg);

   if (ui.final_radio_button->isChecked())
   {

       displayLogMessage("Adding rover epsilon...");
       return_msg = sim_creator.addRover("epsilon", 1, 1, 0);
       displayLogMessage(return_msg);

       displayLogMessage("Adding rover delta...");
       return_msg = sim_creator.addRover("delta", -1, -1, 0);
       displayLogMessage(return_msg);

       displayLogMessage("Adding rover zeta...");
       return_msg = sim_creator.addRover("zeta", 1, -1, 0);
       displayLogMessage(return_msg);


   displayLogMessage("Starting rover node for delta...");
   return_msg = sim_creator.startRoverNode("delta");
   displayLogMessage(return_msg);

   displayLogMessage("Starting rover node for episilon...");
   return_msg = sim_creator.startRoverNode("epsilon");
   displayLogMessage(return_msg);

   displayLogMessage("Starting rover node for zeta...");
   return_msg = sim_creator.startRoverNode("zeta");
   displayLogMessage(return_msg);
}
   if (ui.powerlaw_distribution_radio_button->isChecked())
   {
       displayLogMessage("Adding powerlaw distribution of targets...");
       return_msg = addPowerLawTargets();
       displayLogMessage(return_msg);
   }
   else if (ui.uniform_distribution_radio_button->isChecked())
   {
       displayLogMessage("Adding uniform distribution of targets...");
       return_msg = addUniformTargets();
       displayLogMessage(return_msg);
   }
   else if (ui.clustered_distribution_radio_button->isChecked())
   {
       displayLogMessage("Adding clustered distribution of targets...");
       return_msg = addClusteredTargets();
       displayLogMessage(return_msg);
   }

//   // Test rover movement
//   displayLogMessage("Moving alpha");
//   return_msg = sim_creator.moveRover("alpha", 10, 0, 0);
//   displayLogMessage(return_msg);
}

void RoverGUIPlugin::clearSimulationButtonEventHandler()
{
    displayLogMessage("Clearing simulation...");

    QString return_msg;
    return_msg = sim_creator.stopGazebo();
    displayLogMessage(return_msg);

    ui.rover_list->clear();
    rover_names.clear();
    rover_control_state.clear();
}

QString RoverGUIPlugin::startROSJoyNode()
{
    if (!joy_process)
    {

        QString argument = "rosrun joy joy_node";

        joy_process = new QProcess();

        joy_process->start("sh", QStringList() << "-c" << argument);

       // joy_process->waitForStarted();

        return "Started the joystick node.";

    }
    else
    {
        return "The joystick node is already running.";
    }
}

QString RoverGUIPlugin::stopROSJoyNode()
{
   //return "Do nothing for debug";

    if (joy_process)
    {
        joy_process->terminate();
        joy_process->waitForFinished();
        delete joy_process;
        joy_process = NULL;

        return "Stopped the running joystick node.";

    }
    else
    {
        return "Tried to stop the joystick node but it isn't running.";
    }
}

QString RoverGUIPlugin::addUniformTargets()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing 256 Targets");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    QString output;
    float clearance = 0; //meters
    float proposed_x;
    float proposed_y;

    // 256 piles of 1 tag
    for (int i = 0; i < 256; i++)
    {
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            displayLogMessage("Tried to place target "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = arena_width/2.0 - ((float) rand()) / RAND_MAX*arena_width;
            proposed_y = arena_height/2.0 - ((float) rand()) / RAND_MAX*arena_height;
       }
        while (sim_creator.isLocationOccupied(proposed_x, proposed_y, clearance));

        output = sim_creator.addModel(QString("at")+QString::number(i), proposed_x, proposed_y, 0);

       progress_dialog.setValue(i*100.0f/256);
    }
    displayLogMessage("Placed 256 single targets");

    return output;
}

QString RoverGUIPlugin::addClusteredTargets()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing 256 Targets into 4 Clusters (64 targets each)");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    QString output;
    float clearance = 0.5; //meters
    float proposed_x;
    float proposed_y;

    // Four piles of 64
    for (int i = 0; i < 4; i++)
    {
        // Keep GUI responsive
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

        do
        {
            displayLogMessage("Tried to place cluster "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = arena_width/2.0 - ((float) rand()) / RAND_MAX*arena_width;
            proposed_y = arena_height/2.0 - ((float) rand()) / RAND_MAX*arena_height;
        }
        while (sim_creator.isLocationOccupied(proposed_x, proposed_y, clearance));

        progress_dialog.setValue(i*100.0f/4);

        output = sim_creator.addModel(QString("atags64_")+QString::number(i), proposed_x, proposed_y, 0);
        displayLogMessage(output);
    }

    displayLogMessage("Placed four clusters of 64 targets");

    return output;
}

QString RoverGUIPlugin::addPowerLawTargets()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing 256 Targets into 85 Clusters (Power Law pattern)");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.resize(500, 50);
    progress_dialog.show();


    float total_number_of_clusters = 85;
    float clusters_placed = 0;

    QString output = "";
    // One pile of 64
    float clearance = 0.1; //meters
    float proposed_x;
    float proposed_y;

    do
    {
        displayLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        proposed_x = arena_width/2.0 - ((float) rand()) / RAND_MAX*arena_width;
        proposed_y = arena_height/2.0 - ((float) rand()) / RAND_MAX*arena_height;
    }
    while (sim_creator.isLocationOccupied(proposed_x, proposed_y, clearance));

    progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
    output+= sim_creator.addModel("atags64_0", arena_width/2-rand()%boost::math::iround(arena_width), 10-rand()%boost::math::iround(arena_height), 0);

    // Four piles of 16
    for (int i = 0; i < 4; i++)
    {
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            proposed_x = arena_width/2.0 - ((float) rand()) / RAND_MAX*arena_width;
            proposed_y = arena_height/2.0 - ((float) rand()) / RAND_MAX*arena_height;
            displayLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        }
        while (sim_creator.isLocationOccupied(proposed_x, proposed_y, clearance));

        progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
        output+= sim_creator.addModel(QString("atags16_")+QString::number(i), arena_width/2-rand()%boost::math::iround(arena_width), arena_height/2-rand()%boost::math::iround(arena_height), 0);
    }

    // Sixteen piles of 4
    for (int i = 0; i < 16; i++)
    {
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            displayLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = arena_width/2.0 - ((float) rand()) / RAND_MAX*arena_width;
            proposed_y = arena_height/2.0 - ((float) rand()) / RAND_MAX*arena_height;
        }
        while (sim_creator.isLocationOccupied(proposed_x, proposed_y, clearance));

        progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
        output+= sim_creator.addModel(QString("atags4_")+QString::number(i), arena_width/2-rand()%boost::math::iround(arena_width), arena_height/2-rand()%boost::math::iround(arena_height), 0);
    }

    // Sixty-four piles of 1
    for (int i = 0; i < 64; i++)
    {
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            displayLogMessage("Tried to place target "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = arena_width/2.0 - ((float) rand()) / RAND_MAX*arena_width;
            proposed_y = arena_height/2.0 - ((float) rand()) / RAND_MAX*arena_height;
        }
        while (sim_creator.isLocationOccupied(proposed_x, proposed_y, clearance));

        progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
        output+= sim_creator.addModel(QString("at")+QString::number(i), arena_width/2-rand()%boost::math::iround(arena_width), arena_height/2-rand()%boost::math::iround(arena_height), 0);
    }

    return output;
}

void RoverGUIPlugin::checkAndRepositionRover(QString rover_name, float x, float y)
{
    return;
    float arena_width = 20;
    float arena_height = 20;
    if (x < -arena_width/2)
    {
        float duration = 10; //seconds
        float x_comp, y_comp, z_comp;
        x_comp =
        z_comp = 0;
        y_comp = 0;
        displayLogMessage("Moving rover back into the arena");
        QString return_msg = sim_creator.moveRover(rover_name, x_comp, y, 0);
        displayLogMessage(return_msg);
    }
}

void RoverGUIPlugin::readXML(QString path)
{
    QDomDocument xml_doc;
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly))
    {
        displayLogMessage("Error: could not open " + path );
    }

    xml_doc.setContent(&file);

    QDomElement root = xml_doc.documentElement();

    QString gps_reference_lat = root.attribute("referenceLatitude");
    QString gps_reference_long = root.attribute("referenceLongitude");
    QString gps_reference_heading = root.attribute("referenceHeading");
    QString gps_reference_altitude = root.attribute("referenceAltitude");
    QString gps_offset = root.attribute("offset");
    QString gps_drift = root.attribute("drift");
    QString gps_drift_frequency = root.attribute("driftFrequency");
    QString gps_gaussian_noise = root.attribute("gaussianNoise");

    QString imu_update_rate = root.attribute("updateRate");
    QString imu_rpy_offsets = root.attribute("rpyOffsets");
    QString imu_gaussian_noise = root.attribute("gaussianNoise");
    QString imu_accel_drift = root.attribute("accelDrift");
    QString imu_accel_gaussian_noise = root.attribute("accelGaussianNoise");
    QString imu_rate_drift = root.attribute("rateDrift");
    QString imu_rate_gaussian_noise = root.attribute("rateGaussianNoise");
    QString imu_heading_drift = root.attribute("headingDrift");
    QString imu_heading_gaussian_noise = root.attribute("headingGaussianNoise");

    QString camera_update_rate = root.attribute("update_rate");
    QString camera_horizontal_fov = root.attribute("horizontal_fov");
    QString camera_width = root.attribute("width");
    QString camera_height = root.attribute("height");
    QString camera_format = root.attribute("format");
    QString camera_clip_near = root.attribute("near");
    QString camera_clip_far = root.attribute("far");
    QString camera_noise_type = root.attribute("type");
    QString camera_noise_mean = root.attribute("mean");
    QString camera_noise_stddev = root.attribute("stddev");

    QString sonar_noise_mean = root.attribute("samples");
    QString sonar_horz_resolution = root.attribute("resolution");
    QString sonar_min_angle = root.attribute("min_angle");
    QString sonar_max_angle = root.attribute("max_angle");
    QString sonar_min = root.attribute("min");
    QString sonar_max = root.attribute("max");
    QString sonar_range_resolution = root.attribute("resolution");

}

} // End namespace



PLUGINLIB_EXPORT_CLASS(rqt_rover_gui::RoverGUIPlugin, rqt_gui_cpp::Plugin)

