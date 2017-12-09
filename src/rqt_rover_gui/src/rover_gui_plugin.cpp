// Author: Matthew Fricke
// E-mail: matthew@fricke.co.uk
// Date: 9-16-2015
// Purpose: implementation of a simple graphical front end for the UNM-NASA Swarmathon rovers.
// License: GPL3

#include <rover_gui_plugin.h>
#include <Version.h>
#include <pluginlib/class_list_macros.h>
#include <QDir>
#include <QtXml>
#include <QFile>
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
#include <QFileDialog>
#include <QComboBox>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <algorithm>

#ifndef Q_MOC_RUN
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#endif // End Q_MOC_RUN

//#include <regex> // For regex expressions

#include "MapData.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

using namespace std;

using boost::property_tree::ptree;

namespace rqt_rover_gui 
{
  RoverGUIPlugin::RoverGUIPlugin() :
      rqt_gui_cpp::Plugin(),
      widget(0),
      disconnect_threshold(5.0), // Rovers are marked as diconnected if they haven't sent a status message for 5 seconds
      current_simulated_time_in_seconds(0.0),
      last_current_time_update_in_seconds(0.0),
      timer_start_time_in_seconds(0.0),
      timer_stop_time_in_seconds(0.0),
      is_timer_on(false)
  {
    setObjectName("RoverGUI");
    info_log_messages = "";
    diag_log_messages = "";

    // Arbitrarily chosen 10000. These values should be set after experimentation.
    max_info_log_length = 10000;
    max_diag_log_length = 10000;

    joy_process = NULL;
    joystickGripperInterface = NULL;

    obstacle_call_count = 0;

    arena_dim = 20;

    display_sim_visualization = false;

    // Set object clearance values: radius in meters. Values taken from the max dimension of the gazebo collision box for the object.
    // So one half the distance from two opposing corners of the bounding box.
    // In the case of the collection disk a bounding circle is used which gives the radius directly.
    // Values are rounded up to the nearest 10cm.
    target_cluster_size_64_clearance = 0.8;
    target_cluster_size_16_clearance = 0.6;
    target_cluster_size_4_clearance = 0.2;
    target_cluster_size_1_clearance = 0.1;
    rover_clearance = 0.4;
    collection_disk_clearance = 0.5;

    barrier_clearance = 0.5; // Used to prevent targets being placed to close to walls

    map_data = new MapData();
  }

  void RoverGUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    cout << "Rover GUI Starting..." << endl;

    QStringList argv = context.argv();

    widget = new QWidget();

    ui.setupUi(widget);
    
    context.addWidget(widget);

    // Next two lines allow us to catch keyboard input
    widget->installEventFilter(this);
    widget->setFocus();

    // GIT_VERSION is passed in as a compile time definition (see CMakeLists.txt). The version is taken from the last git tag.
    QString version_qstr("<font color='white'>"+GIT_VERSION+"</font>");
    ui.version_number_label->setText(version_qstr);

    widget->setWindowTitle("Rover Interface: Built on " + BUILD_TIME + " Branch: " + GIT_BRANCH);

    string rover_name_msg = "<font color='white'>Rover: " + selected_rover_name + "</font>";
    QString rover_name_msg_qstr = QString::fromStdString(rover_name_msg);
    ui.rover_name->setText(rover_name_msg_qstr);

    // Setup QT message connections
    connect(this, SIGNAL(sendDiagsDataUpdate(QString, QString, QColor)), this, SLOT(receiveDiagsDataUpdate(QString, QString,QColor)));
    connect(ui.rover_list, SIGNAL(currentItemChanged(QListWidgetItem*,QListWidgetItem*)), this, SLOT(currentRoverChangedEventHandler(QListWidgetItem*,QListWidgetItem*)));
    connect(ui.rover_list, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(refocusKeyboardEventHandler()));
    connect(ui.rover_list, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(refocusKeyboardEventHandler()));
    connect(ui.ekf_checkbox, SIGNAL(toggled(bool)), this, SLOT(EKFCheckboxToggledEventHandler(bool)));
    connect(ui.gps_checkbox, SIGNAL(toggled(bool)), this, SLOT(GPSCheckboxToggledEventHandler(bool)));
    connect(ui.encoder_checkbox, SIGNAL(toggled(bool)), this, SLOT(encoderCheckboxToggledEventHandler(bool)));
    connect(ui.global_offset_checkbox, SIGNAL(toggled(bool)), this, SLOT(globalOffsetCheckboxToggledEventHandler(bool)));
    connect(ui.unique_rover_colors_checkbox, SIGNAL(toggled(bool)), this, SLOT(uniqueRoverColorsCheckboxToggledEventHandler(bool)));
    connect(ui.autonomous_control_radio_button, SIGNAL(toggled(bool)), this, SLOT(autonomousRadioButtonEventHandler(bool)));
    connect(ui.joystick_control_radio_button, SIGNAL(toggled(bool)), this, SLOT(joystickRadioButtonEventHandler(bool)));
    connect(ui.all_autonomous_button, SIGNAL(pressed()), this, SLOT(allAutonomousButtonEventHandler()));
    connect(ui.all_stop_button, SIGNAL(pressed()), this, SLOT(allStopButtonEventHandler()));
    connect(ui.build_simulation_button, SIGNAL(pressed()), this, SLOT(buildSimulationButtonEventHandler()));
    connect(ui.clear_simulation_button, SIGNAL(pressed()), this, SLOT(clearSimulationButtonEventHandler()));
    connect(ui.visualize_simulation_button, SIGNAL(pressed()), this, SLOT(visualizeSimulationButtonEventHandler()));
    connect(ui.map_auto_radio_button, SIGNAL(toggled(bool)), this, SLOT(mapAutoRadioButtonEventHandler(bool)));
    connect(ui.map_manual_radio_button, SIGNAL(toggled(bool)), this, SLOT(mapManualRadioButtonEventHandler(bool)));
    connect(ui.map_popout_button, SIGNAL(pressed()), this, SLOT(mapPopoutButtonEventHandler()));
    connect(this, SIGNAL(allStopButtonSignal()), this, SLOT(allStopButtonEventHandler()));


    // Joystick output display - Drive
    connect(this, SIGNAL(joystickDriveForwardUpdate(double)), ui.joy_lcd_drive_forward, SLOT(display(double)));
    connect(this, SIGNAL(joystickDriveBackwardUpdate(double)), ui.joy_lcd_drive_back, SLOT(display(double)));
    connect(this, SIGNAL(joystickDriveLeftUpdate(double)), ui.joy_lcd_drive_left, SLOT(display(double)));
    connect(this, SIGNAL(joystickDriveRightUpdate(double)), ui.joy_lcd_drive_right, SLOT(display(double)));

    // Joystick output display - Gripper
    connect(this, SIGNAL(joystickGripperWristUpUpdate(double)), ui.joy_lcd_gripper_up, SLOT(display(double)));
    connect(this, SIGNAL(joystickGripperWristDownUpdate(double)), ui.joy_lcd_gripper_down, SLOT(display(double)));
    connect(this, SIGNAL(joystickGripperFingersCloseUpdate(double)), ui.joy_lcd_gripper_close, SLOT(display(double)));
    connect(this, SIGNAL(joystickGripperFingersOpenUpdate(double)), ui.joy_lcd_gripper_open, SLOT(display(double)));

    connect(this, SIGNAL(updateCurrentSimulationTimeLabel(QString)), ui.currentSimulationTimeLabel, SLOT(setText(QString)));
    connect(this, SIGNAL(updateObstacleCallCount(QString)), ui.perc_of_time_avoiding_obstacles, SLOT(setText(QString)));
    connect(this, SIGNAL(updateNumberOfTagsCollected(QString)), ui.num_targets_collected_label, SLOT(setText(QString)));
    connect(this, SIGNAL(updateNumberOfSatellites(QString)), ui.gps_numSV_label, SLOT(setText(QString)));
    connect(this, SIGNAL(sendInfoLogMessage(QString)), this, SLOT(receiveInfoLogMessage(QString)));
    connect(this, SIGNAL(sendDiagLogMessage(QString)), this, SLOT(receiveDiagLogMessage(QString)));

    connect(ui.custom_world_path_button, SIGNAL(pressed()), this, SLOT(customWorldButtonEventHandler()));
    connect(ui.custom_distribution_radio_button, SIGNAL(toggled(bool)), this, SLOT(customWorldRadioButtonEventHandler(bool)));
    connect(ui.powerlaw_distribution_radio_button, SIGNAL(toggled(bool)), this, SLOT(powerlawDistributionRadioButtonEventHandler(bool)));
    connect(ui.unbounded_radio_button, SIGNAL(toggled(bool)), this, SLOT(unboundedRadioButtonEventHandler(bool)));
    connect(ui.override_num_rovers_checkbox, SIGNAL(toggled(bool)), this, SLOT(overrideNumRoversCheckboxToggledEventHandler(bool)));
    connect(ui.create_savable_world_checkbox, SIGNAL(toggled(bool)), this, SLOT(createSavableWorldCheckboxToggledEventHandler(bool)));

    connect(this, SIGNAL(updateMapFrameWithCurrentRoverName(QString)), ui.map_frame, SLOT(receiveCurrentRoverName(QString)));

    // Receive waypoint commands from MapFrame
    connect(ui.map_frame, SIGNAL(sendWaypointCmd(WaypointCmd, int, float, float)), this, SLOT(receiveWaypointCmd(WaypointCmd, int, float, float)));
    connect(this, SIGNAL(sendWaypointReached(int)), ui.map_frame, SLOT(receiveWaypointReached(int)));
    
    // Receive log messages from contained frames
    connect(ui.map_frame, SIGNAL(sendInfoLogMessage(QString)), this, SLOT(receiveInfoLogMessage(QString)));

    // Add the checkbox handler so we can process events. We have to listen for itemChange events since
    // we don't have a real chackbox with toggle events
    connect(ui.map_selection_list, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(mapSelectionListItemChangedHandler(QListWidgetItem*)));

    // Create a subscriber to listen for joystick events
    joystick_subscriber = nh.subscribe("/joy", 1000, &RoverGUIPlugin::joyEventHandler, this);

    emit sendInfoLogMessage("Searching for rovers...");

    // Add discovered rovers to the GUI list
    rover_poll_timer = new QTimer(this);
    connect(rover_poll_timer, SIGNAL(timeout()), this, SLOT(pollRoversTimerEventHandler()));
    rover_poll_timer->start(5000);

    // Setup the initial display parameters for the map
    ui.map_frame->setMapData(map_data);
    ui.map_frame->createPopoutWindow(map_data); // This has to happen before the display radio buttons are set
    ui.map_frame->setDisplayGPSData(ui.gps_checkbox->isChecked());
    ui.map_frame->setDisplayEncoderData(ui.encoder_checkbox->isChecked());
    ui.map_frame->setDisplayEKFData(ui.ekf_checkbox->isChecked());

    ui.joystick_frame->setHidden(false);

    ui.custom_world_path_button->setEnabled(true);
    ui.custom_world_path_button->setStyleSheet("color: white; border:1px solid white;");

    // Make the custom rover number combo box look greyed out to begin with
    ui.custom_num_rovers_combobox->setStyleSheet("color: grey; border:2px solid grey;");

    ui.tab_widget->setCurrentIndex(0);
    ui.log_tab->setCurrentIndex(1);

    ui.texture_combobox->setItemData(0, QColor(Qt::white), Qt::TextColorRole);

    ui.visualize_simulation_button->setEnabled(false);
    ui.clear_simulation_button->setEnabled(false);
    ui.all_autonomous_button->setEnabled(false);
    ui.all_stop_button->setEnabled(false);

    ui.visualize_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.clear_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.all_autonomous_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.all_stop_button->setStyleSheet("color: grey; border:2px solid grey;");

    //QString return_msg = startROSJoyNode();
    //displayLogMessage(return_msg);

    info_log_subscriber = nh.subscribe("/infoLog", 10, &RoverGUIPlugin::infoLogMessageEventHandler, this);
    diag_log_subscriber = nh.subscribe("/diagsLog", 10, &RoverGUIPlugin::diagLogMessageEventHandler, this);

    emit updateNumberOfSatellites("<font color='white'>---</font>");
  }

  void RoverGUIPlugin::shutdownPlugin()
  {
    map_data->clear(); // Clear the map and stop drawing before the map_frame is destroyed
    ui.map_frame->clear();
    clearSimulationButtonEventHandler();
    rover_poll_timer->stop();
    stopROSJoyNode();
    ros::shutdown();
  }

void RoverGUIPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
}

void RoverGUIPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
}


// Receives messages from the ROS joystick driver and used them to articulate the gripper and drive the rover.
void RoverGUIPlugin::joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Are we in autonomous mode? If so do not process manual drive and gripper controls.
  if ( rover_control_state.count(selected_rover_name) == 2 )
  {
    return;
  }
  
     // Give the array values some helpful names:
    int left_stick_x_axis = 0; // Gripper fingers close and open
    int left_stick_y_axis = 1; // Gripper wrist up and down

    int right_stick_x_axis = 3; // Turn left and right
    int right_stick_y_axis = 4; // Drive forward and backward

    // Note: joystick stick axis output value are between -1 and 1

     if (joystick_publisher)
        {
         // Handle drive commands - BEGIN

        //Set the gui values. Filter values to be large enough to move the physical rover.
        if (joy_msg->axes[right_stick_y_axis] >= 0.1)
        {
            emit joystickDriveForwardUpdate(joy_msg->axes[right_stick_y_axis]);
        }
        if (joy_msg->axes[right_stick_y_axis] <= -0.1)
        {
            emit joystickDriveBackwardUpdate(-joy_msg->axes[right_stick_y_axis]);
        }
        //If value is too small, display 0.
        if (abs(joy_msg->axes[right_stick_y_axis]) < 0.1)
        {
            emit joystickDriveForwardUpdate(0);
            emit joystickDriveBackwardUpdate(0);
        }

        if (joy_msg->axes[right_stick_x_axis] >= 0.1)
        {
	  emit joystickDriveLeftUpdate(joy_msg->axes[right_stick_x_axis]);
        }
        if (joy_msg->axes[right_stick_x_axis] <= -0.1)
        {
	  emit joystickDriveRightUpdate(-joy_msg->axes[right_stick_x_axis]);
        }
        //If value is too small, display 0.
        if (abs(joy_msg->axes[right_stick_x_axis]) < 0.1)
        {
            emit joystickDriveLeftUpdate(0);
            emit joystickDriveRightUpdate(0);
        }

        // Handle drive commands - END

        // Handle gripper commands - BEGIN

        // The joystick output is a 1D vector since it has a direction (-/+) and a magnitude.
        // This vector is processed by the JoystickGripperInterface to produce gripper angle commands
        float wristCommandVector = joy_msg->axes[left_stick_y_axis];
        float fingerCommandVector = joy_msg->axes[left_stick_x_axis];

        // These if statements just determine which GUI element to update.
        if (wristCommandVector >= 0.1)
        {
            emit joystickGripperWristUpUpdate(wristCommandVector);
        }
        if (wristCommandVector <= -0.1)
        {
            emit joystickGripperWristDownUpdate(-wristCommandVector);
        }

        //If value is too small, display 0
        if (abs(wristCommandVector) < 0.1)
        {
            emit joystickGripperWristUpUpdate(0);
            emit joystickGripperWristDownUpdate(0);
        }

        if (fingerCommandVector >= 0.1)
        {
            emit joystickGripperFingersCloseUpdate(fingerCommandVector);
        }

        if (fingerCommandVector <= -0.1)
        {
            emit joystickGripperFingersOpenUpdate(-fingerCommandVector);
        }

        //If value is too small, display 0
        if (abs(fingerCommandVector) < 0.1)
        {
            emit joystickGripperFingersCloseUpdate(0);
            emit joystickGripperFingersOpenUpdate(0);
        }

        // Use the joystick output to generate ROS gripper commands
        // Lock this section so the interface isnt recreated while in use

        if (joystickGripperInterface)
        {
            try {
                joystickGripperInterface->moveWrist(wristCommandVector);
                joystickGripperInterface->moveFingers(fingerCommandVector);
            } catch (JoystickGripperInterfaceNotReadyException e) {
                emit sendInfoLogMessage("Tried to use the joystick gripper interface before it was ready.");
            }

        }
        else
        {
            emit sendInfoLogMessage("Error: joystickGripperInterface has not been instantiated.");
        }

        // Handle gripper commands - END


        joystick_publisher.publish(joy_msg);
     }
}

void RoverGUIPlugin::EKFEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const boost::shared_ptr<const nav_msgs::Odometry> msg = event.getMessage();

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    QString x_str; x_str.setNum(x);
    QString y_str; y_str.setNum(y);
    // Extract rover name from the message source. Publisher is in the format /*rover_name*_MAP
    size_t found = publisher_name.find("_MAP");
    string rover_name = publisher_name.substr(1,found-1);

    // Store map info for the appropriate rover name
    ui.map_frame->addToEKFRoverPath(rover_name, x, y);
}


void RoverGUIPlugin::encoderEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    string topic = header.at("topic");

    const boost::shared_ptr<const nav_msgs::Odometry> msg = event.getMessage();
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    QString x_str; x_str.setNum(x);
    QString y_str; y_str.setNum(y);

    // Extract rover name from the message source. Get the topic name from the event header. Can't use publisher_name here because it is just /gazebo.
    size_t found = topic.find("/odom/filtered");
    string rover_name = topic.substr(1,found-1);

    // Store map info for the appropriate rover name
   ui.map_frame->addToEncoderRoverPath(rover_name, x, y);
}


void RoverGUIPlugin::GPSEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const boost::shared_ptr<const nav_msgs::Odometry> msg = event.getMessage();
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    QString x_str; x_str.setNum(x);
    QString y_str; y_str.setNum(y);

    // Extract rover name from the message source. Publisher is in the format /*rover_name*_NAVSAT
    size_t found = publisher_name.find("_NAVSAT");
    string rover_name = publisher_name.substr(1,found-1);

    // Store map info for the appropriate rover name
    ui.map_frame->addToGPSRoverPath(rover_name, x, y);
}

void RoverGUIPlugin::GPSNavSolutionEventHandler(const ros::MessageEvent<const ublox_msgs::NavSOL> &event) {
    const boost::shared_ptr<const ublox_msgs::NavSOL> msg = event.getMessage();

    // Extract rover name from the message source. Publisher is in the format /*rover_name*_UBLOX
    size_t found = event.getPublisherName().find("_UBLOX");
    QString rover_name = event.getPublisherName().substr(1,found-1).c_str();

    // Update the number of sattellites detected for the specified rover
    rover_numSV_state[rover_name.toStdString()] = msg.get()->numSV;

    // only update the label if a rover is selected by the user in the GUI
    // and the number of detected satellites is > 0
    if (selected_rover_name.compare("") != 0 && msg.get()->numSV > 0) {
        // Update the label in the GUI with the selected rover's information
        QString newLabelText = QString::number(rover_numSV_state[selected_rover_name]);
        emit updateNumberOfSatellites("<font color='white'>" + newLabelText + "</font>");
    } else {
        emit updateNumberOfSatellites("<font color='white'>---</font>");
    }
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

     // For debugging
     //QString debug_msg = "Received image ";
     //debug_msg += "(" + QString::number(image_rows) + " x " + QString::number(image_cols) +")";
     //cout << debug_msg.toStdString() << endl;

    // ROS_INFO_STREAM("Image received Size:" + rows_stream.str() + "x" + cols_stream.str());

     // Create QImage to hold the image
     //const uchar* image_buffer = (const uchar*)cv_image_ptr->image.data; // extract the raw data
     QImage qimg(&(image->data[0]), image_cols, image_rows, image_step, QImage::Format_RGB888);
     qimg = qimg.rgbSwapped(); // Convert from RGB to BGR which is the output format for the rovers.
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

// Receives and stores the status update messages from rovers
void RoverGUIPlugin::statusEventHandler(const ros::MessageEvent<std_msgs::String const> &event)
{
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = ros::Time::now();

    // Extract rover name from the message source

    // This method is used rather than reading the publisher name to accomodate teams that changed the node name.
    string topic = header.at("topic");
    size_t found = topic.find("/status");
    string rover_name = topic.substr(1,found-1);

    const std_msgs::StringConstPtr& msg = event.getMessage();

    string status = msg->data;

    RoverStatus rover_status;
    rover_status.status_msg = status;
    rover_status.timestamp = receipt_time;

    rover_statuses[rover_name] = rover_status;
}

void RoverGUIPlugin::waypointEventHandler(const swarmie_msgs::Waypoint& msg)
{
  emit sendWaypointReached(msg.id);
}

// Counts the number of obstacle avoidance calls
void RoverGUIPlugin::obstacleEventHandler(const ros::MessageEvent<const std_msgs::UInt8> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const std_msgs::UInt8ConstPtr& msg = event.getMessage();

    //QString displ = QString("Target number ") + QString::number(msg->data) + QString(" found.");

    // 0 for no obstacle, 1 for right side obstacle, and 2 for left side obsticle
    int code = msg->data;

    if (code != 0)
    {
        emit updateObstacleCallCount("<font color='white'>"+QString::number(++obstacle_call_count)+"</font>");
    }
}

// Takes the published score value from the ScorePlugin and updates the GUI
void RoverGUIPlugin::scoreEventHandler(const ros::MessageEvent<const std_msgs::String> &event) {
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const std_msgs::StringConstPtr& msg = event.getMessage();
    std::string tags_collected = msg->data;

    emit updateNumberOfTagsCollected("<font color='white'>"+QString::fromStdString(tags_collected)+"</font>");
}

void RoverGUIPlugin::simulationTimerEventHandler(const rosgraph_msgs::Clock& msg) {

    current_simulated_time_in_seconds = msg.clock.toSec();

    bool updateTimeLabel = ((current_simulated_time_in_seconds - last_current_time_update_in_seconds) >= 1.0) ? (true) : (false);

    // only update the current time once per second; faster update rates make the GUI unstable
    // and in the worst cases it will hang and/or crash
    if (updateTimeLabel == true) {
        emit updateCurrentSimulationTimeLabel("<font color='white'>" +
                                              QString::number(getHours(current_simulated_time_in_seconds)) + " hours, " +
                                              QString::number(getMinutes(current_simulated_time_in_seconds)) + " minutes, " +
                                              QString::number(floor(getSeconds(current_simulated_time_in_seconds))) + " seconds</font>");
        last_current_time_update_in_seconds = current_simulated_time_in_seconds;
    }

    // this catches the case when the /clock timer is not running
    // AKA: when we are not running a simulation
    if (current_simulated_time_in_seconds <= 0.0) {
        return;
    }

    if (is_timer_on == true) {
        if (current_simulated_time_in_seconds >= timer_stop_time_in_seconds) {
            is_timer_on = false;
            emit allStopButtonSignal();
            emit sendInfoLogMessage("\nSimulation timer complete at: " +
                                    QString::number(getHours(current_simulated_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(current_simulated_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(current_simulated_time_in_seconds)) + " seconds\n");
        }
    }
}

void RoverGUIPlugin::currentRoverChangedEventHandler(QListWidgetItem *current, QListWidgetItem *previous)
{
    emit sendInfoLogMessage("Selcted Rover Changed");

    if (!current) return; // Check to make sure the current selection isn't null

    // Extract rover name
    string rover_name_and_status = current->text().toStdString();

    // Rover names start at the begining of the rover name and status string and end at the first space
    size_t rover_name_length = rover_name_and_status.find_first_of(" ");
    string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);

    selected_rover_name = ui_rover_name;

    string rover_name_msg = "<font color='white'>Rover: " + selected_rover_name + "</font>";
    QString rover_name_msg_qstr = QString::fromStdString(rover_name_msg);
    ui.rover_name->setText(rover_name_msg_qstr);

    emit sendInfoLogMessage(QString("Selected rover: ") + QString::fromStdString(selected_rover_name));

    // Tell the MapFrame that the currently selected rover changed
    emit updateMapFrameWithCurrentRoverName(QString::fromStdString(selected_rover_name));

    // Attempt to read the simulation model xml file if it exists. If it does not exist assume this is a physical rover.
    const char *name = "GAZEBO_MODEL_PATH";
    char *model_root_cstr;
    model_root_cstr = getenv(name);
    QString model_root(model_root_cstr);

    QString model_path = model_root+"/"+QString::fromStdString(selected_rover_name)+"/model.sdf";

    readRoverModelXML(model_path);
    
    //Set up subscribers
    image_transport::ImageTransport it(nh);
    camera_subscriber = it.subscribe("/"+selected_rover_name+"/targets/image", 1, &RoverGUIPlugin::cameraEventHandler, this, image_transport::TransportHints("theora"));
    imu_subscriber = nh.subscribe("/"+selected_rover_name+"/imu", 10, &RoverGUIPlugin::IMUEventHandler, this);
    us_center_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarCenter", 10, &RoverGUIPlugin::centerUSEventHandler, this);
    us_left_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarLeft", 10, &RoverGUIPlugin::leftUSEventHandler, this);
    us_right_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarRight", 10, &RoverGUIPlugin::rightUSEventHandler, this);

    emit sendInfoLogMessage(QString("Displaying map for ")+QString::fromStdString(selected_rover_name));

    // Add to the rover map.
    QListWidgetItem* map_selection_item = ui.map_selection_list->item(ui.rover_list->row(current));
    map_selection_item->setCheckState(Qt::Checked);

    // No entry for this rover name
    if ( 0 == rover_control_state.count(selected_rover_name) )
    {
        // Default to joystick
        ui.joystick_control_radio_button->setChecked(true);
        ui.autonomous_control_radio_button->setChecked(false);
        joystickRadioButtonEventHandler(true); // Manually trigger the joystick selected event
        rover_control_state[selected_rover_name]=1;
        emit sendInfoLogMessage("New rover selected");
    }
    else
    {
        int control_state = rover_control_state.find(selected_rover_name)->second;

        switch (control_state)
        {
        case 1: // manual
            ui.joystick_control_radio_button->setChecked(true);
            ui.autonomous_control_radio_button->setChecked(false);
            ui.joystick_frame->setHidden(false);
            joystickRadioButtonEventHandler(true); // Manually trigger the joystick selected event
            break;
        case 2: // autonomous
            ui.joystick_control_radio_button->setChecked(false);
            ui.autonomous_control_radio_button->setChecked(true);
            ui.joystick_frame->setHidden(true);
            autonomousRadioButtonEventHandler(true); // Manually trigger the autonomous selected event
            break;
        default:
            emit sendInfoLogMessage("Unknown control state: "+QString::number(control_state));
        }

        emit sendInfoLogMessage("Existing rover selected");
    }

    // only update the number of satellites if a valid rover name has been selected
    if (selected_rover_name.compare("") != 0 && rover_numSV_state[selected_rover_name] > 0) {
        QString newLabelText = QString::number(rover_numSV_state[selected_rover_name]);
        emit updateNumberOfSatellites("<font color='white'>" + newLabelText + "</font>");
    } else {
        emit updateNumberOfSatellites("<font color='white'>---</font>");
    }

    // Enable control mode radio group now that a rover has been selected
    ui.autonomous_control_radio_button->setEnabled(true);
    ui.joystick_control_radio_button->setEnabled(true);
}

void RoverGUIPlugin::pollRoversTimerEventHandler()
{
    //If there are no rovers connected to the GUI, reset the obstacle call count to 0
    if(ui.rover_list->count() == 0)
    {
        obstacle_call_count = 0;
    }

    // Returns rovers that have created a status topic
    set<string>new_rover_names = findConnectedRovers();

    std::set<string> orphaned_rover_names;

    // Calculate which of the old rover names are not in the new list of rovers then clear their maps and control states.
    std::set_difference(rover_names.begin(), rover_names.end(), new_rover_names.begin(), new_rover_names.end(),
        std::inserter(orphaned_rover_names, orphaned_rover_names.end()));

    for (set<string>::iterator it = orphaned_rover_names.begin(); it != orphaned_rover_names.end(); ++it)
    {
        emit sendInfoLogMessage(QString("Clearing interface data for disconnected rover ") + QString::fromStdString(*it));
        map_data->clear(*it);
        ui.map_frame->clear(*it);
        rover_control_state.erase(*it); // Remove the control state for orphaned rovers
        rover_numSV_state.erase(*it);
        rover_statuses.erase(*it);

        // If the currently selected rover disconnected, shutdown its subscribers and publishers
        if (it->compare(selected_rover_name) == 0)
        {
            camera_subscriber.shutdown();
            imu_subscriber.shutdown();
            us_center_subscriber.shutdown();
            us_left_subscriber.shutdown();
            us_right_subscriber.shutdown();
            joystick_publisher.shutdown();

            //Reset selected rover name to empty string
            selected_rover_name = "";
        }

        // For the other rovers that disconnected...

        // Shutdown the subscribers
        status_subscribers[*it].shutdown();
        waypoint_subscribers[*it].shutdown();
        encoder_subscribers[*it].shutdown();
        gps_subscribers[*it].shutdown();
        gps_nav_solution_subscribers[*it].shutdown();
        ekf_subscribers[*it].shutdown();
        rover_diagnostic_subscribers[*it].shutdown();

        // Delete the subscribers
        status_subscribers.erase(*it);
        waypoint_subscribers.erase(*it);
        encoder_subscribers.erase(*it);
        gps_subscribers.erase(*it);
        gps_nav_solution_subscribers.erase(*it);
        ekf_subscribers.erase(*it);
        rover_diagnostic_subscribers.erase(*it);
        
        // Shudown Publishers
        control_mode_publishers[*it].shutdown();
        waypoint_cmd_publishers[*it].shutdown();

        // Delete Publishers
        control_mode_publishers.erase(*it);
        waypoint_cmd_publishers.erase(*it);

        ui.map_frame->resetWaypointPathForSelectedRover(*it);
    }

    // Wait for a rover to connect
    if (new_rover_names.empty())
    {
        //displayLogMessage("Waiting for rover to connect...");
        selected_rover_name = "";
        rover_control_state.clear();
        rover_numSV_state.clear();
        rover_names.clear();        
        ui.rover_list->clearSelection();
        ui.rover_list->clear();
        ui.rover_diags_list->clear();
        ui.map_selection_list->clear();

        // Disable control mode group since no rovers are connected
        ui.autonomous_control_radio_button->setEnabled(false);
        ui.joystick_control_radio_button->setEnabled(false);
        ui.all_autonomous_button->setEnabled(false);
        ui.all_stop_button->setEnabled(false);
        ui.all_autonomous_button->setStyleSheet("color: grey; border:2px solid grey;");
        ui.all_stop_button->setStyleSheet("color: grey; border:2px solid grey;");

    }
    else if (new_rover_names == rover_names)
    {

        // Just update the statuses in ui rover list
        for(int row = 0; row < ui.rover_list->count(); row++)
        {
            QListWidgetItem *item = ui.rover_list->item(row);

            // Extract rover name
            string rover_name_and_status = item->text().toStdString();

            // Rover names start at the begining of the rover name and status string and end at the first space
            size_t rover_name_length = rover_name_and_status.find_first_of(" ");
            string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);

            // Get current status

            RoverStatus updated_rover_status;
            // Build new ui rover list string
            try
            {
                updated_rover_status = rover_statuses.at(ui_rover_name);
            }
            catch (std::out_of_range& e)
            {
                emit sendInfoLogMessage("Error: No status entry for rover " + QString::fromStdString(ui_rover_name));
            }


            // Build new ui rover list string
            QString updated_rover_name_and_status = QString::fromStdString(ui_rover_name)
                                                    + " ("
                                                    + QString::fromStdString(updated_rover_status.status_msg)
                                                    + ")";

            // Update the UI
            item->setText(updated_rover_name_and_status);
        }
    }
    else
    {
    rover_names = new_rover_names;
    
    emit sendInfoLogMessage("List of connected rovers has changed");
    selected_rover_name = "";
    ui.rover_list->clearSelection();
    ui.rover_list->clear();

    // Also clear the rover diagnostics list
    ui.rover_diags_list->clear();
    ui.map_selection_list->clear();
    
    //Enable all autonomous button
    ui.all_autonomous_button->setEnabled(true);
    ui.all_autonomous_button->setStyleSheet("color: white; border:2px solid white;");

    // This code is from above. Consider moving into a function or restructuring
    for(set<string>::const_iterator i = rover_names.begin(); i != rover_names.end(); ++i)
    {
        //Set up publishers
        control_mode_publishers[*i]=nh.advertise<std_msgs::UInt8>("/"+*i+"/mode", 10, true); // last argument sets latch to true
        waypoint_cmd_publishers[*i]=nh.advertise<swarmie_msgs::Waypoint>("/"+*i+"/waypoints/cmd", 10, true);
        

        //Set up subscribers
        status_subscribers[*i] = nh.subscribe("/"+*i+"/status", 10, &RoverGUIPlugin::statusEventHandler, this);
        waypoint_subscribers[*i] = nh.subscribe("/"+*i+"/waypoints", 10, &RoverGUIPlugin::waypointEventHandler, this);
        obstacle_subscribers[*i] = nh.subscribe("/"+*i+"/obstacle", 10, &RoverGUIPlugin::obstacleEventHandler, this);
        encoder_subscribers[*i] = nh.subscribe("/"+*i+"/odom/filtered", 10, &RoverGUIPlugin::encoderEventHandler, this);
        ekf_subscribers[*i] = nh.subscribe("/"+*i+"/odom/ekf", 10, &RoverGUIPlugin::EKFEventHandler, this);
        gps_subscribers[*i] = nh.subscribe("/"+*i+"/odom/navsat", 10, &RoverGUIPlugin::GPSEventHandler, this);
        gps_nav_solution_subscribers[*i] = nh.subscribe("/"+*i+"/navsol", 10, &RoverGUIPlugin::GPSNavSolutionEventHandler, this);
        rover_diagnostic_subscribers[*i] = nh.subscribe("/"+*i+"/diagnostics", 1, &RoverGUIPlugin::diagnosticEventHandler, this);

        RoverStatus rover_status;
        // Build new ui rover list string
        try
        {
            rover_status = rover_statuses.at(*i);
        }
        catch (std::out_of_range& e)
        {
            emit sendInfoLogMessage("No status entry for rover " + QString::fromStdString(*i));
        }

        QString rover_name_and_status = QString::fromStdString(*i) // Add the rover name
                                                + " (" // Delimiters needed for parsing the rover name and status when read
                                                +  QString::fromStdString(rover_status.status_msg) // Add the rover status
                                                + ")";

        QListWidgetItem* new_item = new QListWidgetItem(rover_name_and_status);
        new_item->setForeground(Qt::green);
        ui.rover_list->addItem(new_item);

        // Create the corresponding diagnostic data listwidgetitem
        QListWidgetItem* new_diags_item = new QListWidgetItem("");

        // The user shouldn't be able to select the diagnostic output
        new_diags_item->setFlags(new_diags_item->flags() & ~Qt::ItemIsSelectable);

        ui.rover_diags_list->addItem(new_diags_item);

        // Add the map selection checkbox for this rover
        QListWidgetItem* new_map_selection_item = new QListWidgetItem("");

        // set checkable but not selectable flags
        new_map_selection_item->setFlags(new_map_selection_item->flags() | Qt::ItemIsUserCheckable);
        new_map_selection_item->setFlags(new_map_selection_item->flags() & ~Qt::ItemIsSelectable);
        new_map_selection_item->setCheckState(Qt::Unchecked);

        // Add to the widget list
        ui.map_selection_list->addItem(new_map_selection_item);

    }
    }

    // If rovers have not sent a status message recently mark them as disconnected
    for(int row = 0; row < ui.rover_list->count(); row++)
    {
        QListWidgetItem *rover_item = ui.rover_list->item(row);
        QListWidgetItem *diags_item = ui.rover_diags_list->item(row);

        // Extract rover name
        string rover_name_and_status = rover_item->text().toStdString();

        // Rover names start at the begining of the rover name and status string and end at the first space
        size_t rover_name_length = rover_name_and_status.find_first_of(" ");
        string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);

        // Check the time of last contact with this rover
        RoverStatus rover_status = rover_statuses[ui_rover_name];
        if (ros::Time::now() - rover_status.timestamp < disconnect_threshold)
        {
          rover_item->setForeground(Qt::green);
        }
        else
        {
          rover_item->setForeground(Qt::red);
          diags_item->setForeground(Qt::red);
          diags_item->setText("disconnected");
        }
    }
}

void RoverGUIPlugin::centerUSEventHandler(const sensor_msgs::Range::ConstPtr& msg)
{
    // Temp hardcode max and min because setting the max and min on the controller side causes a problem
    float min_range = 0.01;; // meters
    float max_range = 3;


    //ui.us_frame->setCenterRange(msg->range, msg->min_range, msg->max_range);
    ui.us_frame->setCenterRange(msg->range, min_range, max_range);
 }

void RoverGUIPlugin::rightUSEventHandler(const sensor_msgs::Range::ConstPtr& msg)
{
    // Temp hardcode max and min because setting the max and min on the controller side causes a problem
    float min_range = 0.01;
    float max_range = 3;
    //ui.us_frame->setCenterRange(msg->range, min_range, max_range);

    ui.us_frame->setRightRange(msg->range, min_range, max_range);

//    ui.us_frame->setRightRange(msg->range, msg->min_range, msg->max_range);

}

void RoverGUIPlugin::leftUSEventHandler(const sensor_msgs::Range::ConstPtr& msg)
{
    // Temp hardcode max and min because setting the max and min on the controller side causes a problem
    float min_range = 0.01;;
    float max_range = 3;


    //ui.us_frame->setLeftRange(msg->range, msg->min_range, msg->max_range);
    ui.us_frame->setLeftRange(msg->range, min_range, max_range);
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

// This handler receives data messages from the diagnostics package. It uses a float array to package the
// data for flexibility. This means callers have to know what data is stored at each poistion.
// When the data we cant to display stabalizes we should consider changing this to a custom
// ROS message type that names the data being stored.
// We extract the sender name from the ROS topic name rather than the publisher node name because that
// tends to be more stable. Sometimes teams rename the nodes but renaming the topics would cause
// other problems for them. This is distinct from the diagnostics log handler which received messages rather
// than continual data readings.
void RoverGUIPlugin::diagnosticEventHandler(const ros::MessageEvent<const std_msgs::Float32MultiArray> &event) {

    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    // Extract rover name from the message source
    string topic = header.at("topic");
    size_t found = topic.find("/diagnostics");
    string rover_name = topic.substr(1,found-1);

    const boost::shared_ptr<const std_msgs::Float32MultiArray> msg = event.getMessage();

    string diagnostic_display = "";

    // Read data from the message array
    int wireless_quality = static_cast<int>(msg->data[0]); // Wireless quality is an integer value
    float byte_rate = msg->data[1]; // Bandwidth used by the wireless interface
    float sim_rate = msg->data[2]; // Simulation update rate

    // Declare the output colour variables
    int red = 255;
    int green = 255;
    int blue = 255;
    
    // Check whether there is sim update data. Will be < 0 for the sim rate if a physical rover is sending the data.
    // If so assume the diagnostic data is coming from a simulated rover.
    // TODO: replace with a proper message type so we don't need to use in stream flags like this.
    if ( sim_rate < 0 )
    {
        // Change the color of the text based on the link quality. These numbers are from
        // experience but need tuning. The raw quality value is scaled into a new range to make the colors more meaningful
        int quality_max = 70;
        int quality_min = 0;
        int scaled_max = 10;
        int scaled_min = 0;
        int quality_range = quality_max - quality_min; // Max minus min
        int scaled_range = scaled_max - scaled_min; // Scaled to match the experimental quality of the connection. Below 30 should be red = bad
        int scaled_wireless_quality = (((wireless_quality - quality_min)*static_cast<float>(scaled_range))/quality_range) + scaled_min; // scale the quality to the new range

        if (scaled_range != 0)
        {
            green = 255 * scaled_wireless_quality/static_cast<float>(scaled_range);
            red = 255 * (2*scaled_range - (scaled_wireless_quality))/static_cast<float>(2*scaled_range);
        }
        else
        {
            green = 0;
            red = 0;
        }

        blue = 0;

        // Make sure color is in a valid range
        if (green > 255) green = 255;
        if (blue > 255) blue = 255;
        if (red > 255) red = 255;

        if (green < 0) green = 0;
        if (blue < 0) blue = 0;
        if (red < 0) red = 0;

        diagnostic_display = to_string(wireless_quality);

        // Convert the byte rate into a string with units
        // Rate in B/s
        float rate = byte_rate;

        // Conversion factors to make the rate human friendly
        int KB = 1024;
        int MB = 1024*1024;

        string rate_str;
        string units;
        if (rate < KB) {
            rate_str = to_string(rate);
            units = "B/s";
        } else if (rate < MB) {
            rate = rate/KB;
            units = "KB/s";
        } else {
            rate = rate/MB;
            units = "MB/s";
        }

        rate_str = to_string(rate);

        if (rate_str[rate_str.find(".")+1] != '0')
            rate_str = rate_str.erase(rate_str.find(".")+2,string::npos);
        else
            rate_str = rate_str.erase(rate_str.find("."),string::npos);

        diagnostic_display += " | " + rate_str + " " + units;

    }
    else
    {
        string sim_rate_str = to_string(sim_rate);

        // Truncate to 1 digit
        if (sim_rate_str[sim_rate_str.find(".")+1] != '0')
            sim_rate_str = sim_rate_str.erase(sim_rate_str.find(".")+3,string::npos);
        else
            sim_rate_str = sim_rate_str.erase(sim_rate_str.find("."),string::npos);

        red = 255*(1-sim_rate);
        green = 255*sim_rate;
        blue = 0;

        // Make sure color is in a valid range
        if (green > 255) green = 255;
        if (blue > 255) blue = 255;
        if (red > 255) red = 255;

        if (green < 0) green = 0;
        if (blue < 0) blue = 0;
        if (red < 0) red = 0;

        diagnostic_display = sim_rate_str + " sim rate";
    }

    emit sendDiagsDataUpdate(QString::fromStdString(rover_name), QString::fromStdString(diagnostic_display), QColor(red, green, blue));

}

// We use item changed signal as a proxy for the checkbox being clicked
void RoverGUIPlugin::mapSelectionListItemChangedHandler(QListWidgetItem* changed_item)
{
    // Get the rover name associated with this map selction list item
    int row = ui.map_selection_list->row(changed_item);

    // Rover names start at the begining of the rover name and status string and end at the first space
    QListWidgetItem* rover_item = ui.rover_list->item(row);

    // Extract the rover name corresponding to the changed map selection item
    string rover_name_and_status = rover_item->text().toStdString();

    // Rover names start at the begining of the rover name and status string and end at the first space
    size_t rover_name_length = rover_name_and_status.find_first_of(" ");
    string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);

    bool checked = changed_item->checkState();

    emit sendInfoLogMessage("Map selection changed to " + (checked ? QString("true") : QString("false")) + " for rover " + QString::fromStdString(ui_rover_name));

    ui.map_frame->setWhetherToDisplay(ui_rover_name, checked);
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

void RoverGUIPlugin::globalOffsetCheckboxToggledEventHandler(bool checked)
{
    ui.map_frame->setGlobalOffset(checked);
}

void RoverGUIPlugin::uniqueRoverColorsCheckboxToggledEventHandler(bool checked)
{
    ui.map_frame->setDisplayUniqueRoverColors(checked);
}

void RoverGUIPlugin::displayDiagLogMessage(QString msg)
{
    if (msg.isEmpty()) msg = "Message is empty";
    if (msg == NULL) msg = "Message was a NULL pointer";


     // replace new lines with <BR> in the message. Uppercase in order to differentiate from <br> below
    msg.replace("\n","<BR>");


    // Prevent the log from growning too large. Maintain a maximum specified size
    // by removing characters from the beginning of the log.
    // Calculate the number of characters in the log. If the log size is larger than the max size specified
    // then find the position of the first newline that reduces the log size to less than the max size.
    // Delete all characters up to that position.
    int overflow = diag_log_messages.size() - max_diag_log_length; 
    
    // Get the position of the the first newline after the overflow amount
    int newline_pos = diag_log_messages.indexOf( "<br>", overflow, Qt::CaseSensitive );
    
    // If the max size is exceeded and the number of characters to remove is less than
    // the size of the log remove those characters.
    if ( overflow > 0 && newline_pos < diag_log_messages.size() ) {   
      diag_log_messages.remove(0, newline_pos);
    }

    diag_log_messages += "<font color='white'>"+msg+"</font><br>";

    ui.diag_log->setText(diag_log_messages);

    QScrollBar *sb = ui.diag_log->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void RoverGUIPlugin::displayInfoLogMessage(QString msg)
{
    if (msg.isEmpty()) msg = "Message is empty";
    if (msg == NULL) msg = "Message was a NULL pointer";

    // replace new lines with <BR> in the message. Uppercase in order to differentiate from <br> below
    msg.replace("\n","<BR>");

    // Prevent the log from growning too large. Maintain a maximum specified size
    // by removing characters from the beginning of the log.
    // Calculate the number of characters in the log. If the log size is larger than the max size specified
    // then find the position of the first newline that reduces the log size to less than the max size.
    // Delete all characters up to that position.
    int overflow = info_log_messages.size() - max_info_log_length; 
    
    // Get the position of the the first newline after the overflow amount
    int newline_pos = info_log_messages.indexOf( "<br>", overflow, Qt::CaseSensitive );
    
    // If the max size is exceeded and the number of characters to remove is less than
    // the size of the log remove those characters.
    if ( overflow > 0 && newline_pos < info_log_messages.size() ) {   
      info_log_messages.remove(0, newline_pos);
    }
    
    // Use the <br> tag to make log messages atomic for easier deletion later.
    info_log_messages += "<font color='white'>"+msg+"</font><br>";

    ui.info_log->setText(info_log_messages);

    QScrollBar *sb = ui.info_log->verticalScrollBar();
    sb->setValue(sb->maximum());
}

// These button handlers allow the user to select whether to manually pan and zoom the map
// or use auto scaling.
void RoverGUIPlugin::mapAutoRadioButtonEventHandler(bool marked)
{
    if (!marked) return;
    ui.map_frame->setAutoTransform();

}

void RoverGUIPlugin::mapManualRadioButtonEventHandler(bool marked)
{
    if (!marked) return;
    ui.map_frame->setManualTransform();
}


void RoverGUIPlugin::autonomousRadioButtonEventHandler(bool marked)
{
    if (!marked) return;

    rover_control_state[selected_rover_name] = 2;

    std_msgs::UInt8 control_mode_msg;
    control_mode_msg.data = 2; // 2 indicates autonomous control

    control_mode_publishers[selected_rover_name].publish(control_mode_msg);
    emit sendInfoLogMessage(QString::fromStdString(selected_rover_name)+" changed to autonomous control");

    QString return_msg = stopROSJoyNode();
    emit sendInfoLogMessage(return_msg);
    
    //Enable all stop button
    ui.all_stop_button->setEnabled(true);
    ui.all_stop_button->setStyleSheet("color: white; border:2px solid white;");

    //Hide joystick frame
    ui.joystick_frame->setHidden(true);

    // disable waypoint input in map frame
    ui.map_frame->disableWaypoints(selected_rover_name);
}

void RoverGUIPlugin::joystickRadioButtonEventHandler(bool marked)
{
    if (!marked) return;

    rover_control_state[selected_rover_name] = 1;
    emit sendInfoLogMessage("Setting up joystick publisher " + QString::fromStdString("/"+selected_rover_name+"/joystick"));

    // Setup joystick publisher
    joystick_publisher = nh.advertise<sensor_msgs::Joy>("/"+selected_rover_name+"/joystick", 10, this);

    // Setup Gripper publishers

    // Have to allocate the joystickGripperInterface on the heap because it derives from QObject which disallows copy constructors
    // Lock this section to prevent the inferface from being changed while in use

    if (joystickGripperInterface != NULL)
    {
        emit sendInfoLogMessage("Redirecting existing Joystick Gripper Interface to " + QString::fromStdString(selected_rover_name));
        joystickGripperInterface->changeRovers(selected_rover_name);
    }
    else
    {
        emit sendInfoLogMessage("Setting up Joystick Gripper Interface for  " + QString::fromStdString(selected_rover_name));
        joystickGripperInterface = new JoystickGripperInterface(nh, selected_rover_name);
    }

    std_msgs::UInt8 control_mode_msg;
    control_mode_msg.data = 1; // 1 indicates manual control

    control_mode_publishers[selected_rover_name].publish(control_mode_msg);
    emit sendInfoLogMessage(QString::fromStdString(selected_rover_name)+" changed to joystick control");\

    QString return_msg = startROSJoyNode();
    emit sendInfoLogMessage(return_msg);
    
    //Enable all autonomous button
    ui.all_autonomous_button->setEnabled(true);
    ui.all_autonomous_button->setStyleSheet("color: white; border:2px solid white;");
    
    //Show joystick frame
    ui.joystick_frame->setHidden(false);

    // enable wayoint input in the map frame
    ui.map_frame->enableWaypoints(selected_rover_name);
}

void RoverGUIPlugin::allAutonomousButtonEventHandler()
{
    emit sendInfoLogMessage("changing all rovers to autonomous control...");

    std::string remember_selected_rover_name = selected_rover_name;
    int remember_selected_index = ui.rover_list->currentRow();
    int selected_index = -1; // zero array indexing, ensure last selected index is in range

    // manually trigger the autonomous radio button event for all rovers
    for (set<string>::iterator it = rover_names.begin(); it != rover_names.end(); it++)
    {
        selected_index++;
        selected_rover_name = *it;
        autonomousRadioButtonEventHandler(true);
    }

    // trigger the current rover changed event:
    // if we previously selected a rover, keep that rover selected
    if (remember_selected_index >= 0)
    {
        selected_rover_name = remember_selected_rover_name;
        ui.rover_list->setCurrentItem(ui.rover_list->item(remember_selected_index));
    }
    // otherwise, default to the last rover in the rover list
    else
    {
        ui.rover_list->setCurrentItem(ui.rover_list->item(selected_index));
    }

    ui.joystick_control_radio_button->setEnabled(true);
    ui.autonomous_control_radio_button->setEnabled(true);
    ui.joystick_control_radio_button->setChecked(false);
    ui.autonomous_control_radio_button->setChecked(true);
    ui.joystick_frame->setHidden(true);
    
    // Disable all autonomous button
    ui.all_autonomous_button->setEnabled(false);
    ui.all_autonomous_button->setStyleSheet("color: grey; border:2px solid grey;");

    // Experiment Timer START

    // this catches the case when the /clock timer is not running
    // AKA: when we are not running a simulation
    if (current_simulated_time_in_seconds > 0.0) {
        if (ui.simulation_timer_combobox->currentText() == "no time limit") {
            timer_start_time_in_seconds = 0.0;
            timer_stop_time_in_seconds = 0.0;
            is_timer_on = false;
        } else if (ui.simulation_timer_combobox->currentText() == "10 min (Testing)") {
            timer_start_time_in_seconds = current_simulated_time_in_seconds;
            timer_stop_time_in_seconds = timer_start_time_in_seconds + 600.0;
            is_timer_on = true;
            emit sendInfoLogMessage("\nSetting experiment timer to start at: " +
                                    QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_start_time_in_seconds)) + " seconds");
            ui.simulationTimerStartLabel->setText("<font color='white'>" +
                                                  QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                                  QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                                  QString::number(floor(getSeconds(timer_start_time_in_seconds))) + " seconds</font>");
            emit sendInfoLogMessage("Setting experiment timer to stop at: " +
                                    QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_stop_time_in_seconds)) + " seconds\n");
            ui.simulationTimerStopLabel->setText("<font color='white'>" +
                                                 QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                                 QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                                 QString::number(floor(getSeconds(timer_stop_time_in_seconds))) + " seconds</font>");
            ui.simulation_timer_combobox->setEnabled(false);
            ui.simulation_timer_combobox->setStyleSheet("color: grey; border:2px solid grey;");
        } else if (ui.simulation_timer_combobox->currentText() == "20 min (Preliminary)") {
            timer_start_time_in_seconds = current_simulated_time_in_seconds;
            timer_stop_time_in_seconds = timer_start_time_in_seconds + 1200.0;
            is_timer_on = true;
            emit sendInfoLogMessage("\nSetting experiment timer to start at: " +
                                    QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_start_time_in_seconds)) + " seconds");
            ui.simulationTimerStartLabel->setText("<font color='white'>" +
                                                  QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                                  QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                                  QString::number(floor(getSeconds(timer_start_time_in_seconds))) + " seconds</font>");
            emit sendInfoLogMessage("Setting experiment timer to stop at: " +
                                    QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_stop_time_in_seconds)) + " seconds\n");
            ui.simulationTimerStopLabel->setText("<font color='white'>" +
                                                 QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                                 QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                                 QString::number(floor(getSeconds(timer_stop_time_in_seconds))) + " seconds</font>");
            ui.simulation_timer_combobox->setEnabled(false);
            ui.simulation_timer_combobox->setStyleSheet("color: grey; border:2px solid grey;");
        } else if (ui.simulation_timer_combobox->currentText() == "30 min (Preliminary)") {
            timer_start_time_in_seconds = current_simulated_time_in_seconds;
            timer_stop_time_in_seconds = timer_start_time_in_seconds + 1800.0;
            is_timer_on = true;
            emit sendInfoLogMessage("\nSetting experiment timer to start at: " +
                                    QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_start_time_in_seconds)) + " seconds");
            ui.simulationTimerStartLabel->setText("<font color='white'>" +
                                                  QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                                  QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                                  QString::number(floor(getSeconds(timer_start_time_in_seconds))) + " seconds</font>");
            emit sendInfoLogMessage("Setting experiment timer to stop at: " +
                                    QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_stop_time_in_seconds)) + " seconds\n");
            ui.simulationTimerStopLabel->setText("<font color='white'>" +
                                                 QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                                 QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                                 QString::number(floor(getSeconds(timer_stop_time_in_seconds))) + " seconds</font>");
            ui.simulation_timer_combobox->setEnabled(false);
            ui.simulation_timer_combobox->setStyleSheet("color: grey; border:2px solid grey;");
        } else if (ui.simulation_timer_combobox->currentText() == "60 min (Final)") {
            timer_start_time_in_seconds = current_simulated_time_in_seconds;
            timer_stop_time_in_seconds = timer_start_time_in_seconds + 3600.0;
            is_timer_on = true;
            emit sendInfoLogMessage("\nSetting experiment timer to start at: " +
                                    QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_start_time_in_seconds)) + " seconds");
            ui.simulationTimerStartLabel->setText("<font color='white'>" +
                                                  QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                                  QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                                  QString::number(floor(getSeconds(timer_start_time_in_seconds))) + " seconds</font>");
            emit sendInfoLogMessage("Setting experiment timer to stop at: " +
                                    QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_stop_time_in_seconds)) + " seconds\n");
            ui.simulationTimerStopLabel->setText("<font color='white'>" +
                                                 QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                                 QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                                 QString::number(floor(getSeconds(timer_stop_time_in_seconds))) + " seconds</font>");
            ui.simulation_timer_combobox->setEnabled(false);
            ui.simulation_timer_combobox->setStyleSheet("color: grey; border:2px solid grey;");
        }
    }

    // Experiment Timer END
}

double RoverGUIPlugin::getHours(double seconds) {
    if (seconds < 3600.0) return 0.0;

    double hours = 0.0;

    while (seconds >= 3600.0) {
        seconds -= 3600.0;
        hours++;
    }

    return hours;
}

double RoverGUIPlugin::getMinutes(double seconds) {
    double hours = getHours(seconds);
    seconds -= hours * 3600.0;

    if (seconds < 60.0) return 0.0;

    double minutes = 0.0;

    while (seconds >= 60.0) {
        seconds -= 60.0;
        minutes++;
    }

    return minutes;
}

double RoverGUIPlugin::getSeconds(double seconds) {
    double hours = getHours(seconds);
    double minutes = getMinutes(seconds);
    seconds -= (hours * 3600.0) + (minutes * 60.0);

    return seconds;
}

void RoverGUIPlugin::allStopButtonEventHandler()
{
    emit sendInfoLogMessage("changing all rovers to manual control...");

    std::string remember_selected_rover_name = selected_rover_name;
    int remember_selected_index = ui.rover_list->currentRow();
    int selected_index = -1; // zero array indexing, ensure last selected index is in range

    // manually trigger the manual radio button event for all rovers
    for (set<string>::iterator it = rover_names.begin(); it != rover_names.end(); it++)
    {
        selected_index++;
        selected_rover_name = *it;
        joystickRadioButtonEventHandler(true);
    }

    // trigger the current rover changed event:
    // if we previously selected a rover, keep that rover selected
    if (remember_selected_index >= 0)
    {
        selected_rover_name = remember_selected_rover_name;
        ui.rover_list->setCurrentItem(ui.rover_list->item(remember_selected_index));
    }
    // otherwise, default to the last rover in the rover list
    else
    {
        ui.rover_list->setCurrentItem(ui.rover_list->item(selected_index));
    }

    ui.joystick_control_radio_button->setEnabled(true);
    ui.autonomous_control_radio_button->setEnabled(true);
    ui.joystick_control_radio_button->setChecked(true);
    ui.autonomous_control_radio_button->setChecked(false);
    ui.joystick_frame->setHidden(false);
    
    //Disable all stop button
    ui.all_stop_button->setEnabled(false);
    ui.all_stop_button->setStyleSheet("color: grey; border:2px solid grey;");

    // reset the simulation timer variables
    ui.simulation_timer_combobox->setEnabled(true);
    ui.simulation_timer_combobox->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px");
    ui.simulationTimerStartLabel->setText("<font color='white'>---</font>");
    ui.simulationTimerStopLabel->setText("<font color='white'>---</font>");
    timer_start_time_in_seconds = 0.0;
    timer_stop_time_in_seconds = 0.0;

    if (is_timer_on == true) {
        is_timer_on = false;
        emit sendInfoLogMessage("\nSimulation timer cancelled at: " +
                                QString::number(getHours(current_simulated_time_in_seconds)) + " hours, " +
                                QString::number(getMinutes(current_simulated_time_in_seconds)) + " minutes, " +
                                QString::number(getSeconds(current_simulated_time_in_seconds)) + " seconds\n");
    }
}

// Get the path to the world file containing the custom distribution from the user
void RoverGUIPlugin::customWorldButtonEventHandler()
{
    const char *name = "SWARMATHON_APP_ROOT";
    char *app_root_cstr;
    app_root_cstr = getenv(name);
    QString app_root = QString(app_root_cstr) + "/simulation/worlds/";

    // NOTE: passing a parent widget here (aka, our "widget" variable) will style the dialog box
    //     in the same manner as the RQT rover GUI, currently with a black background and unreadable
    //     gray text; meanwhile, passing in a null value results in the default operating system
    //     style to be used
    QString path = QFileDialog::getOpenFileName(/*widget*/ NULL, tr("Open File"),
                                                    app_root,
                                                    tr("Gazebo World File (*.world)"));

    sim_mgr.setCustomWorldPath(path);
    emit sendInfoLogMessage("User selected custom world path: " + path);

    // Extract the base filename for short display
    QFileInfo fi=path;
    ui.custom_world_path->setText(fi.baseName());
}

// Enable or disable custom distributions
void RoverGUIPlugin::customWorldRadioButtonEventHandler(bool toggled)
{
    ui.custom_world_path_button->setEnabled(toggled);
    ui.number_of_tags_combobox->setEnabled(!toggled);

    // Set the button color to reflect whether or not it is disabled
    // Clear the sim path if custom distribution it deselected
    if(toggled)
    {
        ui.custom_world_path_button->setStyleSheet("color: white; border:2px solid white;");

        ui.number_of_tags_label->setStyleSheet("color: grey;");
        ui.number_of_tags_combobox->setStyleSheet("color: grey; border:2px solid grey; padding: 1px 0px 1px 3px");
    }
    else
    {
        sim_mgr.setCustomWorldPath("");
        ui.custom_world_path->setText("");
        ui.custom_world_path_button->setStyleSheet("color: grey; border:2px solid grey;");

        ui.number_of_tags_label->setStyleSheet("color: white;");
        ui.number_of_tags_combobox->setStyleSheet("color: white; border:2px solid white; padding: 1px 0px 1px 3px");
    }
}

// Currently, we cannot use the power law distribution with custon numbers of cubes.
// I.E., we always use 256 tags, so disable the option to change the number of cubes when
// generating a power law distribution. If we add dynamic power law distribution generation
// in the future this block can be removed.
void RoverGUIPlugin::powerlawDistributionRadioButtonEventHandler(bool toggled)
{
    ui.number_of_tags_combobox->setEnabled(!toggled);

    if(!toggled)
    {
        ui.number_of_tags_label->setStyleSheet("color: white;");
        ui.number_of_tags_combobox->setStyleSheet("color: white; border:2px solid white; padding: 1px 0px 1px 3px");
    }
    else
    {
        ui.number_of_tags_label->setStyleSheet("color: grey;");
        ui.number_of_tags_combobox->setStyleSheet("color: grey; border:2px solid grey; padding: 1px 0px 1px 3px");
    }
}

void RoverGUIPlugin::unboundedRadioButtonEventHandler(bool toggled)
{
    ui.unbounded_arena_size_combobox->setEnabled(toggled);

    if(toggled)
    {
        ui.unbounded_arena_size_label->setStyleSheet("color: white;");
        ui.unbounded_arena_size_combobox->setStyleSheet("color: white; border:2px solid white; padding: 1px 0px 1px 3px");
    }
    else
    {
        ui.unbounded_arena_size_label->setStyleSheet("color: grey;");
        ui.unbounded_arena_size_combobox->setStyleSheet("color: grey; border:2px solid grey; padding: 1px 0px 1px 3px");
    }
}

void RoverGUIPlugin::mapPopoutButtonEventHandler()
{
    ui.map_frame->popout();
}

void RoverGUIPlugin::buildSimulationButtonEventHandler()
{
    emit sendInfoLogMessage("Building simulation...");

    ui.build_simulation_button->setEnabled(false);
    ui.build_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");

    QString return_msg;

    if (sim_mgr.isGazeboServerRunning())
    {
        emit sendInfoLogMessage("A gazebo server simulation process is already running. Restart the Swarmathon GUI to clear.");
        return;
    }

    QProcess* sim_server_process = sim_mgr.startGazeboServer();
    connect(sim_server_process, SIGNAL(finished(int)), this, SLOT(gazeboServerFinishedEventHandler()));

    if (ui.final_radio_button->isChecked() && !ui.create_savable_world_checkbox->isChecked())
    {
         arena_dim = 23.1;
         addFinalsWalls();
         emit sendInfoLogMessage(QString("Set arena size to ")+QString::number(arena_dim)+"x"+QString::number(arena_dim));
    }
    else if (ui.prelim_radio_button->isChecked() && !ui.create_savable_world_checkbox->isChecked())
    {
        arena_dim = 15;
        addPrelimsWalls();
        emit sendInfoLogMessage(QString("Set arena size to ")+QString::number(arena_dim)+"x"+QString::number(arena_dim));
    }
    else
    {
        arena_dim = ui.unbounded_arena_size_combobox->currentText().toInt();
        emit sendInfoLogMessage(QString("Set arena size to ")+QString::number(arena_dim)+"x"+QString::number(arena_dim)+" with no barriers");
    }

    if(!ui.create_savable_world_checkbox->isChecked())
    {
        if (ui.texture_combobox->currentText() == "Gravel")
        {
            emit sendInfoLogMessage("Adding gravel ground plane...");
            return_msg = sim_mgr.addGroundPlane("mars_ground_plane");
            emit sendInfoLogMessage(return_msg);
        }
        else if (ui.texture_combobox->currentText() == "KSC Concrete")
        {
            emit sendInfoLogMessage("Adding concrete ground plane...");
            return_msg = sim_mgr.addGroundPlane("concrete_ground_plane");
            emit sendInfoLogMessage(return_msg);
        }
        else if (ui.texture_combobox->currentText() == "Car park")
        {
            emit sendInfoLogMessage("Adding carpark ground plane...");
            return_msg = sim_mgr.addGroundPlane("carpark_ground_plane");
            emit sendInfoLogMessage(return_msg);
        }
        else
        {
            emit sendInfoLogMessage("Unknown ground plane...");
        }
    }
    else
    {
        emit sendInfoLogMessage("Not using a ground plane texture...");
    }

    if(!ui.create_savable_world_checkbox->isChecked())
    {
        emit sendInfoLogMessage("Adding collection disk...");
        float collection_disk_radius = 0.5; // meters
        sim_mgr.addModel("collection_disk", "collection_disk", 0, 0, 0, collection_disk_radius);
        score_subscriber = nh.subscribe("/collectionZone/score", 10, &RoverGUIPlugin::scoreEventHandler, this);
        simulation_timer_subscriber = nh.subscribe("/clock", 10, &RoverGUIPlugin::simulationTimerEventHandler, this);
    }
    else
    {
        emit sendInfoLogMessage("Not adding collection disk...");
    }

    if(!ui.create_savable_world_checkbox->isChecked())
    {
        int n_rovers_created = 0;
        int n_rovers = 3;
        if (ui.final_radio_button->isChecked()) n_rovers = 6;

        // If the user chose to override the number of rovers to add to the simulation read the selected value
        // Please notice that this will override "n_rovers = 6" above if the final radio button is selected
        if (ui.override_num_rovers_checkbox->isChecked()) n_rovers = ui.custom_num_rovers_combobox->currentText().toInt();

        QProgressDialog progress_dialog;
        progress_dialog.setWindowTitle("Creating rovers");
        progress_dialog.setCancelButton(NULL); // no cancel button
        progress_dialog.setWindowModality(Qt::ApplicationModal);
        progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
        progress_dialog.resize(500, 50);
        progress_dialog.show();

        QString rovers[8] = {"achilles", "aeneas", "ajax", "diomedes", "hector", "paris", "thor", "zeus"};

        QColor rover_colors[8] = { /* green         */ QColor(  0, 255,   0),
                                   /* yellow        */ QColor(255, 255,   0),
                                   /* white         */ QColor(255, 255, 255),
                                   /* red           */ QColor(255,   0,   0),
                                   /* deep sky blue */ QColor(  0, 191, 255),
                                   /* hot pink      */ QColor(255, 105, 180),
                                   /* chocolate     */ QColor(210, 105,  30),
                                   /* indigo        */ QColor( 75,   0, 130) };

        /**
         * The distance to the rover from a corner position is calculated differently
         * than the distance to a cardinal position.
         *
         * The cardinal direction rovers are a straightforward calculation where:
         *     a = the distance to the edge of the collection zone
         *         i.e., 1/2 of the collection zone square side length
         *     b = the 50cm distance required by the rules for placing the rover
         *     c = offset for the simulation for the center of the rover (30cm)
         *         i.e., the rover position is at the center of its body
         *
         * The corner rovers use trigonometry to calculate the distance where each
         * value of d, e, and f, are the legs to an isosceles right triangle. In
         * other words, we are calculating and summing X and Y offsets to position
         * the rover.
         *     d = a
         *     e = xy offset to move the rover 50cm from the corner of the collection zone
         *     f = xy offset to move the rover 30cm to account for its position being
         *         calculated at the center of its body
         *
         *                       *  *          d = 0.508m
         *                     *      *        e = 0.354m
         *                   *          *    + f = 0.212m
         *                 *     /*     *    ------------
         *                 *    / | f *            1.072m
         *                   * /--| *
         *                    /* *
         *                   / | e
         *                  /--|
         *     *************
         *     *          /|
         *     *         / |
         *     *        /  | d                 a = 0.508m
         *     *       /   |     *********     b = 0.500m
         *     *      /    |     *       *   + c = 0.300m
         *     *     *-----|-----*---*   *   ------------
         *     *        a  *  b  * c     *         1.308m
         *     *           *     *********
         *     *           *
         *     *           *
         *     *           *
         *     *************
         */
        QPointF rover_positions[8] =
        {
          /* cardinal rovers: North, East, South, West */
          QPointF(-1.308,  0.000), // 1.308 = distance_from_center_to_edge_of_collection_zone
          QPointF( 0.000, -1.308), //             + 50 cm distance to rover
          QPointF( 1.308,  0.000), //             + 30 cm distance_from_center_of_rover_to_edge_of_rover
          QPointF( 0.000,  1.308), // 1.308m = 0.508m + 0.5m + 0.3m

          /* corner rovers: Northeast, Southwest */
          QPointF( 1.072,  1.072), // 1.072 = diagonal_distance_from_center_to_edge_of_collection_zone
          QPointF(-1.072, -1.072), //             + diagonal_distance_to_move_50cm
                                   //             + diagonal_distance_to_move_30cm
                                   // 1.072m = 0.508 + 0.354 + 0.212

          /* corner rovers: Northwest, Southeast */
          QPointF(-1.072,  1.072),
          QPointF( 1.072, -1.072)
        };

        /* In this case, the yaw is the value that turns rover "left" and "right" */
        float rover_yaw[8] =
        {
           0.000, //  0.00 * PI
           1.571, //  0.50 * PI
          -3.142, // -1.00 * PI
          -1.571, // -0.50 * PI
          -2.356, // -0.75 * PI
           0.785, //  0.25 * PI
          -0.785, // -0.25 * PI
           2.356  //  0.75 * PI
        };

        // Add rovers to the simulation and start the associated ROS nodes
        for (int i = 0; i < n_rovers; i++)
        {
            // add the global offset for sim rovers
            ui.map_frame->setGlobalOffsetForRover(rovers[i].toStdString(), rover_positions[i].x(), rover_positions[i].y());
            ui.map_frame->setUniqueRoverColor(rovers[i].toStdString(), rover_colors[i]);

            emit sendInfoLogMessage("Adding rover "+rovers[i]+"...");
            return_msg = sim_mgr.addRover(rovers[i], rover_positions[i].x(), rover_positions[i].y(), 0, 0, 0, rover_yaw[i]);
            emit sendInfoLogMessage(return_msg);

            emit sendInfoLogMessage("Starting rover node for "+rovers[i]+"...");
            return_msg = sim_mgr.startRoverNode(rovers[i]);
            emit sendInfoLogMessage(return_msg);

            progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
            qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

            if(i == 0)
            {
              sleep(rover_load_delay); // Gives plugins enough time to finish loading
            }
        }
    }
    else
    {
        emit sendInfoLogMessage("Not creating rovers...");
    }

    if (ui.powerlaw_distribution_radio_button->isChecked())
    {
       emit sendInfoLogMessage("Adding powerlaw distribution of targets...");
       return_msg = addPowerLawTargets();
       emit sendInfoLogMessage(return_msg);
    }
    else if (ui.uniform_distribution_radio_button->isChecked())
    {
       emit sendInfoLogMessage("Adding uniform distribution of targets...");
       return_msg = addUniformTargets();
       emit sendInfoLogMessage(return_msg);
    }
    else if (ui.clustered_distribution_radio_button->isChecked())
    {
       emit sendInfoLogMessage("Adding clustered distribution of targets...");
       return_msg = addClusteredTargets();
       emit sendInfoLogMessage(return_msg);
    }

    // add walls given nw corner (x,y) and height and width (in meters)

    //addWalls(-arena_dim/2, -arena_dim/2, arena_dim, arena_dim);

    //   // Test rover movement
    //   displayLogMessage("Moving aeneas");
    //   return_msg = sim_mgr.moveRover("aeneas", 10, 0, 0);
    //   displayLogMessage(return_msg);

    //displayLogMessage("Starting the gazebo client to visualize the simulation.");
    //sim_mgr.startGazeboClient();

    ui.visualize_simulation_button->setEnabled(true);
    ui.clear_simulation_button->setEnabled(true);

    ui.visualize_simulation_button->setStyleSheet("color: white;border:1px solid white;");
    ui.clear_simulation_button->setStyleSheet("color: white;border:1px solid white;");

    ui.simulation_timer_combobox->setEnabled(true);
    ui.simulation_timer_combobox->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px");

    emit sendInfoLogMessage("Finished building simulation.");

    if (ui.start_visualization_on_build_checkbox->isChecked())
    {
        // Visualize the simulation by default call button event handler
        visualizeSimulationButtonEventHandler();
        display_sim_visualization = true;
    }
    else
    {
        display_sim_visualization = false;
    }
}

void RoverGUIPlugin::clearSimulationButtonEventHandler()
{
    if (!sim_mgr.isGazeboServerRunning())
    {
        emit sendInfoLogMessage("Simulation is not running.");

        return;
    }

    emit sendInfoLogMessage("Ending simulation...");

    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Shutting Down Rovers");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    QString return_msg;
    float count = 0.0f;

    // Make a copy of the rover names because stopRoverNode will cause the original set to change
    set<string> rover_names_copy = rover_names;

    for(set<string>::const_iterator i = rover_names_copy.begin(); i != rover_names_copy.end(); ++i)
    {
        return_msg += sim_mgr.stopRoverNode(QString::fromStdString(*i));
        return_msg += "<br>";
        progress_dialog.setValue((++count)*100.0f/rover_names_copy.size());
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    }

    // Unsubscribe from topics

    emit sendInfoLogMessage("Shutting down subscribers...");

    for (map<string,ros::Subscriber>::iterator it=encoder_subscribers.begin(); it!=encoder_subscribers.end(); ++it) 
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }

    encoder_subscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=gps_subscribers.begin(); it!=gps_subscribers.end(); ++it) {
      qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
      it->second.shutdown();
    }

    for (map<string,ros::Subscriber>::iterator it=gps_nav_solution_subscribers.begin(); it!=gps_nav_solution_subscribers.end(); ++it) {
      qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
      it->second.shutdown();
    }

    gps_subscribers.clear();
    gps_nav_solution_subscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=ekf_subscribers.begin(); it!=ekf_subscribers.end(); ++it) {
      qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
      it->second.shutdown();
    }

    ekf_subscribers.clear();
    us_center_subscriber.shutdown();
    us_left_subscriber.shutdown();
    us_right_subscriber.shutdown();
    imu_subscriber.shutdown();

    // Possible error - the following seems to shutdown all subscribers not just those from simulation

    for (map<string,ros::Subscriber>::iterator it=status_subscribers.begin(); it!=status_subscribers.end(); ++it) 
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }
    status_subscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=waypoint_subscribers.begin(); it!=waypoint_subscribers.end(); ++it) 
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }
    waypoint_subscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=obstacle_subscribers.begin(); it!=obstacle_subscribers.end(); ++it) 
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }

    obstacle_subscribers.clear();
    score_subscriber.shutdown();
    simulation_timer_subscriber.shutdown();
    camera_subscriber.shutdown();

    emit sendInfoLogMessage("Shutting down publishers...");

    for (map<string,ros::Publisher>::iterator it=control_mode_publishers.begin(); it!=control_mode_publishers.end(); ++it)
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }
    control_mode_publishers.clear();

    for (map<string,ros::Publisher>::iterator it=waypoint_cmd_publishers.begin(); it!=waypoint_cmd_publishers.end(); ++it)
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }
    waypoint_cmd_publishers.clear();
    
    
    return_msg += sim_mgr.stopGazeboClient();
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    return_msg += "<br>";
    return_msg += sim_mgr.stopGazeboServer();
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    emit sendInfoLogMessage(return_msg);

    ui.visualize_simulation_button->setEnabled(false);
    ui.build_simulation_button->setEnabled(true);
    ui.clear_simulation_button->setEnabled(false);
    display_sim_visualization = false;


    ui.build_simulation_button->setStyleSheet("color: white; border:1px solid white;");
    ui.visualize_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.clear_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");

    // reset waypoints
    ui.map_frame->resetAllWaypointPaths();

    // Clear the task status values
    obstacle_call_count = 0;
    emit updateObstacleCallCount("<font color='white'>0</font>");
    emit updateNumberOfTagsCollected("<font color='white'>0</font>");
    emit updateNumberOfSatellites("<font color='white'>---</font>");

    // reset the simulation timer variables
    ui.simulation_timer_combobox->setEnabled(true);
    ui.simulation_timer_combobox->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px");
    ui.simulationTimerStartLabel->setText("<font color='white'>---</font>");
    ui.simulationTimerStopLabel->setText("<font color='white'>---</font>");
    ui.currentSimulationTimeLabel->setText("<font color='white'>---</font>");
    timer_start_time_in_seconds = 0.0;
    timer_stop_time_in_seconds = 0.0;
    current_simulated_time_in_seconds = 0.0;
    last_current_time_update_in_seconds = 0.0;
    is_timer_on = false;
}

void RoverGUIPlugin::visualizeSimulationButtonEventHandler()
{
    if (!sim_mgr.isGazeboServerRunning())
    {
        emit sendInfoLogMessage("Simulation is not running.");

        return;
    }

    QString return_msg;
    // toggle visualize or not
    display_sim_visualization = !display_sim_visualization;

    if (display_sim_visualization)
    {
        emit sendInfoLogMessage("Visualizing simulation...");

        QProcess* sim_client_process = sim_mgr.startGazeboClient();
    }
    else
    {
        emit sendInfoLogMessage("Ending visualization...");

        return_msg = sim_mgr.stopGazeboClient();
        emit sendInfoLogMessage(return_msg);
    }

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
    QString number_of_tags = ui.number_of_tags_combobox->currentText();

    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing " + number_of_tags + " Targets");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.resize(500, 50);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.show();

    QString output;

    float proposed_x;
    float proposed_y;

    // d is the distance from the center of the arena to the boundary minus the barrier clearance, i.e. the region where tags can be placed
    // is d - U(0,2d) where U(a,b) is a uniform distribition bounded by a and b.
    // (before checking for collisions including the collection disk at the center)
    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_1_clearance);

    progress_dialog.setValue(0.0);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    for (int i = 0; i < number_of_tags.toInt(); i++)
    {
        do
        {
            emit sendInfoLogMessage("Tried to place target "+QString::number(0)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y) + "...");
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_1_clearance));

        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
        output = sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_1_clearance);
        progress_dialog.setValue(i*100.0f/number_of_tags.toInt());
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    }

    emit sendInfoLogMessage("Placed " + number_of_tags + " single targets");

    return output;
}

QString RoverGUIPlugin::addClusteredTargets()
{
    QString number_of_tags = ui.number_of_tags_combobox->currentText();
    int cluster_length = 0;
    int cluster_width = 0;

    switch(number_of_tags.toInt())
    {
        case 256:
            cluster_length = 8;
            cluster_width = 8;
            break;
        case 128:
            cluster_length = 8;
            cluster_width = 4;
            break;
        case 64:
            cluster_length = 4;
            cluster_width = 4;
            break;
        case 32:
            cluster_length = 4;
            cluster_width = 2;
            break;
        case 16:
            cluster_length = 2;
            cluster_width = 2;
            break;
        case 0:
            cluster_length = 0;
            cluster_width = 0;
        default:
            cluster_length = 1;
            cluster_width = 1;
    }

    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing " + number_of_tags + " Targets into four " + QString::number(cluster_length) + " x " + QString::number(cluster_width) + " clusters");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    QString output;

    float proposed_x, proposed_x2;
    float proposed_y, proposed_y2;

    float target_cluster_clearance = target_cluster_size_1_clearance * ((cluster_length > cluster_width) ? (cluster_length) : (cluster_width));
    float d = arena_dim/2.0-(barrier_clearance+target_cluster_clearance);
    int cube_index = 0;

    progress_dialog.setValue(0.0);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    // Four piles
    for (int i = 0; i < 4; i++)
    {
        do
        {
            emit sendInfoLogMessage("Tried to place cluster "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_clearance));

        proposed_y2 = proposed_y - (target_cluster_size_1_clearance * cluster_length);

        for(int j = 0; j < cluster_length; j++) {
            proposed_x2 = proposed_x - (target_cluster_size_1_clearance * cluster_width);

            for(int k = 0; k < cluster_width; k++) {
                output += sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(cube_index), proposed_x2, proposed_y2, 0, target_cluster_size_1_clearance);
                proposed_x2 += target_cluster_size_1_clearance;
                cube_index++;
                progress_dialog.setValue(cube_index*100.0f/256);
                qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
            }

            proposed_y2 += target_cluster_size_1_clearance;
        }

        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
    }

    emit sendInfoLogMessage("Placed four " + QString::number(cluster_length) + " x " + QString::number(cluster_width) + " clusters of targets");

    return output;
}

QString RoverGUIPlugin::addPowerLawTargets()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing 256 Targets into 85 Clusters (Power Law pattern)");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();


    float total_number_of_clusters = 85;
    float clusters_placed = 0;
    int cube_index = 0;

    QString output = "";

    float proposed_x, proposed_x2;
    float proposed_y, proposed_y2;

    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_64_clearance);

    progress_dialog.setValue(0.0);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    // One pile of 64
    do
    {
        emit sendInfoLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
        proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
    }
    while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_64_clearance));

    proposed_y2 = proposed_y - (target_cluster_size_1_clearance * 8);

    for(int j = 0; j < 8; j++) {
        proposed_x2 = proposed_x - (target_cluster_size_1_clearance * 8);

        for(int k = 0; k < 8; k++) {
            output += sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(cube_index), proposed_x2, proposed_y2, 0, target_cluster_size_1_clearance);
            proposed_x2 += target_cluster_size_1_clearance;
            cube_index++;
            progress_dialog.setValue(cube_index*100.0f/256);
            qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        }

        proposed_y2 += target_cluster_size_1_clearance;
    }

    emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
    d = arena_dim/2.0-(barrier_clearance+target_cluster_size_16_clearance);

    // Four piles of 16
    for (int i = 0; i < 4; i++)
    {
        do
        {
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
            emit sendInfoLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_16_clearance));

        proposed_y2 = proposed_y - (target_cluster_size_1_clearance * 4);

        for(int j = 0; j < 4; j++) {
            proposed_x2 = proposed_x - (target_cluster_size_1_clearance * 4);

            for(int k = 0; k < 4; k++) {
                output += sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(cube_index), proposed_x2, proposed_y2, 0, target_cluster_size_1_clearance);
                proposed_x2 += target_cluster_size_1_clearance;
                cube_index++;
                progress_dialog.setValue(cube_index*100.0f/256);
                qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
            }

            proposed_y2 += target_cluster_size_1_clearance;
        }

        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
    }

    d = arena_dim/2.0-(barrier_clearance+target_cluster_size_4_clearance);

    // Sixteen piles of 4
    for (int i = 0; i < 16; i++)
    {
        do
        {
            emit sendInfoLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_4_clearance));

        proposed_y2 = proposed_y - (target_cluster_size_1_clearance * 2);

        for(int j = 0; j < 2; j++) {
            proposed_x2 = proposed_x - (target_cluster_size_1_clearance * 2);

            for(int k = 0; k < 2; k++) {
                output += sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(cube_index), proposed_x2, proposed_y2, 0, target_cluster_size_1_clearance);
                proposed_x2 += target_cluster_size_1_clearance;
                cube_index++;
                progress_dialog.setValue(cube_index*100.0f/256);
                qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
            }

            proposed_y2 += target_cluster_size_1_clearance;
        }

        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
    }

    d = arena_dim/2.0-(barrier_clearance+target_cluster_size_1_clearance);

    // Sixty-four piles of 1 (using tags 192 through 255 to avoid duplication with piles above)
    for (int i = 192; i < 256; i++)
    {
        do
        {
            emit sendInfoLogMessage("Tried to place target "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_1_clearance));

        progress_dialog.setValue(i*100.0f/256);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
        output+= sim_mgr.addModel(QString("at")+QString::number(0), QString("at")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_1_clearance);
    }

    return output;
}

// Add a cinder block wall to the simulation
QString RoverGUIPlugin::addFinalsWalls()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing Barriers");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    QString output;

    // Setting wall clearance to zero - radius of a wall does not make sense. Barrier clearance values ensure models are not placed on the walls.
    output += sim_mgr.addModel("barrier_final_round", "Barrier_West", -arena_dim/2, 0, 0, 0 );
    progress_dialog.setValue(1*100.0f/4);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_final_round", "Barrier_North", 0, -arena_dim/2, 0, 0, 0, M_PI/2, 0);
       progress_dialog.setValue(2*100.0f/4);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_final_round", "Barrier_East", arena_dim/2, 0, 0, 0 );
       progress_dialog.setValue(3*100.0f/4);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_final_round", "Barrier_South", 0, arena_dim/2, 0, 0, 0, M_PI/2, 0);
       progress_dialog.setValue(4*100.0f/4);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   return output;
}

QString RoverGUIPlugin::addPrelimsWalls()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing Barriers");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    // Setting wall clearance to zero - radius of a wall does not make sense. Barrier clearance values ensure models are not placed on the walls.

   QString output;
   output += sim_mgr.addModel("barrier_prelim_round", "Barrier_West", -arena_dim/2, 0, 0, 0 );
   progress_dialog.setValue(1*100.0f/4);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_prelim_round", "Barrier_North", 0, -arena_dim/2, 0, 0, 0, M_PI/2, 0);
   progress_dialog.setValue(2*100.0f/4);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_prelim_round", "Barrier_East", arena_dim/2, 0, 0, 0 );
   progress_dialog.setValue(3*100.0f/4);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_prelim_round", "Barrier_South", 0, arena_dim/2, 0, 0, 0, M_PI/2, 0);
   progress_dialog.setValue(4*100.0f/4);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   return output;
}

void RoverGUIPlugin::checkAndRepositionRover(QString rover_name, float x, float y)
{
    // Currently disabled.
    return;

    float arena_dim = 20;

    if (x < -arena_dim/2)
    {
        float duration = 10; //seconds
        float x_comp, y_comp, z_comp;
        x_comp =
        z_comp = 0;
        y_comp = 0;
        emit sendInfoLogMessage("Moving rover back into the arena");
        QString return_msg = sim_mgr.moveRover(rover_name, x_comp, y, 0);
        emit sendInfoLogMessage(return_msg);
    }
}

void RoverGUIPlugin::readRoverModelXML(QString path)
{
    ifstream model_file;
    model_file.open(path.toStdString(), ios::in);
    if (model_file.is_open())
        emit sendInfoLogMessage("Read model file at " + path );
    else
    {
        emit sendInfoLogMessage(QString::fromStdString(selected_rover_name) + " appears to be a physical rover.");
        return;
    }

    ptree property_tree;
    read_xml(model_file, property_tree);

    BOOST_FOREACH( ptree::value_type const& v, property_tree.get_child("sdf.model") )
    {
        if (v.first == "link")
        {
            BOOST_FOREACH( ptree::value_type const& w, v.second )
            {
                if (w.first == "sensor")
                {
                    BOOST_FOREACH( ptree::value_type const& x, w.second )
                    {
                        if ( x.first == "ray" )
                        {
                            BOOST_FOREACH( ptree::value_type const& y, x.second.get_child("scan.horizontal") )
                            {
                                if (y.first == "samples")
                                {
                                    //ui.sonar_horz_res->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "resolution")
                                {
                                    ui.sonar_horz_res->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "min_angle")
                                {
                                    ui.sonar_min_angle->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "max_angle")
                                {
                                    ui.sonar_max_angle->setText( QString::fromStdString(y.second.data()) );
                                }
                            }

                            BOOST_FOREACH( ptree::value_type const& y, x.second.get_child("range") )
                            {
                                if (y.first == "min")
                                {
                                    ui.sonar_min->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "max")
                                {
                                    ui.sonar_max->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "resolution")
                                {
                                    ui.sonar_range_res->setText( QString::fromStdString(y.second.data()) );
                                }
                            }
                        }
                        else if ( x.first == "plugin" )
                        {

                            BOOST_FOREACH( ptree::value_type const& y, x.second )
                            {
                                if (y.first == "gaussianNoise")
                                {
                                    ui.sonar_gaussian_noise->setText( QString::fromStdString(y.second.data()) );
                                }
                            }
                        }
                        else if ( x.first == "camera" )
                        {
                            BOOST_FOREACH( ptree::value_type const& x, w.second )
                            {
                                if (x.first == "update_rate")
                                ui.camera_update_rate->setText( QString::fromStdString(x.second.data()) );
                            }

                            BOOST_FOREACH( ptree::value_type const& y, x.second )
                            {
                                if (y.first == "noise")
                                {
                                    BOOST_FOREACH( ptree::value_type const& z, y.second )
                                    {
                                         if (z.first == "mean") ui.camera_noise_mean->setText( QString::fromStdString(z.second.data()) );
                                         else if (z.first == "stddev") ui.camera_noise_stdev->setText( QString::fromStdString(z.second.data()) );
                                    }
                                }
                                else if (y.first == "image")
                                {
                                    BOOST_FOREACH( ptree::value_type const& z, y.second )
                                    {
                                         if (z.first == "width") ui.camera_width->setText( QString::fromStdString(z.second.data()) );
                                         else if (z.first == "height") ui.camera_height->setText( QString::fromStdString(z.second.data()) );
                                         else if (z.first == "format") ui.camera_format->setText( QString::fromStdString(z.second.data()) );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        else if (v.first == "plugin")
        {

            BOOST_FOREACH( ptree::value_type const& w, v.second.get_child("<xmlattr>"))
            {
                 if (w.first == "name")
                    if (w.second.data() == "imu_sim")
                    {
                        BOOST_FOREACH( ptree::value_type const& x, v.second)
                        {
                            if (x.first == "updateRate") ui.imu_update_rate->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "rpyOffsets") ui.imu_rpy_offsets->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "gaussianNoise") ui.imu_noise->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "accelDrift") ui.imu_accel_drift->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "accelGaussianNoise") ui.imu_accel_noise->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "rateDrift") ui.imu_rate_drift->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "rateGaussianNoise") ui.imu_rate_noise->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "headingDrift") ui.imu_heading_drift->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "headingGaussianNoise") ui.imu_heading_noise->setText(QString::fromStdString(x.second.data()));
                        }
                    }
                 else if (w.second.data() == "gps_sim")
                    {
                        BOOST_FOREACH( ptree::value_type const& x, v.second)
                        {
                             if (x.first == "updateRate") ui.gps_update_rate->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "referenceLatitude") ui.gps_ref_lat->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "referenceLongitude") ui.gps_ref_long->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "referenceAltitude") ui.gps_ref_alt->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "referenceHeading") ui.gps_ref_heading->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "drift") ui.gps_drift->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "driftFrequency") ui.gps_drift_freq->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "gaussianNoise") ui.gps_noise->setText(QString::fromStdString(x.second.data()));

                        }
                    }
                }

            }
        else
        {

        }
    }

//    QDomElement root = xml_doc.documentElement();

//    QString gps_reference_lat = root.attribute("referenceLatitude");
//    QString gps_reference_long = root.attribute("referenceLongitude");
//    QString gps_reference_heading = root.attribute("referenceHeading");
//    QString gps_reference_altitude = root.attribute("referenceAltitude");
//    QString gps_offset = root.attribute("offset");
//    QString gps_drift = root.attribute("drift");
//    QString gps_drift_frequency = root.attribute("driftFrequency");
//    QString gps_gaussian_noise = root.attribute("gaussianNoise");

//    QString imu_update_rate = root.attribute("updateRate");
//    QString imu_rpy_offsets = root.attribute("rpyOffsets");
//    QString imu_gaussian_noise = root.attribute("gaussianNoise");
//    QString imu_accel_drift = root.attribute("accelDrift");
//    QString imu_accel_gaussian_noise = root.attribute("accelGaussianNoise");
//    QString imu_rate_drift = root.attribute("rateDrift");
//    QString imu_rate_gaussian_noise = root.attribute("rateGaussianNoise");
//    QString imu_heading_drift = root.attribute("headingDrift");
//    QString imu_heading_gaussian_noise = root.attribute("headingGaussianNoise");

//    QString camera_update_rate = root.attribute("update_rate");
//    QString camera_horizontal_fov = root.attribute("horizontal_fov");
//    QString camera_width = root.attribute("width");
//    QString camera_height = root.attribute("height");
//    QString camera_format = root.attribute("format");
//    QString camera_clip_near = root.attribute("near");
//    QString camera_clip_far = root.attribute("far");
//    QString camera_noise_type = root.attribute("type");
//    QString camera_noise_mean = root.attribute("mean");
//    QString camera_noise_stddev = root.attribute("stddev");

//    QString sonar_noise_mean = root.attribute("samples");
//    QString sonar_horz_resolution = root.attribute("resolution");
//    QString sonar_min_angle = root.attribute("min_angle");
//    QString sonar_max_angle = root.attribute("max_angle");
//    QString sonar_min = root.attribute("min");
//    QString sonar_max = root.attribute("max");
//    QString sonar_range_resolution = root.attribute("resolution");

    //cout << "GPS Ref. Lat. " << gps_reference_lat.toStdString() << endl;

}

void RoverGUIPlugin::gazeboServerFinishedEventHandler()
{

    emit sendInfoLogMessage("Gazebo client exited");

    ui.visualize_simulation_button->setEnabled(false);
    ui.clear_simulation_button->setEnabled(false);
    ui.build_simulation_button->setEnabled(true);

    ui.visualize_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.clear_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.build_simulation_button->setStyleSheet("color: white; border:1px solid white;");
}

bool RoverGUIPlugin::eventFilter(QObject *target, QEvent *event)
{
    sensor_msgs::Joy joy_msg;
    joy_msg.axes = {0.0,0.0,0.0,0.0,0.0,0.0};
    
    if (joystick_publisher)
    {

    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

            bool direction_key = true;

            float speed = 1;

            // displayLogMessage("Key press");

            switch( keyEvent->key() )
            {
            case Qt::Key_I:
                joy_msg.axes[4] = speed;
                ui.joy_lcd_drive_forward->display(speed);
                break;
            case Qt::Key_K:
                joy_msg.axes[4] = -speed;
                ui.joy_lcd_drive_back->display(speed);
                break;
            case Qt::Key_J:
                joy_msg.axes[3] = speed;
                ui.joy_lcd_drive_left->display(speed);
                break;
            case Qt::Key_L:
                joy_msg.axes[3] = -speed;
                ui.joy_lcd_drive_right->display(speed);
                break;
            default:
                // Not a direction key so ignore
                direction_key = false;
            }

            if (direction_key )
            {
                joystick_publisher.publish(joy_msg);
                return true;
            }
        }

        // Stop the rover when key is released
        if (event->type() == QEvent::KeyRelease)
        {
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

            // Don't process auto repeat release events. Just the actual key release.
            if (keyEvent->isAutoRepeat())
            {
                // displayLogMessage("Ignoring auto repeat release.");
                return rqt_gui_cpp::Plugin::eventFilter(target, event);
            }

            // displayLogMessage("Key release");

            if (keyEvent->key() == Qt::Key_I || keyEvent->key() == Qt::Key_J || keyEvent->key() == Qt::Key_K || keyEvent->key() == Qt::Key_L )
            {
                joy_msg.axes[4] = 0;
                joy_msg.axes[3] = 0;
                ui.joy_lcd_drive_forward->display(0);
                ui.joy_lcd_drive_back->display(0);
                ui.joy_lcd_drive_left->display(0);
                ui.joy_lcd_drive_right->display(0);

                joystick_publisher.publish(joy_msg);
                return true;

            }
            else
            {
                return rqt_gui_cpp::Plugin::eventFilter(target, event);
            }
        }
    }
        // Pass on the event since it wasn't handled by us
    return rqt_gui_cpp::Plugin::eventFilter(target, event);
}

void RoverGUIPlugin::receiveInfoLogMessage(QString msg)
{
    displayInfoLogMessage(msg);
}


void RoverGUIPlugin::receiveDiagLogMessage(QString msg)
{
    displayDiagLogMessage(msg);
}

void RoverGUIPlugin::infoLogMessageEventHandler(const ros::MessageEvent<std_msgs::String const>& event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const boost::shared_ptr<const std_msgs::String> msg = event.getMessage();

    string log_msg = msg->data;

    emit sendInfoLogMessage(QString::fromStdString(publisher_name)
                           + " <font color=Lime size=1>"
                           + QString::fromStdString(log_msg)
                           + "</font>");
}

void RoverGUIPlugin::diagLogMessageEventHandler(const ros::MessageEvent<std_msgs::String const>& event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const boost::shared_ptr<const std_msgs::String> msg = event.getMessage();

    string log_msg = msg->data;

    emit sendDiagLogMessage(QString::fromStdString(log_msg));
}

void RoverGUIPlugin::overrideNumRoversCheckboxToggledEventHandler(bool checked)
{
    ui.custom_num_rovers_combobox->setEnabled(checked);
    if (checked) ui.custom_num_rovers_combobox->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px"); // The padding makes the item list color change work
    else ui.custom_num_rovers_combobox->setStyleSheet("color: grey; border:1px solid grey;");
}

void RoverGUIPlugin::createSavableWorldCheckboxToggledEventHandler(bool checked)
{
    ui.round_type_button_group->setEnabled(!checked);
    ui.custom_num_rovers_combobox->setEnabled(!checked);
    ui.override_num_rovers_checkbox->setEnabled(!checked);
    ui.ground_texture_label->setEnabled(!checked);
    ui.texture_combobox->setEnabled(!checked);
    ui.simulation_timer_label->setEnabled(!checked);
    ui.simulation_timer_combobox->setEnabled(!checked);

    ui.prelim_radio_button->click();
    unboundedRadioButtonEventHandler(false);

    // change specific GUI elements to the "disabled" color scheme
    if(checked)
    {
        ui.round_type_button_group->setStyleSheet("color: grey;");
        ui.prelim_radio_button->setStyleSheet("color: grey;");
        ui.final_radio_button->setStyleSheet("color: grey;");
        ui.unbounded_radio_button->setStyleSheet("color: grey;");
        ui.custom_num_rovers_combobox->setStyleSheet("color: grey; border:1px solid grey; padding: 1px 0px 1px 3px;");
        ui.override_num_rovers_checkbox->setStyleSheet("clor: grey;");
        ui.ground_texture_label->setStyleSheet("color: grey;");
        ui.texture_combobox->setStyleSheet("color: grey; border:1px solid grey; padding: 1px 0px 1px 3px;");
        ui.simulation_timer_label->setStyleSheet("color: grey;");
        ui.simulation_timer_combobox->setStyleSheet("color: grey; border:1px solid grey; padding: 1px 0px 1px 3px;");
    }
    // change specific GUI elements to the "enabled" color scheme
    else
    {
        ui.round_type_button_group->setStyleSheet("color: white;");
        ui.prelim_radio_button->setStyleSheet("color: white;");
        ui.final_radio_button->setStyleSheet("color: white;");
        ui.unbounded_radio_button->setStyleSheet("color: white;");
        ui.custom_num_rovers_combobox->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px;");
        ui.override_num_rovers_checkbox->setStyleSheet("color: white;");
        ui.ground_texture_label->setStyleSheet("color: white");
        ui.texture_combobox->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px;");
        ui.simulation_timer_label->setStyleSheet("color: white;");
        ui.simulation_timer_combobox->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px;");
    }
}

// Slot used to update the GUI diagnostic data output. Ensures we update from the correct process.
void RoverGUIPlugin::receiveDiagsDataUpdate(QString rover_name, QString text, QColor colour)
{
    if (!diag_update_mutex.try_lock()) return;

    // Find the row in the rover list that corresponds to the rover that sent us the diagnostics message
    // this is just to make sure the diagnostic data is displayed in the row that matches the rover
    // it came from
    int row = 0; // declare here so we can use it to index into the rover_diags_list
    for(; row < ui.rover_list->count(); row++)
    {
        QListWidgetItem *item = ui.rover_list->item(row);

        // Extract rover name
        string rover_name_and_status = item->text().toStdString();

        // Rover names start at the begining of the rover name and status string and end at the first space
        size_t rover_name_length = rover_name_and_status.find_first_of(" ");
        string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);
        if (ui_rover_name.compare(rover_name.toStdString())==0) break; // We found a rover with the right name
    }

    // Check the the rover was found in the rover list
    if (row >= ui.rover_list->count())
    {
        emit sendInfoLogMessage("Received diagnostic data from an unknown rover: " + rover_name);
        return;
    }

    // Update the UI - needs to happen in the UI thread
    QListWidgetItem *item = ui.rover_diags_list->item(row);

    // We don't want the user to interact with this display item so make non-selectable
    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);

    // Set the text and colour
    item->setText(text);
    item->setTextColor(colour);

    diag_update_mutex.unlock();
}


// Refocus on the main ui widget so the rover list doesn't start capturing key strokes making keyboard rover driving not work.
void RoverGUIPlugin::refocusKeyboardEventHandler()
{
    widget->setFocus();
}

// Publish the waypoint commands recieved from MapFrame to ROS
void RoverGUIPlugin::receiveWaypointCmd(WaypointCmd cmd, int id, float x, float y)
{
    std::set<std::string>::iterator it = rover_names.find(selected_rover_name);

    if(it == rover_names.end())
    {
      emit sendInfoLogMessage("Waypoints Error: a valid rover is not selected!");
      return;
    }

    swarmie_msgs::Waypoint msg;
    msg.action = cmd;
    msg.id = id;
    msg.x = x;
    msg.y = y;
    
    waypoint_cmd_publishers[selected_rover_name].publish(msg);
}

// Clean up memory when this object is deleted
RoverGUIPlugin::~RoverGUIPlugin()
{
    if (map_data) delete map_data;
    delete joystickGripperInterface;
}

} // End namespace



PLUGINLIB_EXPORT_CLASS(rqt_rover_gui::RoverGUIPlugin, rqt_gui_cpp::Plugin)

