// Author: Matthew Fricke
// E-mail: matthew@fricke.co.uk
// Date: 9-16-205
// Purpose: implementation of a simple graphical front end for the UNM-NASA Swarmathon rovers.
// License: GPL3

#include <rover_gui_plugin.h>
#include <Version.h>
#include <pluginlib/class_list_macros.h>
#include <QDir>
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
#include <QFileDialog>
#include <QComboBox>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <algorithm>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

//#include <regex> // For regex expressions

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

using namespace std;

using boost::property_tree::ptree;

namespace rqt_rover_gui 
{
  RoverGUIPlugin::RoverGUIPlugin() : rqt_gui_cpp::Plugin(), widget(0)
  {
    setObjectName("RoverGUI");
    info_log_messages = "";
    diag_log_messages = "";

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

    // Initialize AprilTag detection apparatus
	tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // Allocate image memory up front so it doesn't need to be done for every image frame
    u8_image = image_u8_create(320, 240);
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

    widget->setWindowTitle("Rover Interface: Built on " + BUILD_TIME );

    string rover_name_msg = "<font color='white'>Rover: " + selected_rover_name + "</font>";
    QString rover_name_msg_qstr = QString::fromStdString(rover_name_msg);
    ui.rover_name->setText(rover_name_msg_qstr);

    // Setup QT message connections
    connect(ui.rover_list, SIGNAL(currentItemChanged(QListWidgetItem*,QListWidgetItem*)), this, SLOT(currentRoverChangedEventHandler(QListWidgetItem*,QListWidgetItem*)));
    connect(ui.rover_list, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(refocusKeyboardEventHandler()));
    connect(ui.rover_list, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(refocusKeyboardEventHandler()));
    connect(ui.ekf_checkbox, SIGNAL(toggled(bool)), this, SLOT(EKFCheckboxToggledEventHandler(bool)));
    connect(ui.gps_checkbox, SIGNAL(toggled(bool)), this, SLOT(GPSCheckboxToggledEventHandler(bool)));
    connect(ui.encoder_checkbox, SIGNAL(toggled(bool)), this, SLOT(encoderCheckboxToggledEventHandler(bool)));
    connect(ui.autonomous_control_radio_button, SIGNAL(toggled(bool)), this, SLOT(autonomousRadioButtonEventHandler(bool)));
    connect(ui.joystick_control_radio_button, SIGNAL(toggled(bool)), this, SLOT(joystickRadioButtonEventHandler(bool)));
    connect(ui.all_autonomous_button, SIGNAL(pressed()), this, SLOT(allAutonomousButtonEventHandler()));
    connect(ui.all_stop_button, SIGNAL(pressed()), this, SLOT(allStopButtonEventHandler()));
    connect(ui.build_simulation_button, SIGNAL(pressed()), this, SLOT(buildSimulationButtonEventHandler()));
    connect(ui.clear_simulation_button, SIGNAL(pressed()), this, SLOT(clearSimulationButtonEventHandler()));
    connect(ui.visualize_simulation_button, SIGNAL(pressed()), this, SLOT(visualizeSimulationButtonEventHandler()));

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

    connect(this, SIGNAL(updateObstacleCallCount(QString)), ui.perc_of_time_avoiding_obstacles, SLOT(setText(QString)));

    connect(this, SIGNAL(sendInfoLogMessage(QString)), this, SLOT(receiveInfoLogMessage(QString)));
    connect(this, SIGNAL(sendDiagLogMessage(QString)), this, SLOT(receiveDiagLogMessage(QString)));
    connect(ui.custom_world_path_button, SIGNAL(pressed()), this, SLOT(customWorldButtonEventHandler()));
    connect(ui.custom_distribution_radio_button, SIGNAL(toggled(bool)), this, SLOT(customWorldRadioButtonEventHandler(bool)));

    // Create a subscriber to listen for joystick events
    joystick_subscriber = nh.subscribe("/joy", 1000, &RoverGUIPlugin::joyEventHandler, this);

    emit sendInfoLogMessage("Searching for rovers...");

    // Add discovered rovers to the GUI list
    rover_poll_timer = new QTimer(this);
    connect(rover_poll_timer, SIGNAL(timeout()), this, SLOT(pollRoversTimerEventHandler()));
    rover_poll_timer->start(5000);

    // Setup the initial display parameters for the map
    ui.map_frame->setDisplayGPSData(ui.gps_checkbox->isChecked());
    ui.map_frame->setDisplayEncoderData(ui.encoder_checkbox->isChecked());
    ui.map_frame->setDisplayEKFData(ui.ekf_checkbox->isChecked());

    ui.joystick_frame->setHidden(false);

    ui.custom_world_path_button->setDisabled(true);
    ui.custom_world_path_button->setStyleSheet("color: grey; border:2px solid grey;");

    ui.tab_widget->setCurrentIndex(0);

    ui.texture_combobox->setItemData(0, Qt::white, Qt::TextColorRole);

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

  }

  void RoverGUIPlugin::shutdownPlugin()
  {
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


// Recieves messages from the ROS joystick driver and used them to articulate the gripper and drive the rover.
void RoverGUIPlugin::joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
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
    // Extract rover name from the message source. Publisher is in the format /*rover_name*_GPS
    size_t found = publisher_name.find("_EKF");
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
    size_t found = topic.find("/odom");
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

void RoverGUIPlugin::targetPickUpEventHandler(const ros::MessageEvent<const sensor_msgs::Image> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();
    
    const sensor_msgs::ImageConstPtr& image = event.getMessage();

    // Extract rover name from the message source
    string topic = header.at("topic");
    size_t found = topic.find("/targetPickUpImage");
    string rover_name = topic.substr(1,found-1);

    int targetID = targetDetect(image);
    
    //Check all robots to ensure that no one is already holding the target
    bool targetPreviouslyCollected = false;
	for (map<string,int>::iterator it=targetsPickedUp.begin(); it!=targetsPickedUp.end(); ++it) {
		if (it->second == targetID) {
			targetPreviouslyCollected = true;
			break;
		}
	}
	
    if((targetID < 0) || (targetID == collectionZoneID) || targetPreviouslyCollected) {
        // No valid target was found in the image, or the target was the collection zone ID, or the target was already picked up by another robot
        
        //Publish -1 to alert robot of failed drop off event
        std_msgs::Int16 targetIDMsg;
        targetIDMsg.data = -1;
        targetPickUpPublisher[rover_name].publish(targetIDMsg);
    }
    else {
		//Record target ID according to the rover that reported it
        targetsPickedUp[rover_name] = targetID;
        emit sendInfoLogMessage("Resource " + QString::number(targetID) + " picked up by " + QString::fromStdString(rover_name));
        ui.num_targets_detected_label->setText(QString("<font color='white'>")+QString::number(targetsPickedUp.size())+QString("</font>"));
        
        //Publish target ID
        std_msgs::Int16 targetIDMsg;
        targetIDMsg.data = targetID;
        targetPickUpPublisher[rover_name].publish(targetIDMsg);
    }
}

void RoverGUIPlugin::targetDropOffEventHandler(const ros::MessageEvent<const sensor_msgs::Image> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const sensor_msgs::ImageConstPtr& image = event.getMessage();

    // Extract rover name from the message source
    string topic = header.at("topic");
    size_t found = topic.find("/targetDropOffImage");
    string rover_name = topic.substr(1,found-1);

    int targetID = targetDetect(image);

    if(targetID != collectionZoneID) {
        // This target does not match the official collection zone ID
    }
    else {
		//Use try-catch here in case a rover reports the collection zone ID without ever having picked up a target
        try {
			//Add target ID to list of dropped off targets
            targetsDroppedOff[targetsPickedUp.at(rover_name)] = true;
            emit sendInfoLogMessage("Resource " + QString::number(targetsPickedUp.at(rover_name)) + " dropped off by " + QString::fromStdString(rover_name));
            ui.num_targets_collected_label->setText(QString("<font color='white'>")+QString::number(targetsDroppedOff.size())+QString("</font>"));
            targetsPickedUp.erase(rover_name);
            ui.num_targets_detected_label->setText(QString("<font color='white'>")+QString::number(targetsPickedUp.size())+QString("</font>"));
            
            //Publish target ID (should always be equal to 256)
			std_msgs::Int16 targetIDMsg;
			targetIDMsg.data = targetID;
			targetDropOffPublisher[rover_name].publish(targetIDMsg);
        }
        catch(const std::out_of_range& oor) {
            emit sendInfoLogMessage(QString::fromStdString(rover_name) + " attempted a drop off but was not carrying a target");
            
            //Publish -1 to alert robot of failed drop off event
            std_msgs::Int16 targetIDMsg;
			targetIDMsg.data = -1;
			targetDropOffPublisher[rover_name].publish(targetIDMsg);
        }
    }
}

// Receives coordinates for all detected targets from camera
void RoverGUIPlugin::targetCoordinateEventHandler(const ros::MessageEvent<const shared_messages::TagsImage> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const shared_messages::TagsImageConstPtr& image = event.getMessage();

    // Extract rover name from the message source
    string topic = header.at("topic");
    size_t found = topic.find("/targets");
    string rover_name = topic.substr(1,found-1);

    // Each pair will store an individual corner coordinate and the center coordinate
    // for each april tag 
    std::pair<double, double> c1;
    std::pair<double, double> c2;
    std::pair<double, double> c3;
    std::pair<double, double> c4;
    std::pair<double, double> center;

    for(int i = 0; i < image->corners.size(); i++)
    {
        c1.first = image->corners[i].points[0].x;
        c1.second = image->corners[i].points[0].y;

        c2.first = image->corners[i].points[1].x;
        c2.second = image->corners[i].points[1].y;

        c3.first = image->corners[i].points[2].x;
        c3.second = image->corners[i].points[2].y;

        c4.first = image->corners[i].points[3].x;
        c4.second = image->corners[i].points[3].y;

        center.first = image->centers.points[i].x;
        center.second = image->centers.points[i].y;

        ui.camera_frame->addTarget(c1, c2, c3, c4, center);
    }

}


// Receives and stores the status update messages from rovers
void RoverGUIPlugin::statusEventHandler(const ros::MessageEvent<std_msgs::String const> &event)
{
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    // Extract rover name from the message source

    // This method is used rather than reading the publisher name to accomodate teams that changed the node name.
    string topic = header.at("topic");
    size_t found = topic.find("/status");
    string rover_name = topic.substr(1,found-1);

    const std_msgs::StringConstPtr& msg = event.getMessage();

    string status = msg->data;

    rover_statuses[rover_name] = status;
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

    // Attempt to read the simulation model xml file if it exists. If it does not exist assume this is a physical rover.
    const char *name = "GAZEBO_MODEL_PATH";
    char *model_root_cstr;
    model_root_cstr = getenv(name);
    QString model_root(model_root_cstr);

    QString model_path = model_root+"/"+QString::fromStdString(selected_rover_name)+"/model.sdf";

    readRoverModelXML(model_path);
    
    //Set up subscribers
    image_transport::ImageTransport it(nh);
    camera_subscriber = it.subscribe("/"+selected_rover_name+"/camera/image", 1, &RoverGUIPlugin::cameraEventHandler, this, image_transport::TransportHints("theora"));
    imu_subscriber = nh.subscribe("/"+selected_rover_name+"/imu", 10, &RoverGUIPlugin::IMUEventHandler, this);
    us_center_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarCenter", 10, &RoverGUIPlugin::centerUSEventHandler, this);
    us_left_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarLeft", 10, &RoverGUIPlugin::leftUSEventHandler, this);
    us_right_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarRight", 10, &RoverGUIPlugin::rightUSEventHandler, this);

    emit sendInfoLogMessage(QString("Displaying map for ")+QString::fromStdString(selected_rover_name));
    ui.map_frame->setRoverMapToDisplay(selected_rover_name);

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

    // Clear map
    // ui.map_frame->clearMap();

    // Enable control mode radio group now that a rover has been selected
    ui.autonomous_control_radio_button->setEnabled(true);
    ui.joystick_control_radio_button->setEnabled(true);
}

void RoverGUIPlugin::pollRoversTimerEventHandler()
{
    set<string>new_rover_names = findConnectedRovers();


    std::set<string> orphaned_rover_names;

    // Calculate which of the old rover names are not in the new list of rovers then clear their maps and control states.
    std::set_difference(rover_names.begin(), rover_names.end(), new_rover_names.begin(), new_rover_names.end(),
        std::inserter(orphaned_rover_names, orphaned_rover_names.end()));

    for (set<string>::iterator it = orphaned_rover_names.begin(); it != orphaned_rover_names.end(); ++it)
    {
        emit sendInfoLogMessage(QString("Clearing interface data for disconnected rover ") + QString::fromStdString(*it));
        ui.map_frame->clearMap(*it);
        rover_control_state.erase(*it); // Remove the control state for orphaned rovers
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
        encoder_subscribers[*it].shutdown();
        gps_subscribers[*it].shutdown();
        ekf_subscribers[*it].shutdown();
        targetPickUpSubscribers[*it].shutdown();
        targetDropOffSubscribers[*it].shutdown();
        targetCoordinateSubscribers[*it].shutdown();

        // Delete the subscribers
        status_subscribers.erase(*it);
        encoder_subscribers.erase(*it);
        gps_subscribers.erase(*it);
        ekf_subscribers.erase(*it);
        targetPickUpSubscribers.erase(*it);
        targetDropOffSubscribers.erase(*it);
        targetCoordinateSubscribers.erase(*it);
        
        // Shudown Publishers
        control_mode_publishers[*it].shutdown();
        targetPickUpPublisher[*it].shutdown();
        targetDropOffPublisher[*it].shutdown();

        // Delete Publishers
        control_mode_publishers.erase(*it);
        targetPickUpPublisher.erase(*it);
        targetDropOffPublisher.erase(*it);
    }

    // Wait for a rover to connect
    if (new_rover_names.empty())
    {
        //displayLogMessage("Waiting for rover to connect...");
        selected_rover_name = "";
        rover_control_state.clear();
        rover_names.clear();        
        ui.rover_list->clearSelection();
        ui.rover_list->clear();
        ui.rover_diags_list->clear();

        // Disable control mode group since no rovers are connected
        ui.autonomous_control_radio_button->setEnabled(false);
        ui.joystick_control_radio_button->setEnabled(false);
        ui.all_autonomous_button->setEnabled(false);
        ui.all_stop_button->setEnabled(false);
        ui.all_autonomous_button->setStyleSheet("color: grey; border:2px solid grey;");
        ui.all_stop_button->setStyleSheet("color: grey; border:2px solid grey;");
        return;
    }

    if (new_rover_names == rover_names)
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

            QString updated_rover_status = "";
            // Build new ui rover list string
            try
            {
                updated_rover_status = QString::fromStdString(rover_statuses.at(ui_rover_name));
            }
            catch (std::out_of_range& e)
            {
                emit sendInfoLogMessage("Error: No status entry for rover " + QString::fromStdString(ui_rover_name));
            }


            // Build new ui rover list string
            QString updated_rover_name_and_status = QString::fromStdString(ui_rover_name)
                                                    + " ("
                                                    + updated_rover_status
                                                    + ")";

            // Update the UI
            item->setText(updated_rover_name_and_status);
        }

        return;
    }

    rover_names = new_rover_names;
    
    emit sendInfoLogMessage("List of connected rovers has changed");
    selected_rover_name = "";
    ui.rover_list->clearSelection();
    ui.rover_list->clear();

    // Also clear the rover diagnostics list
    ui.rover_list->clear();
    
    //Enable all autonomous button
    ui.all_autonomous_button->setEnabled(true);
    ui.all_autonomous_button->setStyleSheet("color: white; border:2px solid white;");

    // This code is from above. Consider moving into a function or restructuring
    for(set<string>::const_iterator i = rover_names.begin(); i != rover_names.end(); ++i)
    {
        //Set up publishers
        control_mode_publishers[*i]=nh.advertise<std_msgs::UInt8>("/"+*i+"/mode", 10, true); // last argument sets latch to true
        targetPickUpPublisher[*i] = nh.advertise<std_msgs::Int16>("/"+*i+"/targetPickUpValue", 10, this);
        targetDropOffPublisher[*i] = nh.advertise<std_msgs::Int16>("/"+*i+"/targetDropOffValue", 10, this);

        //Set up subscribers
        status_subscribers[*i] = nh.subscribe("/"+*i+"/status", 10, &RoverGUIPlugin::statusEventHandler, this);
        obstacle_subscribers[*i] = nh.subscribe("/"+*i+"/obstacle", 10, &RoverGUIPlugin::obstacleEventHandler, this);
        encoder_subscribers[*i] = nh.subscribe("/"+*i+"/odom/", 10, &RoverGUIPlugin::encoderEventHandler, this);
        ekf_subscribers[*i] = nh.subscribe("/"+*i+"/odom/ekf", 10, &RoverGUIPlugin::EKFEventHandler, this);
        gps_subscribers[*i] = nh.subscribe("/"+*i+"/odom/navsat", 10, &RoverGUIPlugin::GPSEventHandler, this);
        targetPickUpSubscribers[*i] = nh.subscribe("/"+*i+"/targetPickUpImage", 10, &RoverGUIPlugin::targetPickUpEventHandler, this);
        targetDropOffSubscribers[*i] = nh.subscribe("/"+*i+"/targetDropOffImage", 10, &RoverGUIPlugin::targetDropOffEventHandler, this);
        targetCoordinateSubscribers[*i] = nh.subscribe("/"+*i+"/targets", 10, &RoverGUIPlugin::targetCoordinateEventHandler, this);
        rover_diagnostic_subscribers[*i] = nh.subscribe("/"+*i+"/diagnostics", 10, &RoverGUIPlugin::diagnosticEventHandler, this);

        QString rover_status = "";
        // Build new ui rover list string
        try
        {
            rover_status = QString::fromStdString(rover_statuses.at(*i));
        }
        catch (std::out_of_range& e)
        {
            emit sendInfoLogMessage("No status entry for rover " + QString::fromStdString(*i));
        }

        QString rover_name_and_status = QString::fromStdString(*i) // Add the rover name
                                                + " (" // Delimiters needed for parsing the rover name and status when read
                                                +  rover_status // Add the rover status
                                                + ")";

        QListWidgetItem* new_item = new QListWidgetItem(rover_name_and_status);
        new_item->setForeground(Qt::red);
        ui.rover_list->addItem(new_item);

        // Create the corresponding diagnostic data listwidgetitem
        QListWidgetItem* new_diags_item = new QListWidgetItem("");
        new_diags_item->setForeground(Qt::red);
        ui.rover_diags_list->addItem(new_diags_item);

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

    // Convert to strings

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

       diagnostic_display += " " + rate_str + " " + units;

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
        if (ui_rover_name.compare(rover_name)==0) break; // We found a rover with the right name
    }

    // Check the the rover was found in the rover list
    if (row >= ui.rover_list->count())
    {
        emit sendInfoLogMessage(QString::fromStdString("Received diagnostic data from an unknown rover: " + rover_name));
        return;
    }

    emit sendInfoLogMessage(QString::fromStdString("Received diagnostic data from " + rover_name));

    // Update the UI
    QListWidgetItem *item = ui.rover_diags_list->item(row);
    item->setText(QString::fromStdString(diagnostic_display));
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


void RoverGUIPlugin::displayDiagLogMessage(QString msg)
{
    if (msg.isEmpty()) msg = "Message is empty";
    if (msg == NULL) msg = "Message was a NULL pointer";


    // replace new lines with <br> in the message
    msg.replace("\n","<br>");

    QString new_message = msg+"<br>";
    diag_log_messages = diag_log_messages+new_message;
    ui.diag_log->setText("<font color='white'>"+diag_log_messages+"</font>");

    QScrollBar *sb = ui.diag_log->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void RoverGUIPlugin::displayInfoLogMessage(QString msg)
{
    if (msg.isEmpty()) msg = "Message is empty";
    if (msg == NULL) msg = "Message was a NULL pointer";


    // replace new lines with <br> in the message
    msg.replace("\n","<br>");

    QString new_message = msg+"<br>";
    info_log_messages = info_log_messages+new_message;
    ui.info_log->setText("<font color='white'>"+info_log_messages+"</font>");

    QScrollBar *sb = ui.info_log->verticalScrollBar();
    sb->setValue(sb->maximum());
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
    
    //Disable all autonomous button
    ui.all_autonomous_button->setEnabled(false);
    ui.all_autonomous_button->setStyleSheet("color: grey; border:2px solid grey;");
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
}

// Get the path to the world file containing the custom distribution from the user
void RoverGUIPlugin::customWorldButtonEventHandler()
{
    const char *name = "SWARMATHON_APP_ROOT";
    char *app_root_cstr;
    app_root_cstr = getenv(name);
    QString app_root = QString(app_root_cstr) + "/simulation/worlds/";

    QString path = QFileDialog::getOpenFileName(widget, tr("Open File"),
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

    // Set the button color to reflect whether or not it is disabled
    // Clear the sim path if custom distribution it deselected
    if( toggled )
    {
        ui.custom_world_path_button->setStyleSheet("color: white; border:2px solid white;");
    }
    else
    {
        sim_mgr.setCustomWorldPath("");
        ui.custom_world_path->setText("");
        ui.custom_world_path_button->setStyleSheet("color: grey; border:2px solid grey;");
    }
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

    // Initialize the target counts
    ui.num_targets_collected_label->setText(QString("<font color='white'>0</font>"));
    ui.num_targets_detected_label->setText(QString("<font color='white'>0</font>"));
    targetsPickedUp.clear();
    targetsDroppedOff.clear();

    QProcess* sim_server_process = sim_mgr.startGazeboServer();
    connect(sim_server_process, SIGNAL(finished(int)), this, SLOT(gazeboServerFinishedEventHandler()));


    if (ui.final_radio_button->isChecked())
    {
         arena_dim = 23.1;
         addFinalsWalls();
    }
    else
    {
        arena_dim = 15;
        addPrelimsWalls();
    }

    emit sendInfoLogMessage(QString("Set arena size to ")+QString::number(arena_dim)+"x"+QString::number(arena_dim));

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


    emit sendInfoLogMessage("Adding collection disk...");
    float collection_disk_radius = 0.5; // meters
    sim_mgr.addModel("collection_disk", "collection_disk", 0, 0, 0, collection_disk_radius);

    int n_rovers_created = 0;
    int n_rovers = 3;
    if (ui.final_radio_button->isChecked()) n_rovers = 6;

    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Creating rovers");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    emit sendInfoLogMessage("Adding rover achilles...");
    return_msg = sim_mgr.addRover("achilles", 0, 1, 0);
    emit sendInfoLogMessage(return_msg);

    emit sendInfoLogMessage("Starting rover node for achilles...");
    return_msg = sim_mgr.startRoverNode("achilles");
    emit sendInfoLogMessage(return_msg);

    progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    emit sendInfoLogMessage("Adding rover aeneas...");
    return_msg = sim_mgr.addRover("aeneas", -1, 0, 0);
    emit sendInfoLogMessage(return_msg);

    emit sendInfoLogMessage("Starting rover node for aeneas...");
    return_msg = sim_mgr.startRoverNode("aeneas");
    emit sendInfoLogMessage(return_msg);

    progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    emit sendInfoLogMessage("Adding rover ajax...");
    return_msg = sim_mgr.addRover("ajax", 1, 0, 0);
    emit sendInfoLogMessage(return_msg);

   emit sendInfoLogMessage("Starting rover node for ajax...");
   return_msg = sim_mgr.startRoverNode("ajax");
   emit sendInfoLogMessage(return_msg);

   progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   if (ui.final_radio_button->isChecked())
   {

       emit sendInfoLogMessage("Adding rover diomedes...");
       return_msg = sim_mgr.addRover("diomedes", 1, 1, 0);
       emit sendInfoLogMessage(return_msg);

       emit sendInfoLogMessage("Starting rover node for diomedes...");
       return_msg = sim_mgr.startRoverNode("diomedes");
       emit sendInfoLogMessage(return_msg);

       progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

       emit sendInfoLogMessage("Adding rover hector...");
       return_msg = sim_mgr.addRover("hector", -1, -1, 0);
       emit sendInfoLogMessage(return_msg);

       emit sendInfoLogMessage("Starting rover node for hector...");
       return_msg = sim_mgr.startRoverNode("hector");
       emit sendInfoLogMessage(return_msg);

        progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

       emit sendInfoLogMessage("Adding rover paris...");
       return_msg = sim_mgr.addRover("paris", 1, -1, 0);
       emit sendInfoLogMessage(return_msg);

       emit sendInfoLogMessage("Starting rover node for paris...");
       return_msg = sim_mgr.startRoverNode("paris");
       emit sendInfoLogMessage(return_msg);

        progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

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

   emit sendInfoLogMessage("Finished building simulation.");

  // Visualize the simulation by default call button event handler
   visualizeSimulationButtonEventHandler();

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

    for (map<string,ros::Subscriber>::iterator it=encoder_subscribers.begin(); it!=encoder_subscribers.end(); ++it) it->second.shutdown();
    encoder_subscribers.clear();
    for (map<string,ros::Subscriber>::iterator it=gps_subscribers.begin(); it!=gps_subscribers.end(); ++it) it->second.shutdown();
    gps_subscribers.clear();
    for (map<string,ros::Subscriber>::iterator it=ekf_subscribers.begin(); it!=ekf_subscribers.end(); ++it) it->second.shutdown();
    ekf_subscribers.clear();
    us_center_subscriber.shutdown();
    us_left_subscriber.shutdown();
    us_right_subscriber.shutdown();
    imu_subscriber.shutdown();

    // Possible error - the following seems to shutdown all subscribers not just those from simulation

    for (map<string,ros::Subscriber>::iterator it=status_subscribers.begin(); it!=status_subscribers.end(); ++it) it->second.shutdown();
    status_subscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=obstacle_subscribers.begin(); it!=obstacle_subscribers.end(); ++it) it->second.shutdown();
    obstacle_subscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=targetPickUpSubscribers.begin(); it!=targetPickUpSubscribers.end(); ++it) it->second.shutdown();
    targetPickUpSubscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=targetDropOffSubscribers.begin(); it!=targetDropOffSubscribers.end(); ++it) it->second.shutdown();
    targetDropOffSubscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=targetCoordinateSubscribers.begin(); it!=targetCoordinateSubscribers.end(); ++it) it->second.shutdown();
    targetCoordinateSubscribers.clear();

    camera_subscriber.shutdown();

    emit sendInfoLogMessage("Shutting down publishers...");

    for (map<string,ros::Publisher>::iterator it=control_mode_publishers.begin(); it!=control_mode_publishers.end(); ++it) it->second.shutdown();
    control_mode_publishers.clear();

    return_msg += sim_mgr.stopGazeboClient();
    return_msg += "<br>";
    return_msg += sim_mgr.stopGazeboServer();
    emit sendInfoLogMessage(return_msg);

    ui.visualize_simulation_button->setEnabled(false);
    ui.build_simulation_button->setEnabled(true);
    ui.clear_simulation_button->setEnabled(false);
    display_sim_visualization = false;


    ui.build_simulation_button->setStyleSheet("color: white; border:1px solid white;");
    ui.visualize_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.clear_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");

    // Clear the task status values
    ui.num_targets_collected_label->setText("<font color='white'>0</font>");
    ui.num_targets_detected_label->setText("<font color='white'>0</font>");
    targetsPickedUp.clear();
    targetsDroppedOff.clear();
    obstacle_call_count = 0;
    emit updateObstacleCallCount("<font color='white'>0</font>");
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
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing 256 Targets");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.resize(500, 50);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.show();

    QString output;

    float proposed_x;
    float proposed_y;

    // 256 piles of 1 tag

    // d is the distance from the center of the arena to the boundary minus the barrier clearance, i.e. the region where tags can be placed
    // is d - U(0,2d) where U(a,b) is a uniform distribition bounded by a and b.
    // (before checking for collisions including the collection disk at the center)
    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_1_clearance);

    for (int i = 0; i < 256; i++)
    {
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            emit sendInfoLogMessage("Tried to place target "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y) + "...");
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
       }
       while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_1_clearance));
       emit sendInfoLogMessage("<font color=green>Succeeded.</font>");

        output = sim_mgr.addModel(QString("at")+QString::number(i),  QString("at")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_1_clearance);

       progress_dialog.setValue(i*100.0f/256);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    }
    emit sendInfoLogMessage("Placed 256 single targets");

    return output;
}

QString RoverGUIPlugin::addClusteredTargets()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing 256 Targets into 4 Clusters (64 targets each)");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    QString output;

    float proposed_x;
    float proposed_y;

    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_64_clearance);

    // Four piles of 64
    for (int i = 0; i < 4; i++)
    {
        // Keep GUI responsive
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

        do
        {
            emit sendInfoLogMessage("Tried to place cluster "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_64_clearance));
        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");

        progress_dialog.setValue(i*100.0f/4);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

        output = sim_mgr.addModel(QString("atags64_")+QString::number(i), QString("atags64_")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_64_clearance);
        emit sendInfoLogMessage(output);
    }

    emit sendInfoLogMessage("Placed four clusters of 64 targets");

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

    QString output = "";
    // One pile of 64

    float proposed_x;
    float proposed_y;

    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_64_clearance);

    do
    {
        emit sendInfoLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
        proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
    }
    while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_64_clearance));
    emit sendInfoLogMessage("<font color=green>Succeeded.</font>");

    progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
    output+= sim_mgr.addModel("atags64_0", "atags64_0", proposed_x, proposed_y, 0, target_cluster_size_64_clearance);

    d = arena_dim/2.0-(barrier_clearance+target_cluster_size_16_clearance);

    // Four piles of 16
    for (int i = 0; i < 4; i++)
    {
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
            emit sendInfoLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_16_clearance));
        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");


        progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        output+= sim_mgr.addModel(QString("atags16_")+QString::number(i), QString("atags16_")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_64_clearance);
    }

    d = arena_dim/2.0-(barrier_clearance+target_cluster_size_4_clearance);

    // Sixteen piles of 4
    for (int i = 0; i < 16; i++)
    {
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            emit sendInfoLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_4_clearance));
        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");

        progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        output+= sim_mgr.addModel(QString("atags4_")+QString::number(i), QString("atags4_")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_4_clearance);
    }

    d = arena_dim/2.0-(barrier_clearance+target_cluster_size_1_clearance);

    // Sixty-four piles of 1 (using tags 192 through 255 to avoid duplication with piles above)
    for (int i = 192; i < 256; i++)
    {
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            emit sendInfoLogMessage("Tried to place target "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_1_clearance));
        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");

        progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        output+= sim_mgr.addModel(QString("at")+QString::number(i), QString("at")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_1_clearance);
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

int RoverGUIPlugin::targetDetect(const sensor_msgs::ImageConstPtr& rawImage) {

    cv_bridge::CvImagePtr cvImage;

	//Convert from MONO8 to BGR8
	//TODO: consider whether we should let the camera publish as BGR8 and skip this conversion
    try {
        cvImage = cv_bridge::toCvCopy(rawImage); //, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rawImage->encoding.c_str());
        return -1;
    }

    //Create Mat image for processing
    cv::Mat matImage = cvImage->image;
    cv::cvtColor(matImage, matImage, cv::COLOR_BGR2GRAY);

    //Force greyscale and force image size.  This is only for Gazebo.
    //TODO: fix model so Gazebo publishes the correct format
    //TODO: if Mat is only used here, why not use the cvImage format here and skip the Mat image completely?
    if (matImage.cols != 320 && matImage.rows != 240) {
        cv::resize(matImage, matImage, cv::Size(320, 240), cv::INTER_LINEAR);
    }

    //Copy all image data into an array that AprilTag library expects
    image_u8_t *im = copy_image_data_into_u8_container(	matImage.cols, 
							matImage.rows, 
							(uint8_t *) matImage.data, 
							matImage.step);

    //Detect AprilTags
    zarray_t *detections = apriltag_detector_detect(td, im);
    
    //Check result for valid tag
    for (int i = 0; i < zarray_size(detections); i++) {
	    apriltag_detection_t *det;
	    zarray_get(detections, i, &det);
	
	    int tag = det->id;
	    
	    //Return first tag that has not been collected
	    if (targetsDroppedOff.count(tag) == 0){
			return tag;
		}
	}
	
	return -1;
}

image_u8_t* RoverGUIPlugin::copy_image_data_into_u8_container(int width, int height, uint8_t *rgb, int stride) {
    for (int y = 0; y < u8_image->height; y++) {
        for (int x = 0; x < u8_image->width; x++) {
            u8_image->buf[y * u8_image->stride + x] = rgb[y * stride + x + 0];
        }
    }
    return u8_image;
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

            float speed = 0.5;

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

// Refocus on the main ui widget so the rover list doesn't start capturing key strokes making keyboard rover driving not work.
void RoverGUIPlugin::refocusKeyboardEventHandler()
{
    widget->setFocus();
}

// Clean up memory when this object is deleted
RoverGUIPlugin::~RoverGUIPlugin()
{
    delete joystickGripperInterface;
}

} // End namespace



PLUGINLIB_EXPORT_CLASS(rqt_rover_gui::RoverGUIPlugin, rqt_gui_cpp::Plugin)

