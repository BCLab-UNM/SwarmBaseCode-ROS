// Author: Matthew Fricke
// E-mail: matthew@fricke.co.uk
// Date: 9-16-205
// Purpose: implementation of a simple graphical front end for the UNM-NASA Swarmathon rovers.
// License: GPL3

#include <rover_gui_plugin.h>
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
    log_messages = "";
    joy_process = NULL;

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

	//Initialize AprilTag detection apparatus
	tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    //Allocate image memory up front so it doesn't need to be done for every image frame
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
    QString version_qstr("<font color='white'>"+QString::fromUtf8(GIT_VERSION)+"</font>");
    ui.version_number_label->setText(version_qstr);

    widget->setWindowTitle("Rover Interface: Built on " + QString::fromUtf8(BUILD_TIME) );

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
    connect(this, SIGNAL(joystickForwardUpdate(double)), ui.joy_lcd_forward, SLOT(display(double)));
    connect(this, SIGNAL(joystickBackUpdate(double)), ui.joy_lcd_back, SLOT(display(double)));
    connect(this, SIGNAL(joystickLeftUpdate(double)), ui.joy_lcd_left, SLOT(display(double)));
    connect(this, SIGNAL(joystickRightUpdate(double)), ui.joy_lcd_right, SLOT(display(double)));
    connect(this, SIGNAL(updateObstacleCallCount(QString)), ui.perc_of_time_avoiding_obstacles, SLOT(setText(QString)));
    connect(this, SIGNAL(updateLog(QString)), this, SLOT(displayLogMessage(QString)));

    // Create a subscriber to listen for joystick events
    joystick_subscriber = nh.subscribe("/joy", 1000, &RoverGUIPlugin::joyEventHandler, this);

    displayLogMessage("Searching for rovers...");

    // Add discovered rovers to the GUI list
    rover_poll_timer = new QTimer(this);
    connect(rover_poll_timer, SIGNAL(timeout()), this, SLOT(pollRoversTimerEventHandler()));
    rover_poll_timer->start(5000);

    // Setup the initial display parameters for the map
    ui.map_frame->setDisplayGPSData(ui.gps_checkbox->isChecked());
    ui.map_frame->setDisplayEncoderData(ui.encoder_checkbox->isChecked());
    ui.map_frame->setDisplayEKFData(ui.ekf_checkbox->isChecked());

    ui.joystick_frame->setHidden(true);

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

void RoverGUIPlugin::joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
     if (joystick_publisher)
        {
        //Set the gui values. Filter values to be large enough to move the physical rover.
        if (joy_msg->axes[4] >= 0.1)
        {
            emit joystickForwardUpdate(joy_msg->axes[4]);
        }
        if (joy_msg->axes[4] <= -0.1)
        {
            emit joystickBackUpdate(-joy_msg->axes[4]);
        }
        //If value is too small, display 0.
        if (abs(joy_msg->axes[4]) < 0.1)
        {
            emit joystickForwardUpdate(0);
            emit joystickBackUpdate(0);
        }

        if (joy_msg->axes[3] >= 0.1)
        {
            emit joystickLeftUpdate(joy_msg->axes[3]);
        }
        if (joy_msg->axes[3] <= -0.1)
        {
            emit joystickRightUpdate(-joy_msg->axes[3]);
        }
        //If value is too small, display 0.
        if (abs(joy_msg->axes[3]) < 0.1)
        {
            emit joystickLeftUpdate(0);
            emit joystickRightUpdate(0);
        }

        // Magic axis values in the code below were taken the rover_driver_rqt_motor code /joystick output for default linear and angular velocities.
        // Magic indicies are taken from rover_motor.cpp.
        // This way the code is consistent with the existing GUI joystick.
        // A better way would be to standardize a manual movement control interface and requre all input mechanisms to take input from the user
        // and repackage te information according to the interface spec.
        geometry_msgs::Twist standardized_joy_msg;

        if (abs(joy_msg->axes[4]) >= 0.1)
        {
          standardized_joy_msg.linear.x = joy_msg->axes[4];
        }

        if (abs(joy_msg->axes[3]) >= 0.1)
        {
          standardized_joy_msg.angular.z = joy_msg->axes[3];
        }

        joystick_publisher.publish(standardized_joy_msg);
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
        emit updateLog("Resource " + QString::number(targetID) + " picked up by " + QString::fromStdString(rover_name));
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
            emit updateLog("Resource " + QString::number(targetsPickedUp.at(rover_name)) + " dropped off by " + QString::fromStdString(rover_name));
            ui.num_targets_collected_label->setText(QString("<font color='white'>")+QString::number(targetsDroppedOff.size())+QString("</font>"));
            targetsPickedUp.erase(rover_name);
            ui.num_targets_detected_label->setText(QString("<font color='white'>")+QString::number(targetsPickedUp.size())+QString("</font>"));
            
            //Publish target ID (should always be equal to 256)
			std_msgs::Int16 targetIDMsg;
			targetIDMsg.data = targetID;
			targetDropOffPublisher[rover_name].publish(targetIDMsg);
        }
        catch(const std::out_of_range& oor) {
            emit updateLog(QString::fromStdString(rover_name) + " attempted a drop off but was not carrying a target");
            
            //Publish -1 to alert robot of failed drop off event
            std_msgs::Int16 targetIDMsg;
			targetIDMsg.data = -1;
			targetDropOffPublisher[rover_name].publish(targetIDMsg);
        }
    }
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
    displayLogMessage("Selcted Rover Changed");

    if (!current) return; // Check to make sure the current selection isn't null

    selected_rover_name = current->text().toStdString();
    string rover_name_msg = "<font color='white'>Rover: " + selected_rover_name + "</font>";
    QString rover_name_msg_qstr = QString::fromStdString(rover_name_msg);
    ui.rover_name->setText(rover_name_msg_qstr);

    displayLogMessage(QString("Selected rover: ") + QString::fromStdString(selected_rover_name));

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

    displayLogMessage(QString("Displaying map for ")+QString::fromStdString(selected_rover_name));
    ui.map_frame->setRoverMapToDisplay(selected_rover_name);

    // No entry for this rover name
    if ( 0 == rover_control_state.count(selected_rover_name) )
    {
        // Default to joystick
        ui.joystick_control_radio_button->setChecked(true);
        ui.autonomous_control_radio_button->setChecked(false);
        joystickRadioButtonEventHandler(true); // Manually trigger the joystick selected event
        rover_control_state[selected_rover_name]=1;
        displayLogMessage("New rover selected");
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
            break;
        default:
            displayLogMessage("Unknown control state: "+QString::number(control_state));
        }

        displayLogMessage("Existing rover selected");
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
        displayLogMessage(QString("Clearing interface data for disconnected rover ") + QString::fromStdString(*it));
        ui.map_frame->clearMap(*it);
        rover_control_state.erase(*it); // Remove the control state for orphaned rovers
        
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
        encoder_subscribers[*it].shutdown();
        gps_subscribers[*it].shutdown();
        ekf_subscribers[*it].shutdown();
        targetPickUpSubscribers[*it].shutdown();
        targetDropOffSubscribers[*it].shutdown();

        // Delete the subscribers
        encoder_subscribers.erase(*it);
        gps_subscribers.erase(*it);
        ekf_subscribers.erase(*it);
        targetPickUpSubscribers.erase(*it);
        targetDropOffSubscribers.erase(*it);
        
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
        return;
    }

    rover_names = new_rover_names;
    
    displayLogMessage("List of connected rovers has changed");
    selected_rover_name = "";
    ui.rover_list->clearSelection();
    ui.rover_list->clear();
    
    //Enable all autonomous button
    ui.all_autonomous_button->setEnabled(true);
    ui.all_autonomous_button->setStyleSheet("color: white; border:2px solid white;");

    for(set<string>::const_iterator i = rover_names.begin(); i != rover_names.end(); ++i)
    {
        QListWidgetItem* new_item = new QListWidgetItem(QString::fromStdString(*i));
        new_item->setForeground(Qt::red);
        ui.rover_list->addItem(new_item);
        
        //Set up publishers
        control_mode_publishers[*i]=nh.advertise<std_msgs::UInt8>("/"+*i+"/mode", 10, true); // last argument sets latch to true
        targetPickUpPublisher[*i] = nh.advertise<std_msgs::Int16>("/"+*i+"/targetPickUpValue", 10, this);
        targetDropOffPublisher[*i] = nh.advertise<std_msgs::Int16>("/"+*i+"/targetDropOffValue", 10, this);
        
        //Set up subscribers
        obstacle_subscribers[*i] = nh.subscribe("/"+*i+"/obstacle", 10, &RoverGUIPlugin::obstacleEventHandler, this);
        encoder_subscribers[*i] = nh.subscribe("/"+*i+"/odom/", 10, &RoverGUIPlugin::encoderEventHandler, this);
        ekf_subscribers[*i] = nh.subscribe("/"+*i+"/odom/ekf", 10, &RoverGUIPlugin::EKFEventHandler, this);
        gps_subscribers[*i] = nh.subscribe("/"+*i+"/odom/navsat", 10, &RoverGUIPlugin::GPSEventHandler, this);
        targetPickUpSubscribers[*i] = nh.subscribe("/"+*i+"/targetPickUpImage", 10, &RoverGUIPlugin::targetPickUpEventHandler, this);
        targetDropOffSubscribers[*i] = nh.subscribe("/"+*i+"/targetDropOffImage", 10, &RoverGUIPlugin::targetDropOffEventHandler, this);
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


    // replace new lines with <br> in the message
    msg.replace("\n","<br>");

    QString new_message = msg+"<br>";
    log_messages = log_messages+new_message;
    ui.log->setText("<font color='white'>"+log_messages+"</font>");

    QScrollBar *sb = ui.log->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void RoverGUIPlugin::autonomousRadioButtonEventHandler(bool marked)
{
    if (!marked) return;

    rover_control_state[selected_rover_name] = 2;

    std_msgs::UInt8 control_mode_msg;
    control_mode_msg.data = 2; // 2 indicates autonomous control

    control_mode_publishers[selected_rover_name].publish(control_mode_msg);
    displayLogMessage(QString::fromStdString(selected_rover_name)+" changed to autonomous control");

    QString return_msg = stopROSJoyNode();
    displayLogMessage(return_msg);
    
    //Enable all stop button
    ui.all_stop_button->setEnabled(true);
    ui.all_stop_button->setStyleSheet("color: white; border:2px solid white;");
}

void RoverGUIPlugin::joystickRadioButtonEventHandler(bool marked)
{
    if (!marked) return;

    rover_control_state[selected_rover_name] = 1;
    displayLogMessage("Setting up joystick publisher " + QString::fromStdString("/"+selected_rover_name+"/joystick"));
    joystick_publisher = nh.advertise<geometry_msgs::Twist>("/"+selected_rover_name+"/joystick", 10, this);

    std_msgs::UInt8 control_mode_msg;
    control_mode_msg.data = 1; // 1 indicates manual control

    control_mode_publishers[selected_rover_name].publish(control_mode_msg);
    displayLogMessage(QString::fromStdString(selected_rover_name)+" changed to joystick control");\

    QString return_msg = startROSJoyNode();
    displayLogMessage(return_msg);
    
    //Enable all autonomous button
    ui.all_autonomous_button->setEnabled(true);
    ui.all_autonomous_button->setStyleSheet("color: white; border:2px solid white;");
}

void RoverGUIPlugin::allAutonomousButtonEventHandler()
{
    displayLogMessage("changing all rovers to autonomous control...");

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
    displayLogMessage("changing all rovers to manual control...");

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

void RoverGUIPlugin::buildSimulationButtonEventHandler()
{
    displayLogMessage("Building simulation...");

    ui.build_simulation_button->setEnabled(false);

    ui.build_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");

    QString return_msg;

    if (sim_mgr.isGazeboServerRunning())
    {
        displayLogMessage("A gazebo server simulation process is already running. Restart the Swarmathon GUI to clear.");
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

    displayLogMessage(QString("Set arena size to ")+QString::number(arena_dim)+"x"+QString::number(arena_dim));

    if (ui.texture_combobox->currentText() == "Gravel")
    {
    displayLogMessage("Adding gravel ground plane...");
    return_msg = sim_mgr.addGroundPlane("mars_ground_plane");
    displayLogMessage(return_msg);
    }
    else if (ui.texture_combobox->currentText() == "KSC Concrete")
    {
    displayLogMessage("Adding concrete ground plane...");
    return_msg = sim_mgr.addGroundPlane("concrete_ground_plane");
    displayLogMessage(return_msg);
    }
    else if (ui.texture_combobox->currentText() == "Car park")
    {
    displayLogMessage("Adding carpark ground plane...");
    return_msg = sim_mgr.addGroundPlane("carpark_ground_plane");
    displayLogMessage(return_msg);
    }
    else
    {
        displayLogMessage("Unknown ground plane...");
    }


    displayLogMessage("Adding collection disk...");
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

    displayLogMessage("Adding rover achilles...");
    return_msg = sim_mgr.addRover("achilles", 0, 1, 0);
    displayLogMessage(return_msg);

    displayLogMessage("Starting rover node for achilles...");
    return_msg = sim_mgr.startRoverNode("achilles");
    displayLogMessage(return_msg);

    progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    displayLogMessage("Adding rover aeneas...");
    return_msg = sim_mgr.addRover("aeneas", -1, 0, 0);
    displayLogMessage(return_msg);

    displayLogMessage("Starting rover node for aeneas...");
    return_msg = sim_mgr.startRoverNode("aeneas");
    displayLogMessage(return_msg);

    progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    displayLogMessage("Adding rover ajax...");
    return_msg = sim_mgr.addRover("ajax", 1, 0, 0);
    displayLogMessage(return_msg);

   displayLogMessage("Starting rover node for ajax...");
   return_msg = sim_mgr.startRoverNode("ajax");
   displayLogMessage(return_msg);

   progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   if (ui.final_radio_button->isChecked())
   {

       displayLogMessage("Adding rover diomedes...");
       return_msg = sim_mgr.addRover("diomedes", 1, 1, 0);
       displayLogMessage(return_msg);

       displayLogMessage("Starting rover node for diomedes...");
       return_msg = sim_mgr.startRoverNode("diomedes");
       displayLogMessage(return_msg);

       progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

       displayLogMessage("Adding rover hector...");
       return_msg = sim_mgr.addRover("hector", -1, -1, 0);
       displayLogMessage(return_msg);

       displayLogMessage("Starting rover node for hector...");
       return_msg = sim_mgr.startRoverNode("hector");
       displayLogMessage(return_msg);

        progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

       displayLogMessage("Adding rover paris...");
       return_msg = sim_mgr.addRover("paris", 1, -1, 0);
       displayLogMessage(return_msg);

       displayLogMessage("Starting rover node for paris...");
       return_msg = sim_mgr.startRoverNode("paris");
       displayLogMessage(return_msg);

        progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

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

   displayLogMessage("Finished building simulation.");

  // Visualize the simulation by default call button event handler
   visualizeSimulationButtonEventHandler();

}

void RoverGUIPlugin::clearSimulationButtonEventHandler()
{
    if (!sim_mgr.isGazeboServerRunning())
    {
        displayLogMessage("Simulation is not running.");

        return;
    }

    displayLogMessage("Ending simulation...");

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

    displayLogMessage("Shutting down subscribers...");

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
    for (map<string,ros::Subscriber>::iterator it=obstacle_subscribers.begin(); it!=obstacle_subscribers.end(); ++it) it->second.shutdown();

    obstacle_subscribers.clear();
    for (map<string,ros::Subscriber>::iterator it=targetPickUpSubscribers.begin(); it!=targetPickUpSubscribers.end(); ++it) it->second.shutdown();
    targetPickUpSubscribers.clear();
    for (map<string,ros::Subscriber>::iterator it=targetDropOffSubscribers.begin(); it!=targetDropOffSubscribers.end(); ++it) it->second.shutdown();
    targetDropOffSubscribers.clear();
    camera_subscriber.shutdown();

    displayLogMessage("Shutting down publishers...");

    for (map<string,ros::Publisher>::iterator it=control_mode_publishers.begin(); it!=control_mode_publishers.end(); ++it) it->second.shutdown();
    control_mode_publishers.clear();

    return_msg += sim_mgr.stopGazeboClient();
    return_msg += "<br>";
    return_msg += sim_mgr.stopGazeboServer();
    displayLogMessage(return_msg);

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
        displayLogMessage("Simulation is not running.");

        return;
    }

    QString return_msg;
    // toggle visualize or not
    display_sim_visualization = !display_sim_visualization;

    if (display_sim_visualization)
    {
        displayLogMessage("Visualizing simulation...");

        QProcess* sim_client_process = sim_mgr.startGazeboClient();
    }
    else
    {
        displayLogMessage("Ending visualization...");

        return_msg = sim_mgr.stopGazeboClient();
        displayLogMessage(return_msg);
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
            displayLogMessage("Tried to place target "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y) + "...");
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
       }
       while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_1_clearance));
       displayLogMessage("<font color=green>Succeeded.</font>");

        output = sim_mgr.addModel(QString("at")+QString::number(i),  QString("at")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_1_clearance);

       progress_dialog.setValue(i*100.0f/256);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
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
            displayLogMessage("Tried to place cluster "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_64_clearance));
        displayLogMessage("<font color=green>Succeeded.</font>");

        progress_dialog.setValue(i*100.0f/4);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

        output = sim_mgr.addModel(QString("atags64_")+QString::number(i), QString("atags64_")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_64_clearance);
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
        displayLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
        proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
    }
    while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_64_clearance));
    displayLogMessage("<font color=green>Succeeded.</font>");

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
            displayLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_16_clearance));
        displayLogMessage("<font color=green>Succeeded.</font>");


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
            displayLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_4_clearance));
        displayLogMessage("<font color=green>Succeeded.</font>");

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
            displayLogMessage("Tried to place target "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_1_clearance));
        displayLogMessage("<font color=green>Succeeded.</font>");

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
        displayLogMessage("Moving rover back into the arena");
        QString return_msg = sim_mgr.moveRover(rover_name, x_comp, y, 0);
        displayLogMessage(return_msg);
    }
}

void RoverGUIPlugin::readRoverModelXML(QString path)
{
    ifstream model_file;
    model_file.open(path.toStdString(), ios::in);
    if (model_file.is_open())
        displayLogMessage("Read model file at " + path );
    else
    {
        displayLogMessage(QString::fromStdString(selected_rover_name) + " appears to be a physical rover.");
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

    displayLogMessage("Gazebo client exited");

    ui.visualize_simulation_button->setEnabled(false);
    ui.clear_simulation_button->setEnabled(false);
    ui.build_simulation_button->setEnabled(true);

    ui.visualize_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.clear_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.build_simulation_button->setStyleSheet("color: white; border:1px solid white;");
}

bool RoverGUIPlugin::eventFilter(QObject *target, QEvent *event)
{
    geometry_msgs::Twist standardized_joy_msg;
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
                standardized_joy_msg.linear.x = speed;
                ui.joy_lcd_forward->display(speed);
                break;
            case Qt::Key_K:
                standardized_joy_msg.linear.x = -speed;
                ui.joy_lcd_back->display(speed);
                break;
            case Qt::Key_J:
                standardized_joy_msg.angular.z = speed;
                ui.joy_lcd_left->display(speed);
                break;
            case Qt::Key_L:
                standardized_joy_msg.angular.z = -speed;
                ui.joy_lcd_right->display(speed);
                break;
            default:
                // Not a direction key so ignore
                direction_key = false;
            }

            if (direction_key )
            {
                joystick_publisher.publish(standardized_joy_msg);
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
                standardized_joy_msg.linear.x = 0;
                standardized_joy_msg.angular.z = 0;
                ui.joy_lcd_forward->display(0);
                ui.joy_lcd_back->display(0);
                ui.joy_lcd_left->display(0);
                ui.joy_lcd_right->display(0);

                joystick_publisher.publish(standardized_joy_msg);
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

// Refocus on the main ui widget so the rover list doesn't start capturing key strokes making keyboard rover driving not work.
void RoverGUIPlugin::refocusKeyboardEventHandler()
{
    widget->setFocus();
}

} // End namespace



PLUGINLIB_EXPORT_CLASS(rqt_rover_gui::RoverGUIPlugin, rqt_gui_cpp::Plugin)

