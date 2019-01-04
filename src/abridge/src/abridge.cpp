#include <ros/ros.h>

//ROS libraries
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt8.h>

#include <swarmie_msgs/Skid.h>

//Package include
#include <usbSerial.h>

using namespace std;

//aBridge functions
void driveCommandHandler(const swarmie_msgs::Skid::ConstPtr& message);
void fingerAngleHandler(const std_msgs::Float32::ConstPtr& angle);
void wristAngleHandler(const std_msgs::Float32::ConstPtr& angle);
void serialActivityTimer(const ros::TimerEvent& e);
void publishRosTopics();
void parseData(string data);
std::string getHumanFriendlyTime();

//Globals
geometry_msgs::QuaternionStamped finger_angle;
geometry_msgs::QuaternionStamped wrist_angle;
sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
sensor_msgs::Range sonar_left;
sensor_msgs::Range sonar_center;
sensor_msgs::Range sonar_right;
USBSerial usb;
const int baud = 115200;
char data_cmd[] = "d\n";
char move_cmd[16];
char host[128];
const float delta_time = 0.1; //abridge's update interval
int current_mode = 0;
string published_name;

// Allowing messages to be sent to the arduino too fast causes a disconnect
// This is the minimum time between messages to the arduino in microseconds.
// Only used with the gripper commands to fix a manual control bug.
unsigned int min_usb_send_delay = 100;

float heartbeat_publish_interval = 2;


//PID constants and arrays
const int hist_array_length = 1000;

float vel_ff = 0; // Velocity feed forward
int step_v = 0; // Keeps track of the point in the ev_array for adding new error each update cycle.
float ev_array[hist_array_length]; // History array of previous error for (arraySize/hz) seconds (error Velocity Array)
float vel_error[4] = {0,0,0,0}; // Contains current velocity error and error 3 steps in the past.

int step_y = 0; // Keeps track of the point in the ey_array for adding new error each update cycle.
float ey_array[hist_array_length]; // History array of previous error for (arraySize/hz) seconds (error Yaw Array)
float yaw_error[4] = {0,0,0,0}; // Contains current yaw error and error 3 steps in the past.

float prev_lin = 0;
float prev_yaw = 0;

ros::Time prev_drive_command_update_time;

//Publishers
ros::Publisher finger_angle_publisher;
ros::Publisher wrist_angle_publisher;
ros::Publisher imu_publisher;
ros::Publisher odom_publisher;
ros::Publisher sonar_left_publisher;
ros::Publisher sonar_center_publisher;
ros::Publisher sonar_right_publisher;
ros::Publisher info_log_publisher;
ros::Publisher heartbeat_publisher;

//Subscribers
ros::Subscriber drive_control_subscriber;
ros::Subscriber finger_angle_subscriber;
ros::Subscriber wrist_angle_subscriber;
ros::Subscriber mode_subscriber;

//Timers
ros::Timer publish_timer;
ros::Timer publish_heartbeat_timer;

//Callback handlers
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);

int main(int argc, char **argv)
{

  gethostname(host, sizeof (host));
  string hostname(host);
  ros::init(argc, argv, (hostname + "_ABRIDGE"));

  ros::NodeHandle param("~");
  string device_path;
  param.param("device", device_path, string("/dev/ttyUSB0"));
  usb.openUSBPort(device_path, baud);


  sleep(5);

  ros::NodeHandle aNH;

  if (argc >= 2)
  {
    published_name = argv[1];
    cout << "Welcome to the world of tomorrow " << published_name << "!  ABridge module started." << endl;
  }
  else
  {
    published_name = hostname;
    cout << "No Name Selected. Default is: " << published_name << endl;
  }

  finger_angle_publisher = aNH.advertise<geometry_msgs::QuaternionStamped>((published_name + "/fingerAngle/prev_cmd"), 10);
  wrist_angle_publisher = aNH.advertise<geometry_msgs::QuaternionStamped>((published_name + "/wristAngle/prev_cmd"), 10);
  imu_publisher = aNH.advertise<sensor_msgs::Imu>((published_name + "/imu"), 10);
  odom_publisher = aNH.advertise<nav_msgs::Odometry>((published_name + "/odom"), 10);
  sonar_left_publisher = aNH.advertise<sensor_msgs::Range>((published_name + "/sonarLeft"), 10);
  sonar_center_publisher = aNH.advertise<sensor_msgs::Range>((published_name + "/sonarCenter"), 10);
  sonar_right_publisher = aNH.advertise<sensor_msgs::Range>((published_name + "/sonarRight"), 10);
  info_log_publisher = aNH.advertise<std_msgs::String>("/infoLog", 1, true);
  heartbeat_publisher = aNH.advertise<std_msgs::String>((published_name + "/abridge/heartbeat"), 1, true);

  drive_control_subscriber = aNH.subscribe((published_name + "/driveControl"), 10, driveCommandHandler);
  finger_angle_subscriber = aNH.subscribe((published_name + "/fingerAngle/cmd"), 1, fingerAngleHandler);
  wrist_angle_subscriber = aNH.subscribe((published_name + "/wristAngle/cmd"), 1, wristAngleHandler);
  mode_subscriber = aNH.subscribe((published_name + "/mode"), 1, modeHandler);


  publish_timer = aNH.createTimer(ros::Duration(delta_time), serialActivityTimer);
  publish_heartbeat_timer = aNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

  imu.header.frame_id = published_name+"/base_link";

  odom.header.frame_id = published_name+"/odom";
  odom.child_frame_id = published_name+"/base_link";

  for (int i = 0; i < hist_array_length; i++)
  {
    ev_array[i] = 0;
    ey_array[i] = 0;
  }

  prev_drive_command_update_time = ros::Time::now();

  ros::spin();

  return EXIT_SUCCESS;
}

// Receives a left/right skid steer command to set the left and right
// PWM control for the robot wheels
void driveCommandHandler(const swarmie_msgs::Skid::ConstPtr& message)
{
  float left = message->left;
  float right = message->right;

  // Cap motor commands at 120. Experimentally determined that high values (tested 180 and 255) can cause 
  // the hardware to fail when the robot moves itself too violently.
  int max_motor_cmd = 120;

  // Check that the resulting motor commands do not exceed the specified safe maximum value
  if (left > max_motor_cmd)
  {
    left = max_motor_cmd;
  }
  else if (left < -max_motor_cmd)
  {
    left = -max_motor_cmd;
  }

  if (right > max_motor_cmd)
  {
    right = max_motor_cmd;
  }
  else if (right < -max_motor_cmd)
  {
    right = -max_motor_cmd;
  }

  int left_int = left;
  int right_int = right;

  sprintf(move_cmd, "v,%d,%d\n", left_int, right_int); // Format data for arduino into c string
  usb.sendData(move_cmd);                      // Send movement command to arduino over usb
  memset(&move_cmd, '\0', sizeof (move_cmd));   // Clear the movement command string
}


// The finger and wrist handlers receive gripper angle commands in floating point
// radians, write them to a string and send that to the arduino
// for processing.
void fingerAngleHandler(const std_msgs::Float32::ConstPtr& angle)
{
  // To throttle the message rate so we don't lose connection to the arduino
  usleep(min_usb_send_delay);

  char cmd[16]={'\0'};

  // Avoid dealing with negative exponents which confuse the conversion to string by checking if the angle is small
  if (angle->data < 0.01)
  {
    // 'f' indicates this is a finger command to the arduino
    sprintf(cmd, "f,0\n");
  }
  else
  {
    sprintf(cmd, "f,%.4g\n", angle->data);
  }
  usb.sendData(cmd);
  memset(&cmd, '\0', sizeof (cmd));
}

void wristAngleHandler(const std_msgs::Float32::ConstPtr& angle)
{
  // To throttle the message rate so we don't lose connection to the arduino
  usleep(min_usb_send_delay);

  char cmd[16]={'\0'};

  // Avoid dealing with negative exponents which confuse the conversion to string by checking if the angle is small
  if (angle->data < 0.01)
  {
    // 'w' indicates this is a wrist command to the arduino
    sprintf(cmd, "w,0\n");
  }
  else
  {
    sprintf(cmd, "w,%.4g\n", angle->data);
  }
  usb.sendData(cmd);
  memset(&cmd, '\0', sizeof (cmd));
}

void serialActivityTimer(const ros::TimerEvent& e)
{
  usb.sendData(data_cmd);
  parseData(usb.readData());
  publishRosTopics();
}

void publishRosTopics()
{
  finger_angle_publisher.publish(finger_angle);
  wrist_angle_publisher.publish(wrist_angle);
  imu_publisher.publish(imu);
  odom_publisher.publish(odom);
  sonar_left_publisher.publish(sonar_left);
  sonar_center_publisher.publish(sonar_center);
  sonar_right_publisher.publish(sonar_right);
}

void parseData(string str)
{
  istringstream oss(str);
  string sentence;

  while (getline(oss, sentence, '\n'))
  {
    istringstream wss(sentence);
    string word;

    vector<string> data_set;
    while (getline(wss, word, ','))
    {
      data_set.push_back(word);
    }

    if (data_set.size() >= 3 && data_set.at(1) == "1")
    {
      if (data_set.at(0) == "GRF")
      {
        finger_angle.header.stamp = ros::Time::now();
        finger_angle.quaternion = tf::createQuaternionMsgFromRollPitchYaw(atof(data_set.at(2).c_str()), 0.0, 0.0);
      }
      else if (data_set.at(0) == "GRW")
      {
        wrist_angle.header.stamp = ros::Time::now();
        wrist_angle.quaternion = tf::createQuaternionMsgFromRollPitchYaw(atof(data_set.at(2).c_str()), 0.0, 0.0);
      }
      else if (data_set.at(0) == "IMU")
      {
        imu.header.stamp = ros::Time::now();
        imu.linear_acceleration.x = atof(data_set.at(2).c_str());
        imu.linear_acceleration.y = 0; //atof(data_set.at(3).c_str());
        imu.linear_acceleration.z = atof(data_set.at(4).c_str());
        imu.angular_velocity.x = atof(data_set.at(5).c_str());
        imu.angular_velocity.y = atof(data_set.at(6).c_str());
        imu.angular_velocity.z = atof(data_set.at(7).c_str());
        imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(data_set.at(8).c_str()), atof(data_set.at(9).c_str()), atof(data_set.at(10).c_str()));
      }
      else if (data_set.at(0) == "ODOM")
      {
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x += atof(data_set.at(2).c_str()) / 100.0;
        odom.pose.pose.position.y += atof(data_set.at(3).c_str()) / 100.0;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atof(data_set.at(4).c_str()));
        odom.twist.twist.linear.x = atof(data_set.at(5).c_str()) / 100.0;
        odom.twist.twist.linear.y = atof(data_set.at(6).c_str()) / 100.0;
        odom.twist.twist.angular.z = atof(data_set.at(7).c_str());
      }
      else if (data_set.at(0) == "USL")
      {
        sonar_left.header.stamp = ros::Time::now();
        sonar_left.range = atof(data_set.at(2).c_str()) / 100.0;
      }
      else if (data_set.at(0) == "USC")
      {
        sonar_center.header.stamp = ros::Time::now();
        sonar_center.range = atof(data_set.at(2).c_str()) / 100.0;
      }
      else if (data_set.at(0) == "USR")
      {
        sonar_right.header.stamp = ros::Time::now();
        sonar_right.range = atof(data_set.at(2).c_str()) / 100.0;
      }
    }
  }
}



void modeHandler(const std_msgs::UInt8::ConstPtr& message)
{
  current_mode = message->data;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "";
  heartbeat_publisher.publish(msg);
}
