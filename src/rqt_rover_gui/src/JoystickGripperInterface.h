#ifndef JOYSTICKGRIPPERCONTROL_H
#define JOYSTICKGRIPPERCONTROL_H

#include <QTimer>
#include <exception>
#ifndef Q_MOC_RUN
#include <ros/ros.h> // For ROS publishers
#endif
#include <string> // For the rover name

// This file recieves input from a Microsoft Xbox 360 compatable joystick
// and sends actuation commands to the gripper ROS topics.
// A local autorepeat is implemented so that when users are holding the control
// stick at a constant position the gripper continues to move.
// This meets user expectations about how joysticks should work without
// enabling autorepeat at the ROS driver level. Autorepeat at the driver level
// causes high CPU usage and causes the drive joystick to also repeat (where it is not
// desirable).
// This class stores the current commanded angles

// Define an excetption to throw if the interface has not finished initializing when a move
// function is called. Prevents the user from trying to use uninitialized objects
// such as the timers.
class JoystickGripperInterfaceNotReadyException: public std::exception {
  virtual const char* what() const throw() {
    return "The joystick-gripper interface is not ready";
  }
};

class JoystickGripperInterface : public QObject // inherits from QObject for QT integration
{
    Q_OBJECT // QT integration macro

public:

    JoystickGripperInterface(); // Default constructor
    JoystickGripperInterface(const JoystickGripperInterface &source); // Copy constructor
    // This constructor must be called before use
    JoystickGripperInterface(ros::NodeHandle, std::string roverName);

    ~JoystickGripperInterface();

    // Moves the the wrist joint in the direction indicated by the joystick output
    // Directions are "up" and "down" represented by a 1D direction vector from the joystick.
    // with sign indicating direction and value indicating magnitude. The larger
    // the magnitude the faster the gripper will move in the direction determined by the sign.
    // Throws: interface not ready exception.
    void moveWrist(float vec);


    // Moves the finger joints in the direction indicated by the joysick output
    // Directions are "open" and "close" represented by a 1D direction vector from the joystick.
    // The vector is a float with sign indicating direction and value indicating magnitude.
    // The larger the magnitude of the vector the faster the fingers will move in the direction
    // determined by the vector sign.
    // Throws: interface not ready exception.
    void moveFingers(float vec);

    void changeRovers(std::string roverName);

signals:

    void sendJoystickGripperWristControlTimerStart(int);
    void sendJoystickGripperWristControlTimerStop();
    void sendJoystickGripperFingerControlTimerStart(int);
    void sendJoystickGripperFingerControlTimerStop();

private slots:

private:

    // Publishers for sending gripper commands to the ROS topics
    ros::Publisher gripperWristAnglePublisher;
    ros::Publisher gripperFingerAnglePublisher;

    // Joystick gripper controller state
    float wristAngle, wristAngleChangeRate, wristAngleMax, wristAngleMin, wristJoystickVector;

    // The finger movements are symmetric so we just refer to one finger angle etc
    float fingerAngle, fingerAngleChangeRate, fingerAngleMax, fingerAngleMin, fingerJoystickVector;

    std::string roverName;

    ros::NodeHandle nh;

    bool ready; // Not in constructor or destructor. Used by move functions.
};

#endif // JOYSTICKGRIPPERCONTROL_H
