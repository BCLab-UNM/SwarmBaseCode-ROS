#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H

#include "PID.h"
#include "Controller.h"
#include <angles/angles.h>

class DriveController : virtual Controller
{
public:
  DriveController();


  void reset() override;
  Result doWork() override;
  bool shouldInterrupt() override;
  bool hasWork() override;

  void setResultData(Result result) {this->result = result;}
  void setVelocityData(float linear_velocity,float angular_velocity);
  void setCurrentLocation(Point current_location) {this->current_location = current_location;}

  ~DriveController();
private:

  Result result;

  //MAX PWM is 255
  //abridge currently limits MAX to 120 to prevent overcurrent draw
  float left; //left wheels PWM value
  float right; //right wheels PWM value

  bool interrupt = false; //hold if interupt has occured yet

  float rotate_only_angle_tolerance = 0.05;  //May be too low?
  float final_rotation_tolerance = 0.1; //dead code not used
  const float waypoint_tolerance = 0.15; //15 cm tolerance.

  //0.65 MAX value
  float search_velocity = 0.35; // meters/second

  float linear_velocity = 0;
  float angular_velocity = 0;

  // Numeric Variables for rover positioning
  Point current_location;
  Point current_location_map;
  Point current_location_average;

  Point center_location;
  Point center_location_map;
  Point center_location_odom;

  vector<Point> waypoints;

  //PID configs************************
  PIDConfig fastVelConfig();
  PIDConfig fastYawConfig();
  PIDConfig slowVelConfig();
  PIDConfig slowYawConfig();
  PIDConfig constVelConfig();
  PIDConfig constYawConfig();

  void fastPID(float error_vel,float error_yaw, float set_point_vel, float set_point_yaw);
  void slowPID(float error_vel,float error_yaw, float set_point_vel, float set_point_yaw);
  void constPID(float error_vel,float const_angular_error, float set_point_vel, float set_point_yaw);

  //each PID movement paradigm needs at minimum two PIDs to acheive good robot motion.
  //one PID is for linear movement and the second for rotational movements
  PID fast_vel_PID;
  PID fast_yaw_PID;

  PID slow_vel_PID;
  PID slow_yaw_PID;

  PID const_vel_PID;
  PID const_yaw_PID;

  // state machine states
  enum StateMachineStates {

    //WAITING should not be handled- goes to default (it's a placeholder name)
    STATE_MACHINE_WAITING = 0,
    STATE_MACHINE_PRECISION_DRIVING,
    STATE_MACHINE_WAYPOINTS,
    STATE_MACHINE_ROTATE,
    STATE_MACHINE_SKID_STEER,
  };


  StateMachineStates state_machine_state = STATE_MACHINE_WAITING;

  void ProcessData();

};

#endif // DRIVECONTROLLER_H
