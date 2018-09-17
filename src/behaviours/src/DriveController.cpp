#include "DriveController.h"

DriveController::DriveController() {

  fast_vel_PID.SetConfiguration(fastVelConfig());
  fast_yaw_PID.SetConfiguration(fastYawConfig());

  slow_vel_PID.SetConfiguration(slowVelConfig());
  slow_yaw_PID.SetConfiguration(slowYawConfig());

  const_vel_PID.SetConfiguration(constVelConfig());
  const_vaw_PID.SetConfiguration(constYawConfig());


}

void DriveController::reset()
{
  waypoints.clear();

  if (state_machine_state == STATE_MACHINE_ROTATE || state_machine_state == STATE_MACHINE_SKID_STEER)
  {
    state_machine_state = STATE_MACHINE_WAYPOINTS;
  }
}

Result DriveController::doWork()
{
  
  ///WARNING waypoint input must use FAST_PID at this point in time failure to set fast pid will result in no movment

  if(result.type == behavior)
  {
    if(result.b == noChange)
    {
      //if drive controller gets a no change command it is allowed to continue its previous action
      //normally this will be to follow waypoints but it is not specified as such.
    }

    else if(result.b == wait)
    {
      //do nothing till told otherwise
      left = 0.0;
      right = 0.0;
      state_machine_state = STATE_MACHINE_WAITING;
    }

  }

  else if(result.type == precisionDriving)
  {

    //interpret input result as a precision driving command
    state_machine_state = STATE_MACHINE_PRECISION_DRIVING;

  }

  else if(result.type == waypoint)
  {
    //interpret input result as new waypoints to add into the queue
    processData();

  }

  switch(state_machine_state)
  {

  //Handlers and the final state of STATE_MACHINE are the only parts allowed to call INTERUPT
  //This should be d one as little as possible. I suggest using timeouts to set control bools to false.
  //Then only call INTERUPT if bool switches to true.
  case STATE_MACHINE_PRECISION_DRIVING:
  {

    processData();
    break;
  }


  case STATE_MACHINE_WAYPOINTS:
  {

    //Handles route planning and navigation as well as making sure all waypoints are valid.

    bool too_close = true;
    //while we have waypoints and they are tooClose to drive to
    while (!waypoints.empty() && too_close)
    {
      //check next waypoint for distance
      if (hypot(waypoints.back().x-current_location.x, waypoints.back().y-current_location.y) < waypoint_tolerance)
      {
        //if too close remove it
        waypoints.pop_back();
      }
      else
      {
        //this waypoint is far enough to be worth driving to
        too_close = false;
      }
    }
    
    //if we are out of waypoints then interupt and return to logic controller
    if (waypoints.empty())
    {
      state_machine_state = STATE_MACHINE_WAITING;
      result.type = behavior;
      interrupt = true;
      return result;
    }
    else
    {
      //select setpoint for heading and begin driving to the next waypoint
      state_machine_state = STATE_MACHINE_ROTATE;
      waypoints.back().theta = atan2(waypoints.back().y - current_location.y, waypoints.back().x - current_location.x);
      result.pd.set_point_yaw = waypoints.back().theta;

      //cout << "**************************************************************************" << endl; //DEBUGGING CODE
      //cout << "Waypoint x : " << waypoints.back().x << " y : " << waypoints.back().y << endl; //DEBUGGING CODE
      //fall through on purpose
    }
  }

  case STATE_MACHINE_ROTATE:
  {

    // Calculate angle between currentLocation.theta and waypoints.front().theta
    // Rotate left or right depending on sign of angle
    // Stay in this state until angle is minimized

    waypoints.back().theta = atan2(waypoints.back().y - current_location.y, waypoints.back().x - current_location.x);

    // Calculate the difference between current and desired heading in radians.
    float error_yaw = angles::shortest_angular_distance(current_location.theta, waypoints.back().theta);

    //cout << "ROTATE Error yaw:  " << error_yaw << " target heading : " << waypoints.back().theta << " current heading : " << current_lcation.theta << endl; //DEBUGGING CODE
    //cout << "Waypoint x : " << waypoints.back().x << " y : " << waypoints.back().y << " currentLoc x : " << current_location.x << " y : " << current_location.y << endl; //DEBUGGING CODE

    result.pd.set_point_vel = 0.0;
    //Calculate absolute value of angle

    float abs_error = fabs(angles::shortest_angular_distance(current_location.theta, waypoints.back().theta));

    // If angle > rotate_only_angle_tolerance radians rotate but dont drive forward.
    if (abs_error > rotate_only_angle_tolerance)
    {
      // rotate but dont drive.
      if (result.PIDMode == FAST_PID)
      {
        fastPID(0.0, error_yaw, result.pd.set_point_vel, result.pd.set_point_yaw);
      }

      break;
    }
    else
    {
      //move to differential drive step
      state_machine_state = STATE_MACHINE_SKID_STEER;

      //fall through on purpose.
    }
  }

  case STATE_MACHINE_SKID_STEER:
  {
      // Calculate angle between current_location.x/y and waypoints.back().x/y
      // Drive forward
      // Stay in this state until angle is at least PI/2

    // calculate the distance between current and desired heading in radians
    waypoints.back().theta = atan2(waypoints.back().y - current_location.y, waypoints.back().x - current_location.x);
    float error_yaw = angles::shortest_angular_distance(current_location.theta, waypoints.back().theta);
    float distance = hypot(waypoints.back().x - current_location.x, waypoints.back().y - current_location.y);

    //cout << "Skid steer, Error yaw:  " << error_yaw << " target heading : " << waypoints.back().theta << " current heading : " << current_location.theta << " error distance : " << distance << endl; //DEBUGGING CODE
    //cout << "Waypoint x : " << waypoints.back().x << " y : " << waypoints.back().y << " currentLoc x : " << current_location.x << " y : " << current_location.y << endl; //DEBUGGING CODE



    // goal not yet reached drive while maintaining proper heading.
    if (fabs(error_yaw) < M_PI_2 &&  distance > waypoint_tolerance)
    {
      // drive and turn simultaniously
      result.pd.set_point_vel = search_velocity;
      if (result.PIDMode == FAST_PID)
      {
        //cout << "linear velocity:  " << linear_velocity << endl; //DEBUGGING CODE
        fastPID((search_velocity-linear_velocity) ,error_yaw, result.pd.set_point_vel, result.pd.set_point_yaw);
      }
    }
    else {
      // stopno change
      left = 0.0;
      right = 0.0;

      // move back to transform step
      state_machine_state = STATE_MACHINE_WAYPOINTS;
    }

    break;
  }

  default:
  {
    break;
  }


  }

  //package data for left and right values into result struct for use in ROSAdapter
  result.pd.right = right;
  result.pd.left = left;

  //return modified struct
  return result;

}

bool DriveController::shouldInterrupt()
{
  if (interrupt)
  {
    interrupt = false;
    return true;
  }
  else
  {
    return false;
  }
}

bool DriveController::hasWork() {   }



void DriveController::processData()
{
  //determine if the drive commands are waypoint or precision driving
  if (result.type == waypoint) {
    
    //sets logic controller into stand by mode while drive controller works
    result.type = behavior;
    result.b = noChange;

    if(result.reset) {
      waypoints.clear();
    }

    //add waypoints onto stack and change state to start following them
    if (!result.wpts.waypoints.empty()) {
      waypoints.insert(waypoints.end(),result.wpts.waypoints.begin(), result.wpts.waypoints.end());
      state_machine_state = STATE_MACHINE_WAYPOINTS;
    }
  }
  else if (result.type == precisionDriving)
  {

    //calculate inputs into the PIDS for precision driving
    if (result.PID_mode == FAST_PID)
    {
      float vel = result.pd.cmd_vel - linear_velocity;
      float set_vel = result.pd.cmd_vel;
      fastPID(vel,result.pd.cmd_angular_error, set_vel, result.pd.set_point_yaw);
    }
    else if (result.PID_mode == SLOW_PID)
    {
      //will take longer to reach the setPoint but has less chanse of an overshoot especially with slow feedback
      float vel = result.pd.cmd_vel - linear_velocity;
      float setVel = result.pd.cmd_vel;
      slowPID(vel,result.pd.cmd_angular_error, set_vel, result.pd.set_point_yaw);
    }
    else if (result.PID_mode == CONST_PID)
    {
      //vel is the same as fast PID however
      //this setup takes a target angular velocity to constantly turn at instead of a target heading
      float vel = result.pd.cmd_vel - linear_velocity;
      float angular = result.pd.cmd_angular - angular_velocity;

      //cout << "Ang. Vel.  " << angular_velocity << "  Ang. Error" << angular << endl; //DEBUGGING CODE

      constPID(vel, angular ,result.pd.set_point_vel, result.pd.set_point_yaw);
    }
  }
}


void DriveController::fastPID(float error_vel, float error_yaw , float set_point_vel, float set_point_yaw)
{

  // cout << "PID FAST" << endl; //DEBUGGING CODE

  float vel_out = fast_vel_PID.PIDOut(error_vel, set_point_vel); //returns PWM target to try and get error vel to 0
  float yaw_out = fast_yaw_PID.PIDOut(error_yaw, set_point_yaw); //returns PWM target to try and get yaw error to 0

  int left = vel_out - yaw_out; //combine yaw and vel PWM values
  int right = vel_out + yaw_out; //left and right are the same for vel output but opposite for yaw output

  //prevent combine output from going over tihs value
  int sat = 180; 
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}
  this->left = left;
  this->right = right;
}

void DriveController::slowPID(float error_vel,float error_yaw, float set_point_vel, float set_point_yaw)
{
  //cout << "PID SLOW" << endl; //DEBUGGING CODE

  float vel_out = slow_vel_PID.PIDOut(error_vel, set_point_vel);
  float yaw_out = slow_yaw_PID.PIDOut(error_yaw, set_point_yaw);

  int left = vel_out - yaw_out;
  int right = vel_out + yaw_out;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}

void DriveController::constPID(float error_vel,float const_angular_error, float set_point_vel, float set_point_yaw)
{

  //cout << "PID CONST" << endl; //DEBUGGING CODE

  float vel_out = const_vel_PID.PIDOut(error_vel, set_point_vel);
  float yaw_out = const_yaw_PID.PIDOut(const_angular_error, set_point_yaw);

  int left = vel_out - yaw_out;
  int right = vel_out + yaw_out;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}


void DriveController::setVelocityData(float linear_velocity,float angular_velocity)
{
  this->linear_velocity = linear_velocity;
  this->angular_velocity = angular_velocity;
}




PIDConfig DriveController::fastVelConfig()
{
  PIDConfig config;

  config.Kp = 60; //proportional constant
  config.Ki = 10; //integral constant
  config.Kd = 2; //derivative constant
  config.sat_upper = 255; //upper limit for PID output
  config.sat_lower = -255; //lower limit for PID output
  config.anti_windup = config.satUpper; //prevent integral from acruing error untill proportional output drops below a certain limit
  config.error_hist_length = 4; //how many time steps to average error over
  config.always_integral = true; //should the integral alway be on or only when there is error
  config.reset_on_setpoint = true; //reset the integral and error history whent he setpoint changes
  config.feed_forward_multiplier = 610; //gives 127 pwm at 0.4 commandedspeed  ORIG:320
  config.integral_dead_zone = 0.01; //set the integral dead zone, prevented integral from growing or shrinking do to noise
  config.integral_error_history_length = 10000; //how many time ticks should error history should be stored for integration
  config.integral_max = config.sat_upper/2; //what is the limit of the integral output for the PID
  config.derivative_alpha = 0.7; //dead code not used

  return config;

}

PIDConfig DriveController::fastYawConfig() {
  PIDConfig config;

  config.Kp = 60;
  config.Ki = 15;
  config.Kd = 5;
  config.sat_upper = 255;
  config.sat_lower = -255;
  config.anti_windup = config.sat_upper/6;
  config.error_hist_length = 4;
  config.always_integral = false;
  config.reset_on_setpoint = true;
  config.feed_forward_multiplier = 0;
  config.integral_dead_zone = 0.01;
  config.integral_error_history_length = 10000;
  config.integral_max = config.sat_upper/3;
  config.derivative_alpha = 0.7;

  return config;

}

PIDConfig DriveController::slowVelConfig() {
  PIDConfig config;

  config.Kp = 100;
  config.Ki = 8;
  config.Kd = 1.1;
  config.sat_upper = 255;
  config.sat_lower = -255;
  config.anti_windup = config.sat_upper/2;
  config.error_hist_length = 4;
  config.always_integral = true;
  config.reset_on_setpoint = true;
  config.feed_forward_multiplier = 320; //gives 127 pwm at 0.4 commandedspeed
  config.integral_dead_zone = 0.01;
  config.integral_error_history_length = 10000;
  config.integral_max = config.sat_upper/2;
  config.derivative_alpha = 0.7;

  return config;

}

PIDConfig DriveController::slowYawConfig() {
  PIDConfig config;

  config.Kp = 70;
  config.Ki = 16;
  config.Kd = 10;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/4;
  config.errorHistLength = 4;
  config.alwaysIntegral = false;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/6;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::constVelConfig() {
  PIDConfig config;

  config.Kp = 60;
  config.Ki = 10;
  config.Kd = 2;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 610; //gives 127 pwm at 0.4 commandedspeed  ORIG:320
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::constYawConfig() {
  PIDConfig config;

  config.Kp = 5;
  config.Ki = 5;
  config.Kd = 0;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/4;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper;
  config.derivativeAlpha = 0.6;

  return config;

}

DriveController::~DriveController() {}
