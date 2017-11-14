#include "PickUpController.h"
#include <limits> // For numeric limits
#include <cmath> // For hypot

PickUpController::PickUpController()
{
  lockTarget = false;
  timeOut = false;
  nTargetsSeen = 0;
  blockYawError = 0;
  blockDistance = 0;

  targetFound = false;

  result.type = precisionDriving;
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError= 0;
  result.fingerAngle = -1;
  result.wristAngle = -1;
  result.PIDMode = SLOW_PID;
}

PickUpController::~PickUpController() { /*Destructor*/  }

void PickUpController::SetTagData(vector<Tag> tags)
{

  if (tags.size() > 0)
  {

    nTargetsSeen = tags.size();

    //we saw a target, set target_timer
    target_timer = current_time;

    double closest = std::numeric_limits<double>::max();
    int target  = 0;

    //this loop selects the closest visible block to makes goals for it
    for (int i = 0; i < tags.size(); i++)
    {

      if (tags[i].getID() == 0)
      {

        targetFound = true;

        //absolute distance to block from camera lens
        double test = hypot(hypot(tags[i].getPositionX(), tags[i].getPositionY()), tags[i].getPositionZ());

        if (closest > test)
        {
          target = i;
          closest = test;
        }
      }
      else
      {

        if(tags[i].getID() == 256)
        {

          Reset();

          if (has_control)
          {
            cout << "pickup reset return interupt free" << endl;
            release_control = true;
          }

          return;
        }
      }
    }

    float cameraOffsetCorrection = 0.023; //meters;

    ///TODO: Explain the trig going on here- blockDistance is c, 0.195 is b; find a
    blockDistanceFromCamera = hypot(hypot(tags[target].getPositionX(), tags[target].getPositionY()), tags[target].getPositionZ());

    if ( (blockDistanceFromCamera*blockDistanceFromCamera - 0.195*0.195) > 0 )
    {
      blockDistance = sqrt(blockDistanceFromCamera*blockDistanceFromCamera - 0.195*0.195);
    }
    else
    {
      float epsilon = 0.00001; // A small non-zero positive number
      blockDistance = epsilon;
    }

    //cout << "blockDistance  TAGDATA:  " << blockDistance << endl;

    blockYawError = atan((tags[target].getPositionX() + cameraOffsetCorrection)/blockDistance)*1.05; //angle to block from bottom center of chassis on the horizontal.

    cout << "blockYawError TAGDATA:  " << blockYawError << endl;

  }

}


bool PickUpController::SetSonarData(float rangeCenter)
{

  if (rangeCenter < 0.12 && targetFound)
  {
    result.type = behavior;
    result.b = nextProcess;
    result.reset = true;
    targetHeld = true;
    return true;
  }

  return false;

}

void PickUpController::ProcessData()
{

  if(!targetFound)
  {
    //cout << "PICKUP No Target Seen!" << endl;

    // Do nothing
    return;
  }

  //diffrence between current time and millisecond time
  long int Tdiff = current_time - millTimer;
  float Td = Tdiff/1e3;

  //cout << "PICKUP Target Seen!" << endl;

  //cout << "distance : " << blockDistanceFromCamera << " time is : " << Td << endl;

  if (blockDistanceFromCamera < 0.14 && Td < 3.9)
  {
    result.type = behavior;
    result.b = nextProcess;
    result.reset = true;
    targetHeld = true;
  }
  //Lower wrist and open fingures if no locked targt
  else if (!lockTarget)
  {
    //set gripper;
    result.fingerAngle = M_PI_2;
    result.wristAngle = 1.25;
  }
}


bool PickUpController::ShouldInterrupt(){

  ProcessData();

  if (release_control)
  {
    release_control = false;
    has_control = false;
    return true;
  }

  if ((targetFound && !interupted) || targetHeld)
  {
    interupted = true;
    has_control = false;
    return true;
  }
  else if (!targetFound && interupted)
  {
    interupted = false;
    has_control = false;
    return true;
  }
  else
  {
    return false;
  }
}

Result PickUpController::DoWork()
{

  has_control = true;

  if (!targetHeld)
  {
    //threshold distance to be from the target block before attempting pickup
    float targetDistance = 0.15; //meters

    // -----------------------------------------------------------
    // millisecond time = current time if not in a counting state
    //     when timeOut is true, we are counting towards a time out
    //     when timeOut is false, we are not counting towards a time out
    //
    // In summary, when timeOut is true, the robot is executing a pre-programmed time-based block pickup
    // I routine. <(@.@)/"
    // !!!!! AND/OR !!!!!
    // The robot has started a timer so it doesn't get stuck trying to pick up a cube that doesn't exist.
    //
    // If the robot does not see a block in its camera view within the time out period, the pickup routine
    // is considered to have FAILED.
    //
    // During the pre-programmed pickup routine, a current value of "Td" is used to progress through
    // the routine. "Td" is defined below...
    // -----------------------------------------------------------
    if (!timeOut) millTimer = current_time;

    //difference between current time and millisecond time
    long int Tdifference = current_time - millTimer;

    // converts from a millisecond difference to a second difference
    // Td = [T]ime [D]ifference IN SECONDS
    float Td = Tdifference/1e3;

    // The following nested if statement implements a time based pickup routine.
    // The sequence of events is:
    // 1. Target aquisition phase: Align the robot with the closest visible target cube, if near enough to get a target lock then start the pickup timer (Td)
    // 2. Approach Target phase: until *grasp_time_begin* seconds
    // 3. Stop and close fingers (hopefully on a block - we won't be able to see it remember): at *grasp_time_begin* seconds 
    // 4. Raise the gripper - does the rover see a block or did it miss it: at *raise_time_begin* seconds 
    // 5. If we get here the target was not seen in the robots gripper so drive backwards and and try to get a new lock on a target: at *target_require_begin* seconds
    // 6. If we get here we give up and release control with a task failed flag: for *target_pickup_task_time_limit* seconds
    
    // If we don't see any blocks or cubes turn towards the location of the last cube we saw.
    // I.E., try to re-aquire the last cube we saw.

    float grasp_time_begin = 1.5;
    float raise_time_begin = 2.0;
    float lower_gripper_time_begin = 4.0;
    float target_reaquire_begin= 4.2;
    float target_pickup_task_time_limit = 4.8;

    //cout << "blockDistance DOWORK:  " << blockDistance << endl;

    //Calculate time difference between last seen tag
    float target_timeout = (current_time - target_timer)/1e3;

    //delay between the camera refresh and rover runtime is 6/10's of a second
    float target_timeout_limit = 0.61;

    //Timer to deal with delay in refresh from camera and the runtime of rover code
    if( target_timeout >= target_timeout_limit )
    {
        //Has to be set back to 0
        nTargetsSeen = 0;
    }

    
    if (nTargetsSeen == 0 && !lockTarget)
    {
      // This if statement causes us to time out if we don't re-aquire a block within the time limit.
      if(!timeOut)
      {
        result.pd.cmdVel = 0.0;
        result.pd.cmdAngularError= 0.0;
        result.wristAngle = 1.25;
        // result.fingerAngle does not need to be set here

        // We are getting ready to start the pre-programmed pickup routine now! Maybe? <(^_^)/"
        // This is a guard against being stuck permanently trying to pick up something that doesn't exist.
        timeOut = true;

        // Rotate towards the block that we are seeing.
        // The error is negated in order to turn in a way that minimizes error.
        result.pd.cmdAngularError = -blockYawError;
      }
      //If in a counting state and has been counting for 1 second.
      else if (Td > 1.0 && Td < target_pickup_task_time_limit)
      {
        // The rover will reverse straight backwards without turning.
        result.pd.cmdVel = -0.15;
        result.pd.cmdAngularError= 0.0;
      }
    }
    else if (blockDistance > targetDistance && !lockTarget) //if a target is detected but not locked, and not too close.
    {
      // this is a 3-line P controller, where Kp = 0.20
      float vel = blockDistance * 0.20;
      if (vel < 0.1) vel = 0.1;
      if (vel > 0.2) vel = 0.2;

      result.pd.cmdVel = vel;
      result.pd.cmdAngularError = -blockYawError;
      timeOut = false;

      return result;
    }
    else if (!lockTarget) //if a target hasn't been locked lock it and enter a counting state while slowly driving forward.
    {
      lockTarget = true;
      result.pd.cmdVel = 0.18;
      result.pd.cmdAngularError= 0.0;
      timeOut = true;
      ignoreCenterSonar = true;
    }
    else if (Td > raise_time_begin) //raise the wrist
    {
      result.pd.cmdVel = -0.15;
      result.pd.cmdAngularError= 0.0;
      result.wristAngle = 0;
    }
    else if (Td > grasp_time_begin) //close the fingers and stop driving
    {
      result.pd.cmdVel = 0.0;
      result.pd.cmdAngularError= 0.0;
      result.fingerAngle = 0;
      return result;
    }


    // the magic numbers compared to Td must be in order from greater(top) to smaller(bottom) numbers
    if (Td > target_reaquire_begin && timeOut)
    {
      lockTarget = false;
      ignoreCenterSonar = true;
    }

    //if enough time has passed enter a recovery state to re-attempt a pickup
    else if (Td > lower_gripper_time_begin && timeOut)
    {
      result.pd.cmdVel = -0.15;
      result.pd.cmdAngularError= 0.0;
      //set gripper to open and down
      result.fingerAngle = M_PI_2;
      result.wristAngle = 0;
    }

    //if no targets are found after too long a period go back to search pattern
    if (Td > target_pickup_task_time_limit && timeOut)
    {
      Reset();
      interupted = true;
      result.pd.cmdVel = 0.0;
      result.pd.cmdAngularError= 0.0;
      ignoreCenterSonar = true;
    }
  }

  return result;
}

bool PickUpController::HasWork()
{
  return targetFound;
}

void PickUpController::Reset() {

  result.type = precisionDriving;
  result.PIDMode = SLOW_PID;
  lockTarget = false;
  timeOut = false;
  nTargetsSeen = 0;
  blockYawError = 0;
  blockDistance = 0;

  targetFound = false;
  interupted = false;
  targetHeld = false;

  result.pd.cmdVel = 0;
  result.pd.cmdAngularError= 0;
  result.fingerAngle = -1;
  result.wristAngle = -1;
  result.reset = false;

  ignoreCenterSonar = false;
}

void PickUpController::SetUltraSoundData(bool blockBlock){
  this->blockBlock = blockBlock;
}

void PickUpController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}
