#include "DriveController.h"

DriveController::DriveController()
{

fastVelPID.setConfiguration(fastVelConfig());
fastyawPID.setConfiguration(fastYawConfig());

slowVelPID.setConfiguration(slowVelConfig());
slowYawPID.setConfiguration(slowYawConfig());

constVelPID.setConfiguration(constVelConfig();
constYawPID.setConfiguration(constYawConfig());


}

void DriveController::Reset() {

}

Result DriveController::DoWork() {

    resultHandler();

    result.type = precisionDriving;
    result.pd.cmdAngular = right;
    result.pd.cmdVel = left;

}

bool DriveController::ShouldInterrupt() {

}

bool DriveController::HasWork() {

}


void DriveController::resultHandler()
{
        std_msgs::Float32 angle;
    if (result.wristAngle != -1) {
        angle.data = result.wristAngle;
        wristAnglePublish.publish(angle);
    }
    if (result.fingerAngle != -1) {
        angle.data = result.fingerAngle;
        fingerAnglePublish.publish(angle);
    }

    if (result.type == waypoint) {

        if(result.reset) {
            waypoints.clear();
        }

        if (!result.wpts.waypoints.empty()) {
            waypoints.insert(waypoints.end(),result.wpts.waypoints.begin(), result.wpts.waypoints.end());
            waypointsAvalible = true;
            stateMachineState = STATE_MACHINE_WAYPOINTS;
        }
    }
    else if (result.type == precisionDriving) {
        if (result.PIDMode == FAST_PID){
            float vel = result.pd.cmdVel -linearVelocity;
            fastPID(vel,result.pd.cmdAngularError, result.pd.setPointVel, result.pd.setPointYaw); //needs declaration
        }
        else if (result.PIDMode == SLOW_PID) {
            //will take longer to reach the setPoint but has les chanse of an overshoot
            float vel = result.pd.cmdVel -linearVelocity;
            slowPID(vel,result.pd.cmdAngularError, result.pd.setPointVel, result.pd.setPointYaw);
        }
        else if (result.PIDMode == CONST_PID) {
            float vel = result.pd.cmdVel - linearVelocity;
            float angular = result.pd.cmdAngular - angularVelocity;

            constPID(vel, angular ,result.pd.setPointVel, result.pd.setPointYaw);
        }
    }
}


void DriveController::fastPID(float errorVel, float errorYaw , float setPointVel, float setPointYaw) {

    float velOut = fastVelPID.PIDOut(errorVel, setPointVel);
    float yawOut = fastYawPID.PIDOut(errorYaw, setPointYaw);

    int left = velOut - yawOut;
    int right = velOut + yawOut;

    int sat = 255;
    if (left  >  sat) {left  =  sat;}
    if (left  < -sat) {left  = -sat;}
    if (right >  sat) {right =  sat;}
    if (right < -sat) {right = -sat;}

    this->left = left;
    this->right = right;
}

void DriveController::slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw) {

    float velOut = slowVelPID.PIDOut(errorVel, setPointVel);
    float yawOut = slowYawPID.PIDOut(errorYaw, setPointYaw);

    int left = velOut - yawOut;
    int right = velOut + yawOut;

    int sat = 255;
    if (left  >  sat) {left  =  sat;}
    if (left  < -sat) {left  = -sat;}
    if (right >  sat) {right =  sat;}
    if (right < -sat) {right = -sat;}

    this->left = left;
    this->right = right;
}

void DriveController::constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw) {

    float velOut = fastVelPID.PIDOut(erroVel, setPointVel);
    float yawOut = fastYawPID.PIDOut(constAngularError, setPointYaw);

    int left = velOut - yawOut;
    int right = velOut + yawOut;

    int sat = 255;
    if (left  >  sat) {left  =  sat;}
    if (left  < -sat) {left  = -sat;}
    if (right >  sat) {right =  sat;}
    if (right < -sat) {right = -sat;}

    this->left = left;
    this->right = right;
}



PIDConfig DriveController::fastVelConfig() {
    PIDConfig config;

    config.Kp = 140;
    config.Ki = 10;
    config.Kd = 0.8;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/2;
    config.errorHistLength = 4;
    config.alwaysIntegral = true;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    config.derivativeAlpha = 0.7;

    return config;

}

PIDConfig DriveController::fastYawConfig() {
    PIDConfig config;

    config.Kp = 200;
    config.Ki = 25;
    config.Kd = 0.5;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/2;
    config.errorHistLength = 4;
    config.alwaysIntegral = false;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 0; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    config.derivativeAlpha = 0.7;

    return config;

}

PIDConfig DriveController::slowVelConfig() {
    PIDConfig config;

    config.Kp = 100;
    config.Ki = 8;
    config.Kd = 1.1;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/2;
    config.errorHistLength = 4;
    config.alwaysIntegral = true;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    config.derivativeAlpha = 0.7;

    return config;

}

PIDConfig DriveController::slowYawConfig() {
    PIDConfig config;

    config.Kp = 100;
    config.Ki = 15;
    config.Kd = 1.5;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/4;
    config.errorHistLength = 4;
    config.alwaysIntegral = false;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 0; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/6;
    config.derivativeAlpha = 0.7;

    return config;

}

PIDConfig DriveController::constVelConfig() {
    PIDConfig config;

    config.Kp = 140;
    config.Ki = 10;
    config.Kd = 0.8;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/2;
    config.errorHistLength = 4;
    config.alwaysIntegral = true;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    config.derivativeAlpha = 0.7;

    return config;

}

PIDConfig DriveController::constYawConfig() {
    PIDConfig config;

    config.Kp = 100;
    config.Ki = 5;
    config.Kd = 1.2;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/4;
    config.errorHistLength = 4;
    config.alwaysIntegral = true;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 120; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    config.derivativeAlpha = 0.6;

    return config;

}
