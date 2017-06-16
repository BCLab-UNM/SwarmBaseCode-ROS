#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H

#include "StandardVars.h"
#include "PID.h"

class DriveController : virtual Controller
{
public:
    DriveController();
    ~DriveController();

    void Reset() override;
    Result DoWork() override;
    bool ShouldInterrupt() override;
    bool HasWork() override;

    void setResultData(Result result) {this->result = result;}

private:

    Result result;

    float left;
    float right;

    void ProcessData();

    //PID configs************************
    PIDConfig fastVelConfig();
    PIDConfig fastYawConfig();
    PIDConfig slowVelConfig();
    PIDConfig slowYawConfig();
    PIDConfig constVelConfig();
    PIDConfig constYawConfig();

    void fastPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw);
    void slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw);
    void constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw);

    PID fastVelPID(fastVelConfig());
    PID fastYawPID(fastYawConfig());

    PID slowVelPID(slowVelConfig());
    PID slowYawPID(slowYawConfig());

    PID constVelPID(constVelConfig());
    PID constYawPID(constYawConfig());


};

#endif // DRIVECONTROLLER_H
