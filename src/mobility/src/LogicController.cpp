#include "LogicController.h"

LogicController::LogicController() {
    logicState = LOGIC_STATE_INTERRUPT;
    processState = PROCCESS_STATE_SEARCHING;

    prioritizedControllers = {
        PrioritizedController{0, (Controller*)(&searchController)},
        PrioritizedController{1, (Controller*)(&obstacleController)},
        PrioritizedController{2, (Controller*)(&pickUpController)}
    };

    control_queue = priority_queue<PrioritizedController>();

}

LogicController::~LogicController() {}

void LogicController::Reset() {

    logicState = LOGIC_STATE_INTERRUPT;
    processState = PROCCESS_STATE_SEARCHING;

    prioritizedControllers = {
        PrioritizedController{0, (Controller*)(&searchController)},
        PrioritizedController{1, (Controller*)(&obstacleController)},
        PrioritizedController{2, (Controller*)(&pickUpController)}
    };

    control_queue = priority_queue<PrioritizedController>();
}

Result LogicController::DoWork() {
    Result result;


    if (processState == PROCCESS_STATE_SEARCHING) {

    }
    else if (processState == PROCCESS_COLLECTING_TARGET) {

    }
    else if (processState  == PROCCESS_STATE_TARGET_PICKEDUP) {

    }


    switch(logicState) {
    case LOGIC_STATE_INTERRUPT: {

        //Reset the control queue
        control_queue = priority_queue<PrioritizedController>();

        for(PrioritizedController cntrlr : prioritizedControllers) {
            if(cntrlr.controller->ShouldInterrupt()) {
                control_queue.push(cntrlr);
            }
        }

        if(control_queue.empty()) {
            result.type = behavior;
            result.b = wait;

            return result;
        }

        result = control_queue.top().controller->DoWork();

        if(result.type == behavior) {
            if(result.b == nextProcess) {
                if (processState == _LAST) {
                    processState = _FIRST;
                }
                else {
                processState = (ProcessState)((int)processState + 1);
                }
            } else if(result.b == prevProcess) {
                if (processState == _FIRST) {
                    processState = (ProcessState)((int)_LAST - 1);
                }
                else {
                processState = (ProcessState)((int)processState - 1);
                }
            }
        } else if(result.type == precisionDriving) {

            logicState = LOGIC_STATE_PRECISION_COMMAND;

        } else if(result.type == waypoint) {

            logicState = LOGIC_STATE_WAITING;
            result = control_queue.top().controller->DoWork();

        }

        break;
    }

    case LOGIC_STATE_WAITING: {

        result = driveController.DoWork();
        if (result.type = behavior) {
            if(driveController.ShouldInterrupt()) {
                logicState = LOGIC_STATE_INTERRUPT;
            }
            else {
                return result;
            }
        }
        else {
            return result;
        }

        break;
    }

    case LOGIC_STATE_PRECISION_COMMAND: {

        result = control_queue.top().controller->DoWork();
        driveController.setResultData(result);
        result = driveController.DoWork();
        break;
    }
    }

    return result;
}

void LogicController::UpdateData() {


}

void LogicController::ProcessData() {

}

bool LogicController::ShouldInterrupt() {
    ProcessData();

    return false;
}

bool LogicController::HasWork() {
    return false;
}


void LogicController::controllerInterconnect() {


    if(pickUpController.GetIgnoreCenter()) {
        obstacleController.SetIgnoreCenter();
    }
}


void LogicController::setPositionData(Point currentLocation) {
    searchController.setCurrentLocation(currentLocation);
    dropOffController.SetCurrentLocation(currentLocation);
    obstacleController.SetCurrentLocation(currentLocation);
}

void LogicController::setMapPositionData(Point currentLocationMap) {

}

void LogicController::setVelocityData(float linearVelocity, float angularVelocity) {
    driveController.SetVelocityData(linearVelocity,angularVelocity);
}

void LogicController::setMapVelocityData(float linearVelocity, float angularVelocity) {

}

void LogicController::setAprilTags(vector<TagPoint> tags) {
    pickUpController.SetTagData(tags);
    obstacleController.SetTagData(tags);
    dropOffController.setTargetData(tags);
}

void LogicController::setSonarData(float left, float center, float right) {
    pickUpController.SetSonarData(center);
    obstacleController.SetSonarData(left,center,right);
}

void LogicController::setCenterLocationOdom(Point centerLocationOdom) {
    searchController.setCenterLocation(centerLocationOdom);
    dropOffController.SetCenterLocation(centerLocationOdom);
}

void LogicController::setCenterLocationMap(Point centerLocationMap) {

}
