#include "LogicController.h"

LogicController::LogicController() {
    logicState = LOGIC_STATE_INTERRUPT;
    processState = PROCCESS_STATE_SEARCHING;

    prioritizedControllers = {
        PrioritizedController{0, (Controller*)(&searchController)},
        PrioritizedController{1, (Controller*)(&obstacleController)},
        PrioritizedController{2, (Controller*)(&pickUpController)},
        PrioritizedController{-1, (Controller*)(&dropOffController)}
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
        PrioritizedController{2, (Controller*)(&pickUpController)},
        PrioritizedController{-1, (Controller*)(&dropOffController)}
    };

    control_queue = priority_queue<PrioritizedController>();
}

Result LogicController::DoWork() {
    Result result;    

    for(PrioritizedController cntrlr : prioritizedControllers) {
        if(cntrlr.controller->ShouldInterrupt() && cntrlr.priority > 0) {
                logicState = LOGIC_STATE_INTERRUPT;
                cout << "shouldInterupt " << cntrlr.priority <<endl;
                //do not break all shouldInterupts may need calling in order to properly pre-proccess data.
        }
    }

    cout << "state " << processState << " logicState " << logicState << endl;

    switch(logicState) {
    case LOGIC_STATE_INTERRUPT: {

        //Reset the control queue
        control_queue = priority_queue<PrioritizedController>();

        for(PrioritizedController cntrlr : prioritizedControllers) {
            if(cntrlr.controller->HasWork()) {
                if (cntrlr.priority < 0) {
                    continue;
                }
                else {
                    control_queue.push(cntrlr);
                    cout << "hasWork" << endl;
                }
            }
        }

        if(control_queue.empty()) {
            result.type = behavior;
            result.b = wait;
            cout << "empty" << endl;
            break;
        }
        else {
            result.b = noChange;
        }

        result = control_queue.top().controller->DoWork();

        cout << "result type " << result.type << endl;

        if(result.type == behavior) {
            if (result.reset) {
                control_queue.top().controller->Reset();
            }
            if(result.b == nextProcess) {
                if (processState == _LAST - 1) {
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

            if (result.b == nextProcess || result.b == prevProcess) {
                ProcessData();
                result.b = wait;
            }
            break;
        } else if(result.type == precisionDriving) {

            logicState = LOGIC_STATE_PRECISION_COMMAND;
            break;

        } else if(result.type == waypoint) {

            cout << "waypoint" << endl;

            logicState = LOGIC_STATE_WAITING;
            driveController.setResultData(result);
        }

    }

    case LOGIC_STATE_WAITING: {

        cout << "waiting doing driving" << endl;

        result = driveController.DoWork();

        cout << "result type : " << result.type << endl;

        if (result.type == behavior) {
            if(driveController.ShouldInterrupt()) {
                logicState = LOGIC_STATE_INTERRUPT;
            }
        }
        break;
    }

    case LOGIC_STATE_PRECISION_COMMAND: {

        cout << "precision command"<< endl;

        result = control_queue.top().controller->DoWork();
        if (result.type == behavior) {
          logicState = LOGIC_STATE_INTERRUPT;
        }
        else {
          driveController.setResultData(result);
          result = driveController.DoWork();
          break;
        }
    }
    }

    controllerInterconnect();
    return result;
}

void LogicController::UpdateData() {


}

void LogicController::ProcessData() {

    if (processState == PROCCESS_STATE_SEARCHING) {
        prioritizedControllers = {
            PrioritizedController{0, (Controller*)(&searchController)},
            PrioritizedController{1, (Controller*)(&obstacleController)},
            PrioritizedController{2, (Controller*)(&pickUpController)},
            PrioritizedController{-1, (Controller*)(&dropOffController)}
        };
    }
    else if (processState  == PROCCESS_STATE_TARGET_PICKEDUP) {
        prioritizedControllers = {
            PrioritizedController{-1, (Controller*)(&searchController)},
            PrioritizedController{2, (Controller*)(&obstacleController)},
            PrioritizedController{-1, (Controller*)(&pickUpController)},
            PrioritizedController{1, (Controller*)(&dropOffController)}
        };
    }
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
    if(pickUpController.GetTargetHeld()) {
        dropOffController.SetTargetPickedUp();
        obstacleController.SetTargetHeld();
        driveController.Reset();
        cout << "drop: target held" << endl;
    }
    if (!dropOffController.HasTarget()) {
        obstacleController.SetTargetHeldClear();
    }

    if(obstacleController.GetShouldClearWaypoints()) {
      driveController.Reset();
    }
}


void LogicController::setPositionData(Point currentLocation) {
    searchController.setCurrentLocation(currentLocation);
    dropOffController.SetCurrentLocation(currentLocation);
    obstacleController.SetCurrentLocation(currentLocation);
    driveController.SetCurrentLocation(currentLocation);
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

void LogicController::setCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
  dropOffController.setCurrentTimeInMilliSecs( time );
  pickUpController.setCurrentTimeInMilliSecs( time );
  obstacleController.setCurrentTimeInMilliSecs( time );
}
