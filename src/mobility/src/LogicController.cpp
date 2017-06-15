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

    switch(logicState) {
    case LOGIC_STATE_INTERRUPT: {

        //Reset the control queue
        control_queue = priority_queue<PrioritizedController>();

        for(PrioritizedController cntrlr : prioritizedControllers) {
            if(cntrlr.controller->ShouldBePolled()) {
                control_queue.push(cntrlr);
            }
        }

        if(control_queue.empty()) {
            result.type = behavior;
            result.b = wait;

            return result;
        }

        result = control_queue.top().controller->CalculateResult();

        if(result.type == behavior) {
            if(result.b == nextProcess) {

            } else if(result.b == prevProcess) {

            }
        } else if(result.type == precisionDriving) {

            logicState = LOGIC_STATE_PRECISION_COMMAND;

        } else if(result.type == waypoint) {

            logicState = LOGIC_STATE_WAITING;


        }

        break;
    }

    case LOGIC_STATE_WAITING: {



        break;
    }

    case LOGIC_STATE_PRECISION_COMMAND: {

        result = control_queue.top().controller->CalculateResult();

        break;
    }
    }
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
