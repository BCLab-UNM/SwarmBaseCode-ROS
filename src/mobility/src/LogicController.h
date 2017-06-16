#ifndef LOGICCONTROLLER_H
#define LOGICCONTROLLER_H

#include "Controller.h"
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"
#include "ObstacleController.h"
#include "DriveController.h"

#include <vector>
#include <queue>

using namespace std;

struct PrioritizedController {
    int priority = -1;
    Controller* controller = nullptr;

    inline bool operator <(const PrioritizedController& other) {
        return priority < other.priority;
    }
};

class LogicController : virtual Controller
{
public:
    LogicController();
    ~LogicController();

    void Reset() override;
    Result DoWork() override;
    void UpdateData();
    bool ShouldInterrupt() override;
    bool HasWork() override;

protected:
    void ProcessData();

private:

    enum LogicState {
        LOGIC_STATE_INTERRUPT = 0,
        LOGIC_STATE_WAITING,
        LOGIC_STATE_PRECISION_COMMAND
    };

    enum ProcessState {
        _FIRST = 0,
        PROCCESS_STATE_SEARCHING = 0,
        PROCCESS_COLLECTING_TARGET,
        PROCCESS_STATE_TARGET_PICKEDUP,
        _LAST
    };

    LogicState logicState;
    ProcessState processState;

    PickUpController pickUpController;
    DropOffController dropOffController;
    SearchController searchController;
    ObstacleController obstacleController;
    DriveController driveController;

    std::vector<PrioritizedController> prioritizedControllers;
    priority_queue<PrioritizedController> control_queue;
};

#endif // LOGICCONTROLLER_H
