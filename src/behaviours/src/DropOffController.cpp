#include "DropOffController.h"

DropOffController::DropOffController() {

    reachedCollectionPoint = false;

    result.type = behavior;
    result.b = wait;
    result.wristAngle = 0.8;
    result.reset = false;

    circularCenterSearching = false;
    spinner = 0;
    centerApproach = false;
    seenEnoughCenterTags = false;
    prevCount = 0;

    countLeft = 0;
    countRight = 0;

    isPrecisionDriving = false;
    startWaypoint = false;
    timerTimeElapsed = -1;

}

DropOffController::~DropOffController() {

}

Result DropOffController::DoWork() {

    if(timerTimeElapsed > -1) {
       
	long int elapsed = current_time - returnTimer;
        float timeTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
    }

    //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
    //to resart our search.
    if(reachedCollectionPoint)
    {
        if (timerTimeElapsed >= 4)
        {
            result.type = behavior;
            result.b = nextProcess;
            result.reset = true;
            return result;
        }
        else if (timerTimeElapsed >= 0.5)
        {
            isPrecisionDriving = true;
            result.type = precisionDriving;

            result.fingerAngle = M_PI_2; //open fingers
            result.wristAngle = 0; //raise wrist

            result.pd.cmdVel = -0.3;
            result.pd.cmdAngularError = 0.0;
        }

        return result;
    }

    double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);
    int count = countLeft + countRight;

    //check to see if we are driving to the center location or if we need to drive in a circle and look.
    if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && (count == 0)) {

        result.type = waypoint;
        result.wpts.waypoints.push_back(this->centerLocation);
        startWaypoint = false;
        isPrecisionDriving = false;

        timerTimeElapsed = 0;

        return result;

    }
    else if (timerTimeElapsed >= 5)//spin search for center
    {
        Point nextSpinPoint;

        //sets a goal that is 60cm from the centerLocation and spinner
        //radians counterclockwise from being purly along the x-axis.
        nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
        nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
        nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);

        result.type = waypoint;
        result.wpts.waypoints.clear();
        result.wpts.waypoints.push_back(nextSpinPoint);

        spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
        if (spinner > 2*M_PI)
        {
            spinner -= 2*M_PI;
            spinSizeIncrease += spinSizeIncrement;
        }
        circularCenterSearching = true;
        //safety flag to prevent us trying to drive back to the
        //center since we have a block with us and the above point is
        //greater than collectionPointVisualDistance from the center.

        returnTimer = current_time;
        timerTimeElapsed = 0;

        return result;
    }

    bool left = (countLeft > 0);
    bool right = (countRight > 0);
    bool centerSeen = (right || left);

    //reset lastCenterTagThresholdTime timout timer to current time
    if ((!centerApproach && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {

        lastCenterTagThresholdTime = current_time;

    }

    if (count > 0 || seenEnoughCenterTags || prevCount > 0) //if we have a target and the center is located drive towards it.
    {
        centerSeen = true;

        if (seenEnoughCenterTags) //if we have seen enough tags
        {
            if ((countLeft-5) > countRight) //and there are too many on the left
            {
                right = false; //then we say none on the right to cause us to turn right
            }
            else if ((countRight-5) > countLeft)
            {
                left = false; //or left in this case
            }
        }

        float turnDirection = 1;
        //reverse tag rejection when we have seen enough tags that we are on a
        //trajectory in to the square we dont want to follow an edge.
        if (seenEnoughCenterTags) turnDirection = -1;

        result.type = precisionDriving;

        //otherwise turn till tags on both sides of image then drive straight
        if (left && right) {
            result.pd.cmdVel = searchVelocity;
            result.pd.cmdAngularError = 0.0;
        }
        else if (right) {
            result.pd.cmdVel = -0.1 * turnDirection;
            result.pd.cmdAngularError = -centeringTurn*turnDirection;
        }
        else if (left){
            result.pd.cmdVel = -0.1 * turnDirection;
            result.pd.cmdAngularError = centeringTurn*turnDirection;
        }
        else
        {
            result.pd.cmdVel = searchVelocity;
            result.pd.cmdAngularError = 0.0;
        }

        //must see greater than this many tags before assuming we are driving into the center and not along an edge.
        if (count > centerTagThreshold)
        {
            seenEnoughCenterTags = true; //we have driven far enough forward to be in the circle.
            lastCenterTagThresholdTime = current_time;
        }
        if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
        {
            lastCenterTagThresholdTime = current_time;
        }
        //time since we dropped below countGuard tags
        long int elapsed = current_time - lastCenterTagThresholdTime;
        float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds

        //we have driven far enough forward to have passed over the circle.
        if (count == 0 && seenEnoughCenterTags && timeSinceSeeingEnoughCenterTags > 1) {
            centerSeen = false;
        }
        centerApproach = true;
        prevCount = count;
        count = 0;
    }

    //was on approach to center and did not seenEnoughCenterTags
    //for centerTagSearchCutoff seconds so reset.
    else if (centerApproach) {

        long int elapsed = current_time - lastCenterTagThresholdTime;
        float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds
        if (timeSinceSeeingEnoughCenterTags > centerTagSearchCutoff)
        {
            //go back to drive to center base location instead of drop off attempt
            reachedCollectionPoint = false;
            seenEnoughCenterTags = false;
            centerApproach = false;

            result.type = waypoint;
            result.wpts.waypoints.push_back(this->centerLocation);
            isPrecisionDriving = false;
        }
        else
        {
            result.pd.cmdVel = searchVelocity;
            result.pd.cmdAngularError = 0.0;
        }

        return result;

    }

    if (!centerSeen && seenEnoughCenterTags)
    {
        reachedCollectionPoint = true;
        centerApproach = false;
        returnTimer = current_time;
    }

    return result;
}

void DropOffController::Reset() {
    result.pd.cmdVel = 0;
    result.pd.cmdAngularError = 0;
    result.fingerAngle = -1;
    result.wristAngle = 0.8;
    result.reset = false;
    spinner = 0;
    spinSizeIncrease = 0;
    prevCount = 0;
    timerTimeElapsed = -1;

    countLeft = 0;
    countRight = 0;


    //reset flags
    reachedCollectionPoint = false;
    seenEnoughCenterTags = false;
    circularCenterSearching = false;

}

void DropOffController::setTargetData(vector<TagPoint> tags) {
    countRight = 0;
    countLeft = 0;

    if(targetHeld) {
        // if a target is detected and we are looking for center tags
        if (tags.size() > 0 && !reachedCollectionPoint) {

            // this loop is to get the number of center tags
            for (int i = 0; i < tags.size(); i++) {
                if (tags[i].id == 256) {

                    // checks if tag is on the right or left side of the image
                    if (tags[i].x + cameraOffsetCorrection > 0) {
                        countRight++;

                    } else {
                        countLeft++;
                    }
                }
            }
        }
    }

}

void DropOffController::ProcessData() {
    if((countLeft + countRight) > 0) {
        isPrecisionDriving = true;
    } else {
        startWaypoint = true;
    }
}

bool DropOffController::ShouldInterrupt() {
    ProcessData();

    return startWaypoint;
}

bool DropOffController::HasWork() {
    return ((startWaypoint || isPrecisionDriving) && !circularCenterSearching);
}

bool DropOffController::IsChangingMode() {
    return isPrecisionDriving;
}

void DropOffController::SetCenterLocation(Point center) {
    centerLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
    currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
    targetHeld = true;
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
    targetHeld = targetHeld || blockBlock;
}

void DropOffController::setCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}
