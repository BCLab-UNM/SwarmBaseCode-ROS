#include "DropOffController.h"

DropOffController::DropOffController() {

    reachedCollectionPoint = false;

    result.type = behavior;
    result.b = init;

    circularCenterSearching = false;
    spinner = 0;
    centerApproach = false;
    lastCenterTagThresholdTime = time(0);
    seenEnoughCenterTags = false;
    prevCount = 0;

    isPrecisionDriving = false;
    startWaypoint = false;

}

DropOffController::~DropOffController() {

}

Result DropOffController::CalculateResult() {

    //if we are in the routine for exciting the circle once we have dropped a block off and reseting all our flags
    //to resart our search.
    if(reachedCollectionPoint)
    {
        if (timerTimeElapsed >= 4)
        {
            result.type = behavior;
            result.b = targetReturned;
            Reset();
            return result;
        }
        else if (timerTimeElapsed >= 1)
        {
            isPrecisionDriving = true;
            result.type = precisionDriving;

            //open fingers
            float angle;
            angle = M_PI_2;
            result.fingerAngle = angle;
            angle= 0;
            result.wristAngle = angle; //raise wrist

            result.pd.cmdVel = -0.3;
            result.pd.cmdAngularError = 0.0;
        }

        return result;
    }

    double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);
    int count = countLeft + countRight;

    //check to see if we are driving to the center location or if we need to drive in a circle and look.
    if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && count == 0) {

        result.type = waypoint;
        result.wpts.waypoints.push_back(centerLocation);
        startWaypoint = false;
        isPrecisionDriving = false;

    }
    else if (timerTimeElapsed >=5)//spin search for center
    {
        geometry_msgs::Pose2D nextSpinPoint;

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
    }

    bool left = (countLeft > 0);
    bool right = (countRight > 0);
    bool centerSeen = (right || left);

    //reset lastCenterTagThresholdTime timout timer to current time
    if ((!centerApproach && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {

        lastCenterTagThresholdTime = time(0);

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
            lastCenterTagThresholdTime = time(0);
        }
        if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
        {
            lastCenterTagThresholdTime = time(0);
        }
        //time since we dropped below countGuard tags
        time_t timeSinceSeeingEnoughCenterTags = time(0) - lastCenterTagThresholdTime;

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

        time_t timeSinceSeeingEnoughCenterTags = time(0) - lastCenterTagThresholdTime;
        if (timeSinceSeeingEnoughCenterTags > centerTagSearchCutoff)
        {
            //go back to drive to center base location instead of drop off attempt
            reachedCollectionPoint = false;
            seenEnoughCenterTags = false;
            centerApproach = false;

            result.type = waypoint;
            result.wpts.waypoints.push_back(centerLocation);
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
        returnTimer = time(0);
    }

    return result;
}

void DropOffController::Reset() {
    result.pd.cmdVel = 0;
    result.pd.cmdAngularError = 0;
    result.fingerAngle = -1;
    result.wristAngle = -1;
    spinner = 0;
    spinSizeIncrease = 0;
    prevCount = 0;

    //reset flags
    reachedCollectionPoint = false;
    seenEnoughCenterTags = false;
    circularCenterSearching = false;

}

void DropOffController::UpdateData(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
    countRight = 0;
    countLeft = 0;

    if(targetHeld) {
        // if a target is detected and we are looking for center tags
        if (message->detections.size() > 0 && !reachedCollectionPoint) {

            // this loop is to get the number of center tags
            for (int i = 0; i < message->detections.size(); i++) {
                if (message->detections[i].id == 256) {
                    geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

                    // checks if tag is on the right or left side of the image
                    if (cenPose.pose.position.x + cameraOffsetCorrection > 0) {
                        countRight++;

                    } else {
                        countLeft++;
                    }
                }
            }
        }

        if((countLeft + countRight) > 0) {
            isPrecisionDriving = true;
        } else {
            startWaypoint = true;
        }
    }

}

bool DropOffController::ShouldInterrupt() {
    return startWaypoint;
}

bool DropOffController::IsChangingMode() {
    return isPrecisionDriving;
}

void DropOffController::SetLocationData(geometry_msgs::Pose2D center, geometry_msgs::Pose2D current) {
    centerLocation = center;
    currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
    targetHeld = true;
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
    targetHeld = targetHeld || blockBlock;
}
