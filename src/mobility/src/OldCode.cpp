float killSwitchTimeout = 10;
bool targetDetected = false;
bool targetCollected = false;


void mapAverage();  // constantly averages last 100 positions from map



// Set true when the target block is less than targetDist so we continue
// attempting to pick it up rather than switching to another block in view.
bool lockTarget = false;

// Failsafe state. No legitimate behavior state. If in this state for too long
// return to searching as default behavior.
bool timeOut = false;

// Set to true when the center ultrasound reads less than 0.14m. Usually means
// a picked up cube is in the way.
bool blockBlock = false;

// central collection point has been seen (aka the nest)
bool centerSeen = false;

// Set true when we are insie the center circle and we need to drop the block,
// back out, and reset the boolean cascade.
bool reachedCollectionPoint = false;

// used for calling code once but not in main
bool init = false;

// used to remember place in mapAverage array
int mapCount = 0;

// How many points to use in calculating the map average position
const unsigned int mapHistorySize = 500;

// An array in which to store map positions
geometry_msgs::Pose2D mapLocation[mapHistorySize];

bool avoidingObstacle = false;

float searchVelocity = 0.2; // meters/second










// instantiate random number generator
rng = new random_numbers::RandomNumberGenerator();

//set initial random heading
goalLocation.theta = rng->uniformReal(0, 2 * M_PI);

//select initial search position 50 cm from center (0,0)
goalLocation.x = 0.5 * cos(goalLocation.theta+M_PI);
goalLocation.y = 0.5 * sin(goalLocation.theta+M_PI);

centerLocation.x = 0;
centerLocation.y = 0;
centerLocationOdom.x = 0;
centerLocationOdom.y = 0;

for (int i = 0; i < 100; i++) {
    mapLocation[i].x = 0;
    mapLocation[i].y = 0;
    mapLocation[i].theta = 0;
}






std_msgs::String stateMachineMsg;
float rotateOnlyAngleTolerance = 0.4;
int returnToSearchDelay = 5;

// calls the averaging function, also responsible for
// transform from Map frame to odom frame.
mapAverage();




    // time since timerStartTime was set to current time
    timerTimeElapsed = time(0) - timerStartTime;

    // init code goes here. (code that runs only once at start of
    // auto mode but wont work in main goes here)
    if (!init) {
        if (timerTimeElapsed > startDelayInSeconds) {
            // Set the location of the center circle location in the map
            // frame based upon our current average location on the map.
            centerLocationMap.x = currentLocationAverage.x;
            centerLocationMap.y = currentLocationAverage.y;
            centerLocationMap.theta = currentLocationAverage.theta;

            // initialization has run
            init = true;
        } else {
            return;
        }

    }

    // If no collected or detected blocks set fingers
    // to open wide and raised position.
    if (!targetCollected && !targetDetected) {
        // set gripper
        std_msgs::Float32 angle;

        // open fingers
        angle.data = M_PI_2;

        fingerAnglePublish.publish(angle);
        angle.data = 0;

        // raise wrist
        wristAnglePublish.publish(angle);
    }

    // Select rotation or translation based on required adjustment
    

    // If no adjustment needed, select new goal
    case STATE_MACHINE_TRANSFORM: {
        stateMachineMsg.data = "TRANSFORMING";

        // If returning with a target
        if (targetCollected && !avoidingObstacle) {
            // calculate the euclidean distance between
            // centerLocation and currentLocation
            dropOffController.setCenterDist(hypot(centerLocation.x - currentLocation.x, centerLocation.y - currentLocation.y));
            dropOffController.setDataLocations(centerLocation, currentLocation, timerTimeElapsed);

            DropOffResult result = dropOffController.getState();

            if (result.timer) {
                timerStartTime = time(0);
                reachedCollectionPoint = true;
            }

            std_msgs::Float32 angle;

            if (result.fingerAngle != -1) {
                angle.data = result.fingerAngle;
                fingerAnglePublish.publish(angle);
            }

            if (result.wristAngle != -1) {
                angle.data = result.wristAngle;
                wristAnglePublish.publish(angle);
            }

            if (result.reset) {
                timerStartTime = time(0);
                targetCollected = false;
                targetDetected = false;
                lockTarget = false;
                sendDriveCommand(0.0,0);

                // move back to transform step
                stateMachineState = STATE_MACHINE_TRANSFORM;
                reachedCollectionPoint = false;;
                centerLocationOdom = currentLocation;

                dropOffController.reset();
            } else if (result.goalDriving && timerTimeElapsed >= 5 ) {
                goalLocation = result.centerGoal;
                stateMachineState = STATE_MACHINE_ROTATE;
                timerStartTime = time(0);
            }
            // we are in precision/timed driving
            else {
                goalLocation = currentLocation;
                sendDriveCommand(result.cmdVel,result.angleError);
                stateMachineState = STATE_MACHINE_TRANSFORM;

                break;
            }
        }
        //If angle between current and goal is significant
        //if error in heading is greater than 0.4 radians
        else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance) {
            stateMachineState = STATE_MACHINE_ROTATE;
        }
        //If goal has not yet been reached drive and maintane heading
        else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
            stateMachineState = STATE_MACHINE_SKID_STEER;
        }
        //Otherwise, drop off target and select new random uniform heading
        //If no targets have been detected, assign a new goal
        else if (!targetDetected && timerTimeElapsed > returnToSearchDelay) {
            goalLocation = searchController.search(currentLocation);
        }

        //Purposefully fall through to next case without breaking
    }

    

    // Calculate angle between currentLocation.x/y and goalLocation.x/y
    // Drive forward
    // Stay in this state until angle is at least PI/2
    case STATE_MACHINE_SKID_STEER: {
        stateMachineMsg.data = "SKID_STEER";

        // calculate the distance between current and desired heading in radians
        float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);

        // goal not yet reached drive while maintaining proper heading.
        if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
            // drive and turn simultaniously
            sendDriveCommand(searchVelocity, errorYaw/2);
        }
        // goal is reached but desired heading is still wrong turn only
        else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
             // rotate but dont drive
            sendDriveCommand(0.0, errorYaw);
        }
        else {
            // stop
            sendDriveCommand(0.0, 0.0);
            avoidingObstacle = false;

            // move back to transform step
            stateMachineState = STATE_MACHINE_TRANSFORM;
        }

        break;
    }

    case STATE_MACHINE_PICKUP: {
        stateMachineMsg.data = "PICKUP";

        PickUpResult result;

        // we see a block and have not picked one up yet
        if (targetDetected && !targetCollected) {
            result = pickUpController.pickUpSelectedTarget(blockBlock);
            sendDriveCommand(result.cmdVel,result.angleError);
            std_msgs::Float32 angle;

            if (result.fingerAngle != -1) {
                angle.data = result.fingerAngle;
                fingerAnglePublish.publish(angle);
            }

            if (result.wristAngle != -1) {
                angle.data = result.wristAngle;

                // raise wrist
                wristAnglePublish.publish(angle);
            }

            if (result.giveUp) {
                targetDetected = false;
                stateMachineState = STATE_MACHINE_TRANSFORM;
                sendDriveCommand(0,0);
                pickUpController.reset();
            }

            if (result.pickedUp) {
                pickUpController.reset();

                // assume target has been picked up by gripper
                targetCollected = true;
                result.pickedUp = false;
                stateMachineState = STATE_MACHINE_ROTATE;

                goalLocation.theta = atan2(centerLocationOdom.y - currentLocation.y, centerLocationOdom.x - currentLocation.x);

                // set center as goal position
                goalLocation.x = centerLocationOdom.x;
                goalLocation.y = centerLocationOdom.y;

                // lower wrist to avoid ultrasound sensors
                std_msgs::Float32 angle;
                angle.data = 0.8;
                wristAnglePublish.publish(angle);
                sendDriveCommand(0.0,0);

                return;
            }
        } else {
            stateMachineState = STATE_MACHINE_TRANSFORM;
        }

        break;
    }

    case STATE_MACHINE_DROPOFF: {
        stateMachineMsg.data = "DROPOFF";
        break;
    }

    default: {
        break;
    }

    } /* end of switch() */
}







    // If in manual mode do not try to automatically pick up the target
    if (currentMode == 1 || currentMode == 0) return;

    // if a target is detected and we are looking for center tags
    if (message->detections.size() > 0 && !reachedCollectionPoint) {
        float cameraOffsetCorrection = 0.020; //meters;

        centerSeen = false;
        double count = 0;
        double countRight = 0;
        double countLeft = 0;

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

                centerSeen = true;
                count++;
            }
        }

        if (centerSeen && targetCollected) {
            stateMachineState = STATE_MACHINE_TRANSFORM;
            goalLocation = currentLocation;
        }

        dropOffController.setDataTargets(count,countLeft,countRight);

        // if we see the center and we dont have a target collected
        if (centerSeen && !targetCollected) {

            float centeringTurn = 0.15; //radians
            stateMachineState = STATE_MACHINE_TRANSFORM;

            // this code keeps the robot from driving over
            // the center when searching for blocks
            if (right) {
                // turn away from the center to the left if just driving
                // around/searching.
                goalLocation.theta += centeringTurn;
            } else {
                // turn away from the center to the right if just driving
                // around/searching.
                goalLocation.theta -= centeringTurn;
            }

            // continues an interrupted search
            goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

            targetDetected = false;
            pickUpController.reset();

            return;
        }
    }
    // end found target and looking for center tags

    // found a target april tag and looking for april cubes;
    // with safety timer at greater than 5 seconds.
    PickUpResult result;

    if (message->detections.size() > 0 && !targetCollected && timerTimeElapsed > 5) {
        targetDetected = true;

        // pickup state so target handler can take over driving.
        stateMachineState = STATE_MACHINE_PICKUP;
        result = pickUpController.selectTarget(message);

        std_msgs::Float32 angle;

        if (result.fingerAngle != -1) {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }

        if (result.wristAngle != -1) {
            angle.data = result.wristAngle;
            wristAnglePublish.publish(angle);
        }
    }






    if ((!targetDetected || targetCollected) && (message->data > 0)) {
        // obstacle on right side
        if (message->data == 1) {
            // select new heading 0.2 radians to the left
            goalLocation.theta = currentLocation.theta + 0.6;
        }

        // obstacle in front or on left side
        else if (message->data == 2) {
            // select new heading 0.2 radians to the right
            goalLocation.theta = currentLocation.theta + 0.6;
        }

        // continues an interrupted search
        goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

        // switch to transform state to trigger collision avoidance
        stateMachineState = STATE_MACHINE_ROTATE;

        avoidingObstacle = true;
    }

    // the front ultrasond is blocked very closely. 0.14m currently
    if (message->data == 4) {
        blockBlock = true;
    } else {
        blockBlock = false;
    }







        void mapAverage() {
            // store currentLocation in the averaging array
            mapLocation[mapCount] = currentLocationMap;
            mapCount++;

            if (mapCount >= mapHistorySize) {
                mapCount = 0;
            }

            double x = 0;
            double y = 0;
            double theta = 0;

            // add up all the positions in the array
            for (int i = 0; i < mapHistorySize; i++) {
                x += mapLocation[i].x;
                y += mapLocation[i].y;
                theta += mapLocation[i].theta;
            }

            // find the average
            x = x/mapHistorySize;
            y = y/mapHistorySize;

            // Get theta rotation by converting quaternion orientation to pitch/roll/yaw
            theta = theta/100;
            currentLocationAverage.x = x;
            currentLocationAverage.y = y;
            currentLocationAverage.theta = theta;


            // only run below code if a centerLocation has been set by initilization
            if (init) {
                // map frame
                geometry_msgs::PoseStamped mapPose;

                // setup msg to represent the center location in map frame
                mapPose.header.stamp = ros::Time::now();

                mapPose.header.frame_id = publishedName + "/map";
                mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
                mapPose.pose.position.x = centerLocationMap.x;
                mapPose.pose.position.y = centerLocationMap.y;
                geometry_msgs::PoseStamped odomPose;
                string x = "";

                try { //attempt to get the transform of the center point in map frame to odom frame.
                    tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
                    tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
                }

                catch(tf::TransformException& ex) {
                    ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
                    x = "Exception thrown " + (string)ex.what();
                    std_msgs::String msg;
                    stringstream ss;
                    ss << "Exception in mapAverage() " + (string)ex.what();
                    msg.data = ss.str();
                    infoLogPublisher.publish(msg);
                }

                // Use the position and orientation provided by the ros transform.
                centerLocation.x = odomPose.pose.position.x; //set centerLocation in odom frame
                centerLocation.y = odomPose.pose.position.y;


            }
        }





