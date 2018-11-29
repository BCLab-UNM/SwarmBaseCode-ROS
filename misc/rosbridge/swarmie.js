'use strict';

//exports.make_swarmie = (robot) => new Swarmie(robot);

class Swarmie {
    constructor(robotName) {
        this.robotName = robotName;
        this.robotRos = new ROSLIB.Ros({
            url : 'ws://' + robotName + ':9090'
        });
        this.laptopRos = new ROSLIB.Ros({
            url : 'ws://localhost:9090'
        });
               
        this.robotRos.on('connection', function() {
            console.log('Connected to robot websocket server.');
        });
        this.laptopRos.on('connection', function() {
            console.log('Connected to laptop websocket server');
        });
        
        this.robotRos.on('error', function(error) {
            console.log('Error connecting to websocket server: ', error);
        });

        this.robotRos.on('close', function() {
            console.log('Connection to websocket server closed.');
        });

        this.abridgeHeartbeat = this.createPassthrough(this.robotRos, this.laptopRos, '/abridge/heartbeat', 'std_msgs/String', false, false);
        this.behaviourHeartbeat = this.createPassthrough(this.robotRos, this.laptopRos, '/behaviour/heartbeat', 'std_msgs/String', false, false);
        this.cameraInfo = this.createPassthrough(this.robotRos, this.laptopRos, '/camera/camera_info', 'sensor_msgs/CameraInfo', true, false);
        this.diagnostics = this.createPassthrough(this.robotRos, this.laptopRos, '/diagnostics', 'std_msgs/Float32MultiArray', false, false);
        this.driveControl = this.createPassthrough(this.laptopRos, this.robotRos, '/driveControl', 'geometry_msgs/Twist', false, false);
        this.fingerAngle = this.createPassthrough(this.laptopRos, this.robotRos, '/fingerAngle/cmd', 'std_msgs/Float32', false, false);
        this.fixVelocity = this.createPassthrough(this.robotRos, this.laptopRos, '/fix_velocity', 'geometry_msgs/TwistWithCovarianceStamped', false, false);
        this.imu = this.createPassthrough(this.robotRos, this.laptopRos, '/imu_throttle', 'sensor_msgs/Imu', false, false);
        this.joystick = this.createPassthrough(this.laptopRos, this.robotRos, '/joystick', 'sensor_msgs/Joy', false, false);
        this.mode = this.createPassthrough(this.laptopRos, this.robotRos, '/mode', 'std_msgs/UInt8', false, false);
        this.navsol = this.createPassthrough(this.robotRos, this.laptopRos, '/navsol_throttle', 'ublox_msgs/NavSOL', false, false);
      //  this.odom = this.createPassthrough(this.robotRos, this.laptopRos, '/odom_throttle', 'nav_msgs/Odometry', false, false);
        this.odomEkf = this.createPassthrough(this.robotRos, this.laptopRos, '/odom/ekf_throttle', 'nav_msgs/Odometry', false, false);
        this.odomFiltered = this.createPassthrough(this.robotRos, this.laptopRos, '/odom/filtered_throttle', 'nav_msgs/Odometry', false, false);
        this.odomNavsat = this.createPassthrough(this.robotRos, this.laptopRos, '/odom/navsat_throttle', 'nav_msgs/Odometry', false, false);
        this.sbridgeHeartbeat = this.createPassthrough(this.robotRos, this.laptopRos, '/sbridge/heartbeat', 'std_msgs/String', false, false);
        this.sonarCenter = this.createPassthrough(this.robotRos, this.laptopRos, '/sonarCenter_throttle', 'sensor_msgs/Range', false, false);
        this.sonarLeft = this.createPassthrough(this.robotRos, this.laptopRos, '/sonarLeft_throttle', 'sensor_msgs/Range', false, false);
        this.sonarRight = this.createPassthrough(this.robotRos, this.laptopRos, '/sonarRight_throttle', 'sensor_msgs/Range', false, false);
        this.stateMachine = this.createPassthrough(this.robotRos, this.laptopRos, '/state_machine', 'std_msgs/String', false, false);
        this.status = this.createPassthrough(this.robotRos, this.laptopRos, '/status', 'std_msgs/String', false, false);
        this.targetsImageCompressed = this.createPassthrough(this.robotRos, this.laptopRos, '/targets/image/compressed', 'sensor_msgs/CompressedImage', false, false);
        this.virtualFence = this.createPassthrough(this.laptopRos, this.robotRos, '/virtualFence', 'std_msgs/Float32MultiArray', false, true);
        this.wristAngle = this.createPassthrough(this.laptopRos, this.robotRos, '/wristAngle/cmd', 'std_msgs/Float32', false, false);
    }

    createPassthrough(sourceRosHandle, destinationRosHandle, robotSubscription, messageReceived, oneShot, global) {

        var sourceTopicName, destTopicName;

        if (global === true) {
            sourceTopicName = robotSubscription;
        } else {
            sourceTopicName = '/' + this.robotName + robotSubscription;
        }


	    destTopicName = sourceTopicName;
            
        var passthroughHandle = {
            sourceTopic : new ROSLIB.Topic({
                ros : sourceRosHandle,
                name : sourceTopicName,
                messageType : messageReceived
            }),
            destinationTopic : new ROSLIB.Topic({
                ros : destinationRosHandle,
                name : destTopicName,
                messageType : messageReceived
            })
        };
    
        passthroughHandle.sourceTopic.subscribe(function(message) {
            passthroughHandle.destinationTopic.publish(message);
            if (oneShot) {
                passthroughHandle.sourceTopic.unsubscribe();
            };
        });
        return passthroughHandle;
    }
}
