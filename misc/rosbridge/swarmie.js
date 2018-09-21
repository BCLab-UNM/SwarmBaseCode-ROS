
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
            console.log('Connected to websocket server.');
        });
        this.laptopRos.on('connection', function() {
            console.log('We are on');
        });
        
        this.robotRos.on('error', function(error) {
            console.log('Error connecting to websocket server: ', error);
        });

        this.robotRos.on('close', function() {
            console.log('Connection to websocket server closed.');
        });

        this.abridgeHeartbeat = this.createPassthrough(this.robotRos, this.laptopRos, '/abridge/heartbeat', 'std_msgs/String');
        this.behaviourHeartbeat = this.createPassthrough(this.robotRos, this.laptopRos, '/behaviour/heartbeat', 'std_msgs/String');
        this.cameraInfo = this.createPassthrough(this.robotRos, this.laptopRos, '/camera/camera_info', 'sensor_msgs/CameraInfo', true);
        this.diagnostics = this.createPassthrough(this.robotRos, this.laptopRos, '/diagnostics', 'std_msgs/Float32MultiArray');
        this.driveControl = this.createPassthrough(this.laptopRos, this.robotRos, '/driveControl', 'geometry_msgs/Twist');
        this.fingerAngle = this.createPassthrough(this.laptopRos, this.robotRos, '/fingerAngle/cmd', 'std_msgs/Float32');
        this.fixVelocity = this.createPassthrough(this.robotRos, this.laptopRos, '/fix_velocity', 'geometry_msgs/TwistWithCovarianceStamped');
        this.imu = this.createPassthrough(this.robotRos, this.laptopRos, '/imu', 'sensor_msgs/Imu');
        this.joystick = this.createPassthrough(this.laptopRos, this.robotRos, '/joystick', 'sensor_msgs/Joy');
        this.mode = this.createPassthrough(this.laptopRos, this.robotRos, '/mode', 'std_msgs/UInt8');
        this.navsol = this.createPassthrough(this.robotRos, this.laptopRos, '/navsol', 'ublox_msgs/NavSOL');
        this.odom = this.createPassthrough(this.robotRos, this.laptopRos, '/odom', 'nav_msgs/Odometry');
        this.odomEkf = this.createPassthrough(this.robotRos, this.laptopRos, '/odom/ekf', 'nav_msgs/Odometry'); 
        this.odomFiltered = this.createPassthrough(this.robotRos, this.laptopRos, '/odom/filtered', 'nav_msgs/Odometry');
        this.odomNavsat = this.createPassthrough(this.robotRos, this.laptopRos, '/odom/navsat', 'nav_msgs/Odometry');
        this.sbridgeHeartbeat = this.createPassthrough(this.robotRos, this.laptopRos, '/sbridge/heartbeat', 'std_msgs/String');
        this.sonarCenter = this.createPassthrough(this.robotRos, this.laptopRos, '/sonarCenter', 'sensor_msgs/Range');   
        this.sonarLeft = this.createPassthrough(this.robotRos, this.laptopRos, '/sonarLeft', 'sensor_msgs/Range');
        this.sonarRight = this.createPassthrough(this.robotRos, this.laptopRos, '/sonarRight', 'sensor_msgs/Range');
        this.stateMachine = this.createPassthrough(this.robotRos, this.laptopRos, '/state_machine', 'std_msgs/String');
        this.status = this.createPassthrough(this.robotRos, this.laptopRos, '/status', 'std_msgs/String');
        this.targets = this.createPassthrough(this.robotRos, this.laptopRos, '/targets', 'apriltags_ros/AprilTagDetectionArray');
        this.targetsImage = this.createPassthrough(this.robotRos, this.laptopRos, '/targets/image', 'sensor_msgs/Image');
        this.targetsImageCompressed = this.createPassthrough(this.robotRos, this.laptopRos, '/targets/image/compressed', 'sensor_msgs/CompressedImage');
        this.virtualFence = this.createPassthrough(this.laptopRos, this.robotRos, '/virtualFence', 'std_msgs/Float32MultiArray', false, true);
        this.wristAngle = this.createPassthrough(this.laptopRos, this.robotRos, '/wristAngle/cmd', 'std_msgs/Float32');
    };
    createPassthrough(sourceRosHandle, destinationRosHandle, robotSubscription, messageReceived, oneShot = false, global = false) {

        var sourceTopicName;

        if (global === true) {
            sourceTopicName = robotSubscription;
        } else {
            sourceTopicName = '/' + this.robotName + robotSubscription;
        };
            
        var passthroughHandle = {
            sourceTopic : new ROSLIB.Topic({
                ros : sourceRosHandle,
                name : sourceTopicName,
                messageType : messageReceived
            }),
            destinationTopic : new ROSLIB.Topic({
                ros : destinationRosHandle,
                name : sourceTopicName,
                messageType : messageReceived
            })
        }
    
        passthroughHandle.sourceTopic.subscribe(function(message) {
            passthroughHandle.destinationTopic.publish(message);
            if (oneShot) {
                passthroughHandle.sourceTopic.unsubscribe();
            };
        });
        return passthroughHandle;
    }

}
