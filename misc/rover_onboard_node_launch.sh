#!/bin/bash
echo "running pkill on old rosnodes"
pkill usb_cam_node
pkill behaviours
pkill obstacle
pkill apriltag_detector_node
pkill abridge
pkill ublox_gps
pkill navsat_transform
pkill ekf_localization
pkill diagnostics
pkill static_transform_publisher

source "../devel/setup.bash"
export GAZEBO_MODEL_PATH="../simulation/models"
export GAZEBO_PLUGIN_PATH="../build/gazebo_plugins"

#Point to ROS master on the network
echo "point to ROS master on the network"
if [ -z "$1" ]
then
    echo "Error: ROS_MASTER_URI hostname was not provided"
    exit 1
else
    export ROS_MASTER_URI=http://$1:11311

fi


#Set prefix to fully qualify transforms for each robot
echo "set prefix to fully qualify transforms for each robot: $HOSTNAME"
rosparam set tf_prefix $HOSTNAME


#Function to lookup correct path for a given device
echo "findDevicePath() for usb devices (arduino, camera, etc)"
findDevicePath() {
    for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        if [[ "$syspath" == *"tty"* ]] && [[ "$ID_SERIAL" == *"$1"* ]]
        then
            echo $devname
            break
        fi
    )
    done
}


#Startup ROS packages/processes
echo "rosrun tf static_transform_publisher"
nohup > logs/$HOSTNAME"_transform_log.txt" rosrun tf static_transform_publisher __name:=$HOSTNAME\_BASE2CAM 0.12 -0.03 0.195 -1.57 0 -2.22 /$HOSTNAME/base_link /$HOSTNAME/camera_link 100 &
echo "rosrun video_stream_opencv"

#nohup rosrun video_stream_opencv video_stream __name:=$HOSTNAME\_CAMERA _video_stream_provider:=/dev/video0 /camera:=/$HOSTNAME/camera/image _camera_info_url:=file://${HOME}/SwarmBaseCode-ROS/camera_info head_camera.yaml _width:=320 _height:=240 &
nohup > logs/$HOSTNAME"_USBCAM_log.txt" rosrun usb_cam usb_cam_node __name:=$HOSTNAME\_CAMERA /$HOSTNAME\_CAMERA/image_raw:=/$HOSTNAME/camera/image _camera_info_url:=file://$(realpath ..)/camera_info/head_camera.yaml _image_width:=320 _image_height:=240 &
echo $(realpath ..)/camera_info/head_camera.yaml
# deprecated; we are replacing the usb cam with opencv cam
# mage_raw:=/$HOSTNAME/camera/image _camera_info_url:=file://${HOME}/rover_workspace/camera_info/head_camera.yaml _image_width:=320 _image_height:=240 &

echo "rosrun behaviours"
nohup > logs/$HOSTNAME"_behaviours_log.txt" rosrun behaviours behaviours &
echo "rosrun obstacle_detection"
nohup rosrun obstacle_detection obstacle &
echo "rosrun diagnostics"
nohup > logs/$HOSTNAME"_diagnostics_log.txt" rosrun diagnostics diagnostics &

rosparam set /$HOSTNAME\_TARGET/sensor_frame_id /$HOSTNAME/camera_link
rosparam set /$HOSTNAME\_TARGET/tag_family 36h11
rosparam set /$HOSTNAME\_TARGET/tag_descriptions "[{id: 0, size: 0.038}, {id: 1, size: 0.038}, {id: 2, size: 0.038}, {id: 3, size: 0.038}, {id: 4, size: 0.038}, {id: 5, size: 0.038}, {id: 6, size: 0.038}, {id: 7, size: 0.038}, {id: 8, size: 0.038}, {id: 9, size: 0.038}, {id: 10, size: 0.038}, {id: 11, size: 0.038}, {id: 12, size: 0.038}, {id: 13, size: 0.038}, {id: 14, size: 0.038}, {id: 15, size: 0.038}, {id: 16, size: 0.038}, {id: 17, size: 0.038}, {id: 18, size: 0.038}, {id: 19, size: 0.038}, {id: 20, size: 0.038}, {id: 21, size: 0.038}, {id: 22, size: 0.038}, {id: 23, size: 0.038}, {id: 24, size: 0.038}, {id: 25, size: 0.038}, {id: 26, size: 0.038}, {id: 27, size: 0.038}, {id: 28, size: 0.038}, {id: 29, size: 0.038}, {id: 30, size: 0.038}, {id: 31, size: 0.038}, {id: 32, size: 0.038}, {id: 33, size: 0.038}, {id: 34, size: 0.038}, {id: 35, size: 0.038}, {id: 36, size: 0.038}, {id: 37, size: 0.038}, {id: 38, size: 0.038}, {id: 39, size: 0.038}, {id: 40, size: 0.038}, {id: 41, size: 0.038}, {id: 42, size: 0.038}, {id: 43, size: 0.038}, {id: 44, size: 0.038}, {id: 45, size: 0.038}, {id: 46, size: 0.038}, {id: 47, size: 0.038}, {id: 48, size: 0.038}, {id: 49, size: 0.038}, {id: 50, size: 0.038}, {id: 51, size: 0.038}, {id: 52, size: 0.038}, {id: 53, size: 0.038}, {id: 54, size: 0.038}, {id: 55, size: 0.038}, {id: 56, size: 0.038}, {id: 57, size: 0.038}, {id: 58, size: 0.038}, {id: 59, size: 0.038}, {id: 60, size: 0.038}, {id: 61, size: 0.038}, {id: 62, size: 0.038}, {id: 63, size: 0.038}, {id: 64, size: 0.038}, {id: 65, size: 0.038}, {id: 66, size: 0.038}, {id: 67, size: 0.038}, {id: 68, size: 0.038}, {id: 69, size: 0.038}, {id: 70, size: 0.038}, {id: 71, size: 0.038}, {id: 72, size: 0.038}, {id: 73, size: 0.038}, {id: 74, size: 0.038}, {id: 75, size: 0.038}, {id: 76, size: 0.038}, {id: 77, size: 0.038}, {id: 78, size: 0.038}, {id: 79, size: 0.038}, {id: 80, size: 0.038}, {id: 81, size: 0.038}, {id: 82, size: 0.038}, {id: 83, size: 0.038}, {id: 84, size: 0.038}, {id: 85, size: 0.038}, {id: 86, size: 0.038}, {id: 87, size: 0.038}, {id: 88, size: 0.038}, {id: 89, size: 0.038}, {id: 90, size: 0.038}, {id: 91, size: 0.038}, {id: 92, size: 0.038}, {id: 93, size: 0.038}, {id: 94, size: 0.038}, {id: 95, size: 0.038}, {id: 96, size: 0.038}, {id: 97, size: 0.038}, {id: 98, size: 0.038}, {id: 99, size: 0.038}, {id: 100, size: 0.038}, {id: 101, size: 0.038}, {id: 102, size: 0.038}, {id: 103, size: 0.038}, {id: 104, size: 0.038}, {id: 105, size: 0.038}, {id: 106, size: 0.038}, {id: 107, size: 0.038}, {id: 108, size: 0.038}, {id: 109, size: 0.038}, {id: 110, size: 0.038}, {id: 111, size: 0.038}, {id: 112, size: 0.038}, {id: 113, size: 0.038}, {id: 114, size: 0.038}, {id: 115, size: 0.038}, {id: 116, size: 0.038}, {id: 117, size: 0.038}, {id: 118, size: 0.038}, {id: 119, size: 0.038}, {id: 120, size: 0.038}, {id: 121, size: 0.038}, {id: 122, size: 0.038}, {id: 123, size: 0.038}, {id: 124, size: 0.038}, {id: 125, size: 0.038}, {id: 126, size: 0.038}, {id: 127, size: 0.038}, {id: 128, size: 0.038}, {id: 129, size: 0.038}, {id: 130, size: 0.038}, {id: 131, size: 0.038}, {id: 132, size: 0.038}, {id: 133, size: 0.038}, {id: 134, size: 0.038}, {id: 135, size: 0.038}, {id: 136, size: 0.038}, {id: 137, size: 0.038}, {id: 138, size: 0.038}, {id: 139, size: 0.038}, {id: 140, size: 0.038}, {id: 141, size: 0.038}, {id: 142, size: 0.038}, {id: 143, size: 0.038}, {id: 144, size: 0.038}, {id: 145, size: 0.038}, {id: 146, size: 0.038}, {id: 147, size: 0.038}, {id: 148, size: 0.038}, {id: 149, size: 0.038}, {id: 150, size: 0.038}, {id: 151, size: 0.038}, {id: 152, size: 0.038}, {id: 153, size: 0.038}, {id: 154, size: 0.038}, {id: 155, size: 0.038}, {id: 156, size: 0.038}, {id: 157, size: 0.038}, {id: 158, size: 0.038}, {id: 159, size: 0.038}, {id: 160, size: 0.038}, {id: 161, size: 0.038}, {id: 162, size: 0.038}, {id: 163, size: 0.038}, {id: 164, size: 0.038}, {id: 165, size: 0.038}, {id: 166, size: 0.038}, {id: 167, size: 0.038}, {id: 168, size: 0.038}, {id: 169, size: 0.038}, {id: 170, size: 0.038}, {id: 171, size: 0.038}, {id: 172, size: 0.038}, {id: 173, size: 0.038}, {id: 174, size: 0.038}, {id: 175, size: 0.038}, {id: 176, size: 0.038}, {id: 177, size: 0.038}, {id: 178, size: 0.038}, {id: 179, size: 0.038}, {id: 180, size: 0.038}, {id: 181, size: 0.038}, {id: 182, size: 0.038}, {id: 183, size: 0.038}, {id: 184, size: 0.038}, {id: 185, size: 0.038}, {id: 186, size: 0.038}, {id: 187, size: 0.038}, {id: 188, size: 0.038}, {id: 189, size: 0.038}, {id: 190, size: 0.038}, {id: 191, size: 0.038}, {id: 192, size: 0.038}, {id: 193, size: 0.038}, {id: 194, size: 0.038}, {id: 195, size: 0.038}, {id: 196, size: 0.038}, {id: 197, size: 0.038}, {id: 198, size: 0.038}, {id: 199, size: 0.038}, {id: 200, size: 0.038}, {id: 201, size: 0.038}, {id: 202, size: 0.038}, {id: 203, size: 0.038}, {id: 204, size: 0.038}, {id: 205, size: 0.038}, {id: 206, size: 0.038}, {id: 207, size: 0.038}, {id: 208, size: 0.038}, {id: 209, size: 0.038}, {id: 210, size: 0.038}, {id: 211, size: 0.038}, {id: 212, size: 0.038}, {id: 213, size: 0.038}, {id: 214, size: 0.038}, {id: 215, size: 0.038}, {id: 216, size: 0.038}, {id: 217, size: 0.038}, {id: 218, size: 0.038}, {id: 219, size: 0.038}, {id: 220, size: 0.038}, {id: 221, size: 0.038}, {id: 222, size: 0.038}, {id: 223, size: 0.038}, {id: 224, size: 0.038}, {id: 225, size: 0.038}, {id: 226, size: 0.038}, {id: 227, size: 0.038}, {id: 228, size: 0.038}, {id: 229, size: 0.038}, {id: 230, size: 0.038}, {id: 231, size: 0.038}, {id: 232, size: 0.038}, {id: 233, size: 0.038}, {id: 234, size: 0.038}, {id: 235, size: 0.038}, {id: 236, size: 0.038}, {id: 237, size: 0.038}, {id: 238, size: 0.038}, {id: 239, size: 0.038}, {id: 240, size: 0.038}, {id: 241, size: 0.038}, {id: 242, size: 0.038}, {id: 243, size: 0.038}, {id: 244, size: 0.038}, {id: 245, size: 0.038}, {id: 246, size: 0.038}, {id: 247, size: 0.038}, {id: 248, size: 0.038}, {id: 249, size: 0.038}, {id: 250, size: 0.038}, {id: 251, size: 0.038}, {id: 252, size: 0.038}, {id: 253, size: 0.038}, {id: 254, size: 0.038}, {id: 255, size: 0.038}, {id: 256, size: 0.038}]"
nohup > logs/$HOSTNAME"_aprilTag_log.txt" rosrun apriltags_ros apriltag_detector_node __name:=$HOSTNAME\_TARGET /image_rect:=/$HOSTNAME/camera/image /camera_info:=/$HOSTNAME/camera/camera_info /tag_detections:=/$HOSTNAME/targets /tag_detections_image:=/$HOSTNAME/targets/image &

microcontrollerDevicePath=$(findDevicePath Arduino)
if [ -z "$microcontrollerDevicePath" ]
then
    echo "Error: Microcontroller device not found"
else
    nohup > logs/$HOSTNAME"_abridge_log.txt" rosrun abridge abridge _device:=/dev/$microcontrollerDevicePath &
fi

gpsDevicePath=$(findDevicePath u-blox)
if [ -z "$gpsDevicePath" ]
then
    echo "Error: u-blox GPS device not found"
else
    nohup > logs/$HOSTNAME"_ublox_log.txt" rosrun ublox_gps ublox_gps __name:=$HOSTNAME\_UBLOX /$HOSTNAME\_UBLOX/fix:=/$HOSTNAME/fix /$HOSTNAME\_UBLOX/fix_velocity:=/$HOSTNAME/fix_velocity /$HOSTNAME\_UBLOX/navposllh:=/$HOSTNAME/navposllh /$HOSTNAME\_UBLOX/navsol:=/$HOSTNAME/navsol /$HOSTNAME\_UBLOX/navstatus:=/$HOSTNAME/navstatus /$HOSTNAME\_UBLOX/navvelned:=/$HOSTNAME/navvelned _device:=/dev/$gpsDevicePath _frame_id:=$HOSTNAME/base_link &
fi

nohup > logs/$HOSTNAME"_localization_navsat_log.txt" rosrun robot_localization navsat_transform_node __name:=$HOSTNAME\_NAVSAT _world_frame:=map _frequency:=10 _magnetic_declination_radians:=0.1530654 _yaw_offset:=0 /imu/data:=/$HOSTNAME/imu /gps/fix:=/$HOSTNAME/fix /odometry/filtered:=/$HOSTNAME/odom/ekf /odometry/gps:=/$HOSTNAME/odom/navsat &

rosparam set /$HOSTNAME\_ODOM/odom0 /$HOSTNAME/odom
rosparam set /$HOSTNAME\_ODOM/imu0 /$HOSTNAME/imu
rosparam set /$HOSTNAME\_ODOM/odom0_config [false,false,false,false,false,false,true,false,false,false,false,true,false,false,false]
rosparam set /$HOSTNAME\_ODOM/imu0_config [false,false,false,false,false,true,false,false,false,false,false,true,true,false,false]
nohup > logs/$HOSTNAME"_odom_EKF_log.txt" rosrun robot_localization ekf_localization_node _two_d_mode:=true _world_frame:=odom _frequency:=10 __name:=$HOSTNAME\_ODOM /odometry/filtered:=/$HOSTNAME/odom/filtered &

rosparam set /$HOSTNAME\_MAP/odom0 /$HOSTNAME/odom/navsat
rosparam set /$HOSTNAME\_MAP/odom1 /$HOSTNAME/odom/filtered
rosparam set /$HOSTNAME\_MAP/imu0 /$HOSTNAME/imu
rosparam set /$HOSTNAME\_MAP/odom0_config [true,true,false,false,false,false,false,false,false,false,false,false,false,false,false]
rosparam set /$HOSTNAME\_MAP/odom1_config [false,false,false,false,false,false,true,false,false,false,false,false,false,false,false]
rosparam set /$HOSTNAME\_MAP/imu0_config [false,false,false,false,false,true,false,false,false,false,false,true,false,false,false]

rosparam set /$HOSTNAME\_MAP/initial_estimate_covariance "[1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                               0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                               0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                               0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                               0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                               0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                               0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                               0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0,  /
                                               0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0,  /
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0,  /
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0,  /
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0,  /
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0,  /
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0,  /
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9]"


rosparam set $HOSTNAME\_MAP/process_noise_covariance "[0.005, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                                    0, 0.8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                                    0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                                    0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                                    0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                                    0, 0, 0, 0, 0, 0.005, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                                    0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0,  /
                                                    0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0,  /
                                                    0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0,  /
                                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,  /
                                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0,  /
                                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0,  /
                                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0,  /
                                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0,  /
                                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1]"

nohup > logs/$HOSTNAME"_map_EKF_log.txt" rosrun robot_localization ekf_localization_node _two_d_mode:=true _world_frame:=map _frequency:=10 __name:=$HOSTNAME\_MAP /odometry/filtered:=/$HOSTNAME/odom/ekf &


#Wait for user input to terminate processes
while true; do
    echo "Close all driver processes. [q]"
    read choice;

    if [ "$choice" == "q" ];then
	rosnode kill $HOSTNAME\_BEHAVIOUR
	rostopic pub -1 /$HOSTNAME\/velocity geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
	rosnode kill $HOSTNAME\_ABRIDGE
	rosnode kill $HOSTNAME\_NAVSAT
	rosnode kill $HOSTNAME\_ODOM
	rosnode kill $HOSTNAME\_MAP
	rosnode kill $HOSTNAME\_CAMERA
	rosnode kill $HOSTNAME\_OBSTACLE
	rosnode kill $HOSTNAME\_TARGET
	rosnode kill $HOSTNAME\_DIAGNOSTICS
	rosnode kill $HOSTNAME\_BASE2CAM
	rosnode kill $HOSTNAME\_UBLOX

	exit 1
    fi
done
