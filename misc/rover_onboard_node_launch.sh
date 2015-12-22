#!/bin/bash
pkill camera
pkill mobility
pkill obstacle
pkill target
pkill abridge
pkill ublox_gps
pkill navsat_transfor
pkill ekf_localizatio


#Point to ROS master on the network
if [ -z "$1" ]
then
    echo "Error: ROS_MASTER_URI hostname was not provided"
    exit 1
else
    export ROS_MASTER_URI=http://$1:11311

fi

#Function to lookup correct path for a given device
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
nohup rosrun target_detection camera &
nohup rosrun mobility mobility &
nohup rosrun obstacle_detection obstacle &
nohup rosrun target_detection target &

microcontrollerDevicePath=$(findDevicePath Arduino)
if [ -z "$microcontrollerDevicePath" ]
then
    echo "Error: Microcontroller device not found"
else
    nohup rosrun abridge abridge _device:=/dev/$microcontrollerDevicePath &
fi

gpsDevicePath=$(findDevicePath u-blox)
if [ -z "$gpsDevicePath" ]
then
    echo "Error: u-blox GPS device not found"
else
    nohup rosrun ublox_gps ublox_gps /ublox_gps/fix:=/$HOSTNAME/fix _device:=/dev/$gpsDevicePath &
fi

nohup rosrun robot_localization navsat_transform_node __name:=$HOSTNAME\_NAVSAT _world_frame:=map _magnetic_declination_radians:=0.1530654 _yaw_offset:=1.57079632679 /imu/data:=/$HOSTNAME/imu /gps/fix:=/$HOSTNAME/fix /odometry/filtered:=/$HOSTNAME/odom/ekf /odometry/gps:=/$HOSTNAME/odom/navsat &

rosparam set /$HOSTNAME\_EKF/odom0 /$HOSTNAME/odom/navsat
rosparam set /$HOSTNAME\_EKF/odom1 /$HOSTNAME/odom
rosparam set /$HOSTNAME\_EKF/imu0 /$HOSTNAME/imu
rosparam set /$HOSTNAME\_EKF/odom0_config [true,true,false,false,false,false,false,false,false,false,false,false,false,false,false]
rosparam set /$HOSTNAME\_EKF/odom1_config [true,true,false,false,false,false,false,false,false,false,false,false,false,false,false]
rosparam set /$HOSTNAME\_EKF/imu0_config [false,false,false,false,false,true,false,false,false,false,false,true,true,false,false]
nohup rosrun robot_localization ekf_localization_node _two_d_mode:=true __name:=$HOSTNAME\_EKF /odometry/filtered:=/$HOSTNAME/odom/ekf &


#Wait for user input to terminate processes
while true; do
    echo "Close all driver processes. [q]"
    read choice;

    if [ "$choice" == "q" ];then
	rosnode kill $HOSTNAME\_MOBILITY
	rostopic pub -1 /$HOSTNAME\/mobility geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
	rosnode kill $HOSTNAME\_ABRIDGE
	rosnode kill $HOSTNAME\_NAVSAT
	rosnode kill $HOSTNAME\_EKF
	rosnode kill $HOSTNAME\_CAMERA
	rosnode kill $HOSTNAME\_OBSTACLE
	rosnode kill $HOSTNAME\_TARGET
	rosnode kill ublox_gps

	pkill camera
	pkill mobility
	pkill obstacle
	pkill target
	pkill abridge
	pkill ublox_gps
	pkill navsat_transfor
	pkill ekf_localizatio

	exit 1
    fi
done
