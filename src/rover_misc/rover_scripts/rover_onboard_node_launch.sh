#!/bin/bash


#Point to ROS master on the network
export ROS_MASTER_URI=http://beta:11311


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
nohup rosrun rover_onboard_target_detection camera &
nohup rosrun rover_onboard_localization localization &
nohup rosrun rover_onboard_mobility mobility &
nohup rosrun rover_onboard_obstacle_detection obstacle &
nohup rosrun rover_onboard_path_planning path &
nohup rosrun rover_onboard_target_detection target &

microcontrollerDevicePath=$(findDevicePath Arduino)
if [ -z "$microcontrollerDevicePath" ]
then
    echo "Error: Microcontroller device not found"
else
    nohup rosrun rover_onboard_abridge abridge _device:=/dev/$microcontrollerDevicePath &
fi

gpsDevicePath=$(findDevicePath u-blox)
if [ -z "$gpsDevicePath" ]
then
    echo "Error: u-blox GPS device not found"
else
    nohup rosrun ublox_gps ublox_gps /ublox_gps/fix:=/$HOSTNAME/fix _device:=/dev/$gpsDevicePath &
fi


#Wait for user input to terminate processes
while true; do
    echo "Close all driver processes. [q]"
    read choice;

    if [ "$choice" == "q" ];then
	pkill abridge
	pkill camera
	pkill localization
	pkill mobility
	pkill obstacle
	pkill path
	pkill target
	pkill ublox_gps
	exit 1
    fi
done
