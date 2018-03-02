#!/bin/bash
get_arduino_port () {
for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue

        if [[ "$ID_SERIAL" = *"Arduino"* ]]; then
        echo "/dev/$devname"
        fi
    )
done
}

arduino_port=$(get_arduino_port)
if [ -z $arduino_port ]; then
	echo "No Ardunios found"
	exit 1
else
	echo "Found Arduino on port $arduino_port"
fi

# Load the calibration Arduino sketch
~/arduino-1.8.5/arduino --upload --preserve-temp-files --pref serial.port=$arduino_port --pref build.verbose=true --pref upload.verbose=true --pref build.path=/tmp --pref sketchbook.path=../arduino --pref board=leonardo calibrator/calibrator.ino

#Start ROS core for ros serial
roscore &
sleep 1
rosrun rosserial_python serial_node.py _baud:=9600 $arduino_port &
sleep 1

echo "Starting Calibration"
python calibrate.py
pkill roscore
exit 0
