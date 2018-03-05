#!/bin/bash
if [ -z $1 ]; then
	echo "Usage: ./run_calibration.sh location"
	exit 1
fi
pkill roscore
arduino_port=$(./get_arduino_port.sh)
if [ -z $arduino_port ]; then
	echo "No Ardunios found"
	exit 1
else
	echo "Found Arduino on port $arduino_port"
fi

# Load the calibration Arduino sketch
repo_path=`eval "cd $dir;pwd;cd - > /dev/null"`
full_path=repo_path
~/arduino-1.8.5/arduino --upload --preserve-temp-files --pref serial.port=$arduino_port --pref build.verbose=true --pref upload.verbose=true --pref build.path=/tmp --pref sketchbook.path=$(realpath $PWD/../arduino)  --pref board=leonardo $(realpath $PWD/../arduino/calibrator/calibrator.ino)

#Start ROS core for ros serial
roscore &
sleep 1
rosrun rosserial_python serial_node.py _baud:=9600 $arduino_port &
sleep 1

echo "Starting Calibration"
echo "Press Ctrl+C (^C) when finished..."

python calibrate.py $1
pkill roscore
exit 0
