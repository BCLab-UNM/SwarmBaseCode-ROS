#!/bin/bash
if [ $# -ne  1 ]; then
	echo "USAGE: ./load_arduino_code.sh [calibration-filepath]"
        exit 1
fi

echo "Locate calibration file and extract values..."
# e.g. min: { -9999, -9999, -9999 } max: { +9999, +9999, +9999 }"
calibration_vars=$(awk '/\+/ || /\-/ {print}' $1)
if [ $? -ne 0 ]; then
	echo "********************************"
        echo "FAILED TO OPEN CALIBRATION FILE!"
        exit 1
fi
calibration_vars=$(echo $calibration_vars | sed 's/}/ /g; s/{/ /g; s/,/ /g; s/min:/ /; s/max:/ /;')
echo $calibration_vars

echo "Copy calibration values to arduino sketch..."
min_cal=$(echo $calibration_vars | awk '{printf "{%s, %s, %s};", $1, $2, $3}')
max_cal=$(echo $calibration_vars | awk '{printf "{%s, %s, %s};", $4, $5, $6}') 
arduino_sketch=../arduino/swarmie_control/swarmie_control.ino
sed -i "s/magnetometer_accelerometer.m_min\s*=.*;/magnetometer_accelerometer.m_min = (LSM303::vector<int16_t>)${min_cal}/" $arduino_sketch
if [ $? -ne 0 ]; then
	echo "*********************************************"
	echo "FAILED TO WRITE CALIBRATION VALUES TO SKETCH!"
	exit 1
fi
sed -i "s/magnetometer_accelerometer.m_max\s*=.*;/magnetometer_accelerometer.m_max = (LSM303::vector<int16_t>)${max_cal}/" $arduino_sketch
if [ $? -ne 0 ]; then
	echo "*********************************************"
	echo "FAILED TO WRITE CALIBRATION VALUES TO SKETCH!"
	exit 1
fi

echo "Build sketch and upload to arduino leonardo..."
arduino_port=/dev/ttyACM1
leo_cmd="--upload $arduino_sketch"
leo_pref="--preserve-temp-files"
leo_pref="$leo_pref --pref serial.port=$arduino_port"
leo_pref="$leo_pref --pref build.verbose=true"
leo_pref="$leo_pref --pref upload.verbose=true"
leo_pref="$leo_pref --pref build.path=/tmp"
leo_pref="$leo_pref --pref sketchbook.path=../arduino/swarmie_control/"
leo_pref="$leo_pref --pref board=leonardo"
echo "~/arduino-1.8.5/arduino $leo_cmd $leo_pref" 
if [ $? -ne 0 ]; then
	echo "*********************************************"
	echo "FAILED TO UPLOAD CALIBRATION FILE TO ARDUINO!"
	exit 1
fi
echo "***************************"
echo "CALIBRATION UPLOAD SUCCESS!"

