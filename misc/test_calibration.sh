#!/bin/bash
# START OF GET ARDUINO PORT FUNCTION
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
# END OF GET ARDUINO PORT FUNTION 

# START OF LOAD CALIBRATION
load_calibration_data () {
echo "Locate calibration file and extract values..."
calibrationFile="../../KSC.cal"
# e.g. min: { -9999, -9999, -9999 } max: { +9999, +9999, +9999 }"
calibration_vars=$(awk '/\+/ || /\-/ {print}' $calibrationFile)
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
arduino_port=$(get_arduino_port)
leo_cmd="--upload $arduino_sketch"
leo_pref="--preserve-temp-files"
leo_pref="$leo_pref --pref serial.port=$arduino_port"
leo_pref="$leo_pref --pref build.verbose=true"
leo_pref="$leo_pref --pref upload.verbose=true"
leo_pref="$leo_pref --pref build.path=/tmp"
leo_pref="$leo_pref --pref sketchbook.path=../arduino/"
leo_pref="$leo_pref --pref board=leonardo"
~/arduino-1.8.5/arduino $leo_cmd $leo_pref
if [ $? -ne 0 ]; then
	echo "*********************************************"
	echo "FAILED TO UPLOAD CALIBRATION FILE TO ARDUINO!"
	exit 1
fi
echo "***************************"
echo "CALIBRATION UPLOAD SUCCESS!"
}
# END OF LOAD CALIBRATION FUNCTION

arduino_port=$(get_arduino_port)
if [ -z $arduino_port ]; then
	echo "No Ardunios found"
	exit 1
else
	echo "Found Arduino on port $arduino_port"
fi

# Load the calibration Arduino sketch
~/arduino-1.8.5/arduino --upload --preserve-temp-files --pref serial.port=$arduino_port --pref build.verbose=true --pref upload.verbose=true --pref build.path=/tmp --pref sketchbook.path=/home/swarmie/SwarmBaseCode-ROS/arduino --pref board=leonardo calibrator/calibrator.ino

exit 1

#Start ROS core for ros serial
roscore &
sleep 1
rosrun rosserial_python serial_node.py _baud:=9600 $arduino_port &
#sleep 1

#echo "Starting Calibration"
#python calibrate.py
#pkill roscore
#load_calibration_data
exit 0
