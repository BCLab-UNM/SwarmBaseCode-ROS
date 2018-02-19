#!/bin/bash
echo "locate calibration file"
if [ -z  $1 ]; then
	echo "oh shit"
        exit 1
fi

echo "extract calibration values"
# e.g. min: { +9999, -9999, -9999 } max: { -9999, -9999, +9999 }"
calibration_vars=$(awk '/\+/ || /\-/ {print}' $1 | sed 's/}/ /g; s/{/ /g; s/,/ /g; s/min:/ /; s/max:/ /;')
echo $calibration_vars

echo "copy calibration values to arduino sketch"
min_cal=$(echo $calibration_vars | awk '{printf "{%s, %s, %s};", $1, $2, $3}')
max_cal=$(echo $calibration_vars | awk '{printf "{%s, %s, %s};", $4, $5, $6}') 
echo $min_cal $max_cal
sed -i "s/magnetometer_accelerometer.m_min\s*=.*;/magnetometer_accelerometer.m_min = (LSM303::vector<int16_t>)${min_cal}/" ~/Swarmathon-Arduino/Swarmathon_Arduino/Swarmathon_Arduino.ino
sed -i "s/magnetometer_accelerometer.m_max\s*=.*;/magnetometer_accelerometer.m_max = (LSM303::vector<int16_t>)${max_cal}/" ~/Swarmathon-Arduino/Swarmathon_Arduino/Swarmathon_Arduino.ino

#echo "build sketch upload to microcontroller"
~/arduino-1.6.8/arduino --upload ~/Swarmathon-Arduino/Swarmathon_Arduino/Swarmathon_Arduino.ino --port /dev/ttyACM1

