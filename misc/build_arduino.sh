#! /bin/bash

arduino=/opt/arduino/arduino
repo=~/Swarmathon-Arduino
build=$repo/build
sketch=$repo/Swarmathon_Arduino/Swarmathon_Arduino.ino

if udevadm info /dev/ttyACM0 | grep -q Leonardo; then
  dev=/dev/ttyACM0
else
  dev=/dev/ttyACM1
fi

echo "Building for Leonardo on $dev"

$arduino --upload --preserve-temp-files --pref serial.port=$dev --pref build.verbose=true --pref upload.verbose=true --pref build.path=$build --pref sketchbook.path=$repo --pref board=leonardo $sketch

 
