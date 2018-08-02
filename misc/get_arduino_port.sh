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
get_arduino_port
# END OF GET ARDUINO PORT FUNTION 
