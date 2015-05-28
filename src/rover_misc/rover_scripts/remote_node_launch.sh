#!/usr/bin/expect
set timeout 20

set host [lindex $argv 0]
set user [lindex $argv 1]

spawn ssh "$host@$user"

expect "*assword:"
send "saturn\r";

expect "$host@$user:"
send "./rover_onboard_node_launch.sh\r";

interact

