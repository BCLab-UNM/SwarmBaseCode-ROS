#!/usr/bin/expect
set timeout 20

set homeUserName [lindex $argv 0]
set homeHostname [lindex $argv 1]
set swarmieUserName [lindex $argv 2]
set swarmieHostname [lindex $argv 3]


expect ""
spawn ssh "$swarmieUserName@$swarmieHostname"
expect "*assword:"
send "saturn\r";

expect "$swarmieUserName@$swarmieHostname:"
send "rm softwareLoadOnboard.sh\r";

expect "$swarmieUserName@$swarmieHostname:"
send "exit\r";

expect "$homeUserName@$homeHostname:"
spawn scp -r /home/$homeUserName/rover_driver_workspace/devel/include/rover_driver_world_state/ $swarmieUserName@$swarmieHostname:~/
expect "*assword:"
send "saturn\r";
sleep 1

expect "$homeUserName@$homeHostname:"
spawn scp -r /home/$homeUserName/rover_onboard_workspace/src/ $swarmieUserName@$swarmieHostname:~/
expect "*assword:"
send "saturn\r";

sleep 1

expect "$homeUserName@$homeHostname:"
spawn scp /home/$homeUserName/rover_misc_workspace/src/rover_scripts/rover_onboard_node_launch.sh $swarmieUserName@$swarmieHostname:~/
expect "*assword:"
send "saturn\r";

spawn scp /home/$homeUserName/rover_misc_workspace/src/rover_scripts/softwareLoadOnboard.sh $swarmieUserName@$swarmieHostname:~/
expect "*assword:"
send "saturn\r";
sleep 1

#make process
#waits for previous process to end
expect "$homeUserName@$homeHostname:"
spawn ssh "$swarmieUserName@$swarmieHostname"
expect "*assword:"
send "saturn\r";

expect "$swarmieUserName@$swarmieHostname:"
send "./softwareLoadOnboard.sh\r";

interact


