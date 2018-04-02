#!/bin/bash

proceed=""

warn()
{
    echo "WARNING: $1 is not connected to this computer!"
    echo "ARE YOU SURE YOU WANT TO PROCEED (y/n)?"
    read proceed
}

connected()
{
    if [ -z "`rosnode list | grep -i "^/$1"`" ]
    then
        warn $1
    else
        proceed="y"
    fi
}

for robot in $@
do
    connected $robot
    if [ "$proceed" = "y" ]
    then
        ssh swarmie@$robot 'sudo reboot'
    fi
    proceed="n"
done