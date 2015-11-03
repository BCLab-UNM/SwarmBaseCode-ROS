#!/bin/sh
roscore &
rqt -s rqt_rover_gui
pkill roscore
