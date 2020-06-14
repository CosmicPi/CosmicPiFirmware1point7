#!/bin/sh

#script to reset the attached hardware

#18 = boot
# 4 = reset
#17 = flag for cal

gpio -g mode 18 out
gpio -g mode 4 out
gpio -g mode 17 out

#set the boot flag low
gpio -g write 18 0

#set the cal flag low
gpio -g write 17 0

#pulse the reset pin
gpio -g write 4 0
sleep 1 #wait 1 second
gpio -g write 4 1
