sudo systemctl stop cosmicpi-detector.service
stty -F /dev/serial0 115200

#script to reset the attached hardware

#18 = boot
# 4 = reset
#17 = flag for cal

gpio -g mode 18 out
gpio -g mode 4 out
gpio -g mode 17 out

#set the cal flag low
gpio -g write 17 0
gpio -g write 18 0
gpio -g write 4 1


#reset
gpio -g write 4 0

#set the cal flag high
gpio -g write 17 1
sleep 1 #wait 1 second
gpio -g write 4 1
sleep 1

stty -F /dev/serial0 19200

echo "now run minicom as sudo and set up device"

#debug via console
#sudo minicom

#restart the service
#sudo systemctl start cosmicpi-detector.service

#reset everything and restart
#gpio -g write 17 0
#gpio -g write 18 0
#gpio -g write 4 0
#sleep 1
#gpio -g write 4 1
