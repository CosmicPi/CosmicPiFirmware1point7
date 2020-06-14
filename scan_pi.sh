sudo systemctl stop CosmicPi-detector.service
stty -F /dev/serial0 115200

cd CosmicPi1point7scan

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

#set the boot flag high
gpio -g write 18 1
sleep 1 #wait 1 second
gpio -g write 4 1
sleep 1
stm32flash -v -w cosmicpi17noise.bin /dev/serial0

sudo systemctl start CosmicPi-detector.service
stty -F /dev/serial0 19200

#reset
gpio -g write 18 0
gpio -g write 4 0
sleep 1
gpio -g write 4 1
