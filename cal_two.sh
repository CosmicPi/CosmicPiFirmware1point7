


#reset
gpio -g write 4 0

#set the cal flag high
gpio -g write 17 0
sleep 1 #wait 1 second
gpio -g write 4 1
sleep 1

stty -F /dev/serial0 19200

#debug via console
#sudo minicom

#restart the service
sudo systemctl start CosmicPi-detector.service
