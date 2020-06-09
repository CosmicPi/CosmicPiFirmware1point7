# Cosmic Pi V.1.7 Calibration procedure

Calibration in 7 easy steps!

## 0) Connect the GPS antennae and make sure it can see the sky (dangle it out of a window if necessary!).

## 1) Power up the device and connect to the open "CosmicPi" hotspot or create a wifi network called 'CosmicPiTest' without a password.

## 2) Log in via SSH using a client. On windows we recommend Putty, which is free to download. On Linux you can use the commandline SSH tool. On Mac you can also use Putty.
If you are connected to the "CosmicPi" network, you should use the following credentials to connect:
IP address: 192.168.0.1
Port: 22
Username: cosmicpi
Password: MuonsFROMSp8ce
If you have created your own "CosmicPiTest" network, or connected the device to another wifi network, you should check the DHCP server for the correct IP address. 

## 3) When connected to the device via ssh, there are three scripts in the cosmicpi user directory. These should be executed as follows:
First - upload the latest firmware to the cosmic pi custom board processor
"sudo ./flash-pi.sh"
Second - you can reset the custom processor
"sudo ./reset.sh"
Third you can start the calibration mode
"sudo ./cal-one.sh"
after executing this you will be prompted to run Minicom, a terminal utility.
"sudo minicom"
After a short delay (10s for GPS to initialise) you will be presented with the calibration screen menu.
Set the values for:
HV Channel 1
HV Channel 2
DAC Channel 1
DAC Channel 2
by following the on screen commands. Ballpark settings for the Version 1.7 Cosmic Pi: DAC = 700 (out of 1024), apply to both channels HV = 160 to 200 (lower = higher voltage).
The aim is to get a reasonable number of events with the high voltage as low as possible (but still working) by setting the threshold, without overwhelming the unit with noise (which causes a crash, requiring a reset).
When you have set the values, you can get an update on the number of cosmic rays captured by pressing 9 to enable counting via interrupt. Once interrupts are enabled, the event count will update each time the menu is refreshed, press 0 and the menu will update the event count. When you have an approximate rate of 5 new events per second, the unit is correctly configured. Be careful not to set the HV values too low, as this causes noise in the sensors. 
When you are happy with the event rate, press 5 to write to EEPROM and store the values in the Cosmic Pi unit. 

## 4)To exit minicom press CTRL+A then X and select yes.

## 5) On returning to the command line, you can now cancel the calibration mode as follows:
"sudo ./cal-two.sh"

## 6) You are finished, you disconnect from the SSH session by typing "exit" and pressing enter. The performance of your detector can be checked via the web interface, at either http://192.168.0.1 or http://cosmicpi.local depending upon your system configuration if you are using the "CosmicPi" wifi network, or via the device IP address from your network DHCP server or http://cosmicpi.local if supported by your system.
