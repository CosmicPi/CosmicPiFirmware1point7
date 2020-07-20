Dear user,

Welcome to the linux environment inside your cosmic pi that makes it all work.

The file you are using is included in the image we provided on your SD card. If you need to get a new image, you can find it via our website at cosmicpi.org. Be sure to download a version that matches your hardware (it will say on the front of the box).

In the /home/cosmicpi directory there are a number of things to help you.

The unit was shipped to you with the firmware pre-installed on the main board.
If you wish to change this at any time, you can, since it's all open source.

The firmware for the main board resides on github:
https://github.com/CosmicPi/CosmicPiFirmware1point7

It is written in C, using the HAL libraries from STM32.

There are several shell scripts to help you if you need to flash the device again:
reset.sh (to use type "sudo ./reset.sh") - resets the cosmic pi main board (not the raspberry pi), to be used if your device stops detecting events or starts to act strangely. Resetting is the equivalent of powering on/off the STM32, so there is no need to re-flash after a reset.

flash_pi.sh - the script we used to flash the firmware to the pi when we're setting up the device. If you need to re-set the cosmic pi to factory settings, use this:
to use type "sudo ./flash_pi.sh". Note that after using this, the flash and calibration settings are completely wiped.
You will need to re-calibrate the unit.

first_cal.sh - script to put the STM32 in calibration mode. To use type "sudo ./first_cal.sh".
When the script is completed, it will ask you to run minicom ("sudo minicom") to enter the calibration menus.
At the time of writing, the calibration settings are approximately:
DAC 1 and 2 - between 700 and 800. 700 is lower, therefore higher probability of events being detected.
HV 1 and 2 - between 200 and 180. 180 is a higher voltage, therefore higher probability of events being detected, but also noise.
Note that in the calibration program, only the total events shows at present.
There is a bug blocking the individual channels (a and b) from showing up, enabling the interrupts tends to cause a crash. You should be able to calibrate a working unit without this. Note the rolling average for events.
A correctly calibrated unit should have about 1-3 events per second in this field, which resets every 2 minutes. If you are in this range, it should work ok. The units we have tested have a wide range of acceptable parameters and still provide robust performance.
When calibrating, beware setting the values for DAC too low, and HV too low as well. It is possible to crash your unit with noise if poorly tuned. If the unit crashes (terminal becomes unresponsive in minicom), simply exit minicom and run the reset script, then restart calibration.
Do not forget to save the values you have calibrated to EEPROM before finishing calibration!
Otherwise they won't get stored. If you try to operate the unit directly after calibration (via menu), it often crashes. This is another bug, but if you exit calibration and reset the unit will perform normally.

second_cal.sh - script to take the STM32 out of calibration mode, run after you have finished in minicom, by typing "sudo ./second_cal.sh"

update_firmware.sh - script to download the latest firmware from the internet and flash your unit.
This is included so that you can easily upgrade your unit when we get some of the bugs out in future, or add extra functions. The unit as shipped is fully functional, so there is no need to do this. It will also provide you with a pathway to recovery if you want to return to our firmware after trying your own. DO NOT RUN THIS SCRIPT UNLESS YOU ARE CONNECTED TO THE INTERNET, otherwise it will erase your default firmware file on the pi (it won't flash the STM32, so your unit will still work, but you won't be able to reflash using our scripts). If you do accidentally erase the firmware, a copy of the firmware we shipped with is in the /backup directory.
Copy this to the /CosmicPiFirmware1point7 directory and the flashing scripts should work again.
Note that the update script does a partial wipe of the STM32, so your calibration settings should be retained! We'll get in touch or put a note on our website if there are upgrades available.



