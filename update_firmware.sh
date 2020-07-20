echo "This script will delete the current firmware"
echo "and download the latest version from the internet"
echo "you must have an internet connection for it to work"
echo "otherwise it will delete your current firmware"
echo "which will make re-flashing the detector impossible"
echo "until you are able to reconnect to the internet"
echo "there is a copy of the firmware that shipped with"
echo "your unit in the /backup folder just in case."
read -p "Continue (y/n)?" CONT
if [ "$CONT" = "y" ]; then
  echo "starting"
sudo rm -rf CosmicPiFirmware1point7
git clone https://github.com/CosmicPi/CosmicPiFirmware1point7.git
echo "if the download was successful you can now run"
echo "sudo ./flash_pi.sh to flash the new firmware to"
echo "your detector!"    
else
echo "try again when you have an internet connection"
fi
