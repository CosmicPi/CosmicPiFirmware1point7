# CosmicPiFirmware1point7
MbedOS firmware for the V1.7 cosmic pi
Will put the source code here as soon as I figure out how to export from mbedOS online compiler

Ballpark settings for the Version 1.7 Cosmic Pi:
DAC = 750 (typical range for opertaion 700 to 800, out of 1024), apply to both channels
HV = 185 (typical range for operation 160 to 200 lower = higher voltage)

THese scripts use stm32flash-code to flash the STM32, based on instructions I followed from here:
https://siliconjunction.wordpress.com/2017/03/21/flashing-the-stm32f-board-using-a-raspberry-pi-3/

Bug-fix list for distribution from Version 1.5 package:
1) Raspberry pi wifi now operates an independent hotspot and a wifi client. As a reminder, CosmicPiTest is a default wifi network, password CosmicPiTest - if you create this network with another device, then the unit will auto-connect on boot. E.g. an android phone hotspot.
2) Modified IP range for DHCP server 192.168.0.1 is the unit default, with issued addresses in the range .50-150.
3) Installation via PIP from the cosmicpi package. 
4) Modifications to the cosmicpi-detector service
