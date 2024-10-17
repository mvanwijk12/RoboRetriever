# Raspberry Pi Setup
## Install OS
To setup the Raspberry Pi 4B we must first flash the operating system onto the SD card. Our chosen operating system is Raspberry Pi OS Lite Bookwork 64-bit. Download the Raspberry Pi imager from the website below and follow the instructions to install Raspberry Pi OS Lite Bookwork 64-bit. Make sure to enable SSH over password and create a login with username `e21` and password `e21`, this can be done in the flash configuration menu in the Raspberry Pi Imager.
https://www.raspberrypi.com/software/ 

## Install Git
The next step to is to install git onto the Raspberry Pi. Follow the procedure outlined in `git_rpi_setup.md` (in this docs directory).

## Clone the GitHub Repository
Using SSH </br>
`git clone git@github.com:mvanwijk12/RoboRetriever.git` </br>

Using HTTPS </br>
`git clone https://github.com/mvanwijk12/RoboRetriever.git` </br>

## Install Required Packages 
The next step to is to install the required packages onto the Raspberry Pi. Follow the procedure outlined in `RPi_install.md` (in this docs directory).

## Setup the Raspberry Pi as an Access Point
1. Change directory to `RoboRetriever/docs`
2. Make the AP_setup.sh file executable and run with sudo permissions </br>
`chmod +x AP_setup.sh` </br>
`sudo ./AP_setup.sh` </br>
3. This will create a WiFi accesspoint with SSID `robo-retriever` and password `e21_ece419` (configured in AP_setup.sh). Connecting to this network will allow ssh access.

# Off-robot Computer Setup
Follow the guide specified in `compute_setup.md` (in this docs directory).
