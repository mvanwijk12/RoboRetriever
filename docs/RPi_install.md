# Raspberry Pi Install
The Raspberry installation process is simple and only requires installation of a few pip and apt packages.

Firstly make sure pip in installed using the default RPi package manager apt </br>
`sudo apt install pip`

Install the pigpio library for precision control of the RPi's GPIO </br>
`sudo apt-get install pigpio python3-pigpio` </br>

Change directories to the RoboRetriever/src/raspberrypi directory </br>
`cd RoboRetriever/src/raspberrypi`

Install the required packages using pip </br>
`pip install -r requirements.txt`
