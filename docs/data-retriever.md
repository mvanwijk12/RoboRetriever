# Data Retriever Setup
This is the setup instructions for the data retriever RPi Zero 2W. This RPi is to be used to help connect image data for training YOLO computer vision models.

# Login
WiFi SSID: `data-retriever` <br>
WiFi Password: `e21_ece4191` <br>
Hostname: `data-retriever.local` <br>
Username: `e21` <br>
Password: `e21`

# Data Acquision
To acquire images run the file `capture_imgs.sh` in the home directory. Note an internet connection is required to successfully upload to G-Drive (https://drive.google.com/drive/folders/1UyrnengsetC2KwwI0Q-vrXF5I9H4RCKW). 

Without an internet connection the files will be stored locally on the RPi's SD Card.

Currently the RPi is configured to automatically connect to a WiFi hotspot generated from my (Matt's) phone. If this network is not available, the RPi will generate its own WiFi network without an internet connection (credentials above).

For convenience and mobile use, an alias has been setup to run the `capture_imgs.sh` from any directory. Execute the `aq` command to start the data acquision process. 

## References: 
https://gist.github.com/BollaBerg/7a3fc9744d5bf6eb16f8aab5928df755

https://askubuntu.com/questions/165679/how-to-manage-available-wireless-network-priority

https://github.com/nurdtechie98/drive-cli

https://stackoverflow.com/questions/65396850/how-to-handle-app-is-temporarily-blocked-from-logging-in-with-your-google-accou/65507155#65507155 

https://github.com/nurdtechie98/drive-cli/issues/86

https://github.com/nurdtechie98/drive-cli/issues/114

https://raspberrypi.stackexchange.com/questions/105130/how-do-i-upload-files-to-google-drive-automatically-from-raspberry-pi-or-from-pu

https://www.baeldung.com/linux/create-alias 

