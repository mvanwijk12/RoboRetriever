# Login Instructions
To configure the Raspberry Pi as an accesspoint for ssh access
1. Copy the AP_setup.sh file onto the Raspberry Pi using git or scp
   `scp "<Source Location>\AP_setup.sh" e21@robo-retriever.local:~/AP_setup.sh`
   (RPi password is `e21`)
3. Make the AP_setup.sh file executable and run with sudo permissions \
   `chmod +x AP_setup.sh` \
   `sudo ./AP_setup.sh`
4. This will create a WiFi accesspoint with SSID `robo-retriever` and password `e21_ece4191` (configured in AP_setup.sh). Connecting to this network will allow ssh access. 
5. Ssh into the pi using
   `ssh e21@robo-retriever.local`
   The password is e21
