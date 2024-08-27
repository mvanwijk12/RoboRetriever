copy into this directory
sudo cp CameraStart.service /lib/systemd/system/CameraStart.service
sudo systemctl daemon-reload
sudo systemctl enable CameraStart.service
sudo systemctl start myscript.service