# Systemctl Service Guide

Copy the service into /lib/systemd/system </br>
`sudo cp CameraStart.service /lib/systemd/system/CameraStart.service` </br>

Reload the systemctl daemon  </br>
`sudo systemctl daemon-reload` </br>

Enable the service so it starts at system boot  </br>
`sudo systemctl enable CameraStart.service` </br>

Start the service  </br>
`sudo systemctl start myscript.service`
