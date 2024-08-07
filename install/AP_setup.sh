# Run with SUDO permissions!

nmcli con delete robo-retriever-AP
nmcli con add type wifi ifname wlan0 mode ap con-name robo-retriever-AP ssid robo-retriever autoconnect true
nmcli con modify robo-retriever-AP 802-11-wireless.band bg
nmcli con modify robo-retriever-AP 802-11-wireless.channel 3
#nmcli con modify robo-retriever-AP 802-11-wireless.cloned-mac-address 00:12:34:56:78:9a
nmcli con modify robo-retriever-AP ipv4.method shared ipv4.address 192.168.4.1/24
nmcli con modify robo-retriever-AP ipv6.method disabled
nmcli con modify robo-retriever-AP wifi-sec.key-mgmt wpa-psk
nmcli con modify robo-retriever-AP wifi-sec.psk "e21_ece4191"
nmcli con up robo-retriever-AP
