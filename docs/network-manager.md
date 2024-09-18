# Configure Shared Connections
1. `nmcli c add con-name <name> type ethernet ifname <iface> ipv4.method shared ipv6.method ignore`
2. `nmcli c up <name>`

Where `<name>` is an arbitrary name we give to the connection and `<iface>` is the name of the interface where external devices will connect to. In this case we are using an ethernet interface (`type ethernet`) and we provide IPv4 addresses, but this extends to other interfaces and to IPv6.

When the connection is up, NM starts a DHCP server listening on `<iface>` and changes the networking configuration so we can forward packages and masquerading is enabled for the interface. Of course, for this to work we need an interface different from `<iface>` that has to have external connectivity.

# Configure WiFi Connections
1. Show status of all available network devices <br>
`nmcli d` <br>
2. Turn WiFi radio on <br>
`nmcli r wifi on ifname <iface>` <br>
3. List available WiFi networks <br>
`nmcli d wifi list ifname <iface>` <br>
4. Rescan if necessary <br>
`nmcli d wifi rescan ifname <iface>` <br>
5. Connect to WiFi <br>
`nmcli d wifi connect <SSID> password <password> ifname <iface>` <br>
6. Check connection <br>
`nmcli con show` <br>

# Reference
https://ubuntu.com/core/docs/networkmanager  
https://ubuntu.com/core/docs/networkmanager/configure-shared-connections
https://askubuntu.com/questions/1359359/nmcli-proper-way-of-scanning-wifi-networks
