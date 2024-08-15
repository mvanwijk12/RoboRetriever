Configure Shared Connections
https://ubuntu.com/core/docs/networkmanager/configure-shared-connections

nmcli c add con-name network-bridge type ethernet ifname enp0s31f6 ipv4.method shared ipv6.method ignore

nmcli c up network-bridge