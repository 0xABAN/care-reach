# Pi network routes

The Pi prefers Ethernet for IPv4 and IPv6 internet traffic. NetworkManager's
`care-reach-wired-uplink` profile obtains an address by DHCP on `eth0`, uses
route metric 100, and has autoconnect priority 100. Wi-Fi remains connected
with higher route metrics as fallback. The older
`care-reach-direct-ethernet` link-local profile is preserved.

Both interfaces can be on the same LAN. In that case, advertising both
addresses as `pi.local` can send clients to the Wi-Fi address even though
Ethernet is the preferred route. The NetworkManager dispatcher script
[`90-care-reach-mdns-interface`](../deploy/pi/90-care-reach-mdns-interface)
limits Avahi's `pi.local` advertisement to Ethernet while Ethernet has a
DHCP address, and switches it to Wi-Fi when Ethernet goes down.

Install the script on the Pi from this worktree:

```bash
sudo cp -n /etc/avahi/avahi-daemon.conf /etc/avahi/avahi-daemon.conf.care-reach.bak
sudo install -m 755 deploy/pi/90-care-reach-mdns-interface /etc/NetworkManager/dispatcher.d/
sudo /etc/NetworkManager/dispatcher.d/90-care-reach-mdns-interface manual manual
```

Check that `ip route get 1.1.1.1` and `ip -6 route get
2606:4700:4700::1111` use `eth0`, while Wi-Fi's default routes remain with
higher metrics. Tailscale SSH uses `robot@care-reach-robot-pi`, and local
clients can use `robot@pi.local`.
