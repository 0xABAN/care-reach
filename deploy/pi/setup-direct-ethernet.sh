#!/usr/bin/env bash
set -euo pipefail

profile=care-reach-direct-ethernet

if nmcli -g connection.id connection show "$profile" >/dev/null 2>&1; then
  sudo -n nmcli connection modify "$profile" \
    connection.interface-name eth0 connection.autoconnect yes \
    ipv4.method link-local ipv6.method disabled
else
  sudo -n nmcli connection add type ethernet ifname eth0 con-name "$profile" \
    connection.autoconnect yes ipv4.method link-local ipv6.method disabled
fi

sudo -n nmcli connection up "$profile"
nmcli -g IP4.ADDRESS device show eth0
