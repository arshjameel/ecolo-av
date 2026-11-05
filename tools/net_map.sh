#!/usr/bin/env bash
# Map the network this computer is on: tools/net_map.py does the work (read its docstring).
# usage: pixi run bash tools/net_map.sh [--interface en7] [--listen 20] [--subnet 192.168.1.0/24] [--out net-map.md]
#
# Why a wrapper: on a Mac, anything started by Python sees an empty ARP table (macOS's
# Local Network privacy), so the MAC addresses went missing. Here `arp` runs from bash,
# after Python's ping has filled the table. Portable: bash 3.2, no GNU-only flags.
cd "$(dirname "$0")/.."
python3 tools/net_map.py --ping-only "$@" || exit
table=$(mktemp)
trap 'rm -f "$table"' EXIT
arp -an > "$table" 2>/dev/null || ip neigh > "$table" 2>/dev/null
python3 tools/net_map.py --arp "$table" "$@"
