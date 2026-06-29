#!/usr/bin/env bash
# Find the Unitree Go2 on the local Wi-Fi/LAN by its Wi-Fi MAC address.
#
# Usage:
#   ./scripts/find_go2_ip.sh
#   GO2_WIFI_MAC=c8:fe:0f:ee:f7:59 ./scripts/find_go2_ip.sh
#   export ROBOT_IP=$(./scripts/find_go2_ip.sh)
#
# Prints the IPv4 address on stdout and exits 0 if found, 1 otherwise.

set -euo pipefail

GO2_MAC="${GO2_WIFI_MAC:-c8:fe:0f:ee:f7:59}"
GO2_MAC_NORM="$(echo "$GO2_MAC" | tr '[:upper:]' '[:lower:]' | tr -d ' -')"

mac_matches() {
    local entry="${1:-}"
    local norm
    norm="$(echo "$entry" | tr '[:upper:]' '[:lower:]' | tr -d ' :-')"
    [[ -n "$norm" && "$norm" == *"$GO2_MAC_NORM"* ]]
}

lookup_arp() {
    while read -r ip _ rest; do
        local lladdr=""
        for word in $rest; do
            if [[ "$word" == lladdr ]]; then
                continue
            fi
            if [[ "$word" =~ ^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$ ]]; then
                lladdr="$word"
                break
            fi
        done
        if [[ -z "$lladdr" ]]; then
            continue
        fi
        if mac_matches "$lladdr"; then
            echo "$ip"
            return 0
        fi
    done < <(ip neigh show 2>/dev/null || true)
    return 1
}

try_dimos_discover() {
    command -v dimos >/dev/null 2>&1 || return 1

    local line ip mac
    while IFS= read -r line; do
        [[ "$line" == SOURCE* ]] && continue
        [[ -z "${line// }" ]] && continue
        ip="$(echo "$line" | awk '{print $3}')"
        mac="$(echo "$line" | awk '{print $4}')"
        [[ -z "$ip" || "$ip" == "-" ]] && continue
        if mac_matches "$mac"; then
            echo "$ip"
            return 0
        fi
    done < <(dimos go2tool discover --lan -t 12 2>/dev/null || true)
    return 1
}

default_lan_subnet() {
    local iface cidr
    iface="$(ip route show default 0.0.0.0/0 2>/dev/null | awk '{print $5; exit}')"
    [[ -n "$iface" ]] || return 1
    cidr="$(ip -o -4 addr show dev "$iface" 2>/dev/null | awk '{print $4; exit}')"
    [[ -n "$cidr" ]] || return 1
    echo "$cidr"
}

ping_sweep() {
    local cidr="${1:-}"
    local base="${cidr%/*}"
    local prefix="${base%.*}"

    if command -v fping >/dev/null 2>&1; then
        fping -a -g "${prefix}.1" "${prefix}.254" 2>/dev/null >/dev/null || true
        return 0
    fi

    local i
    for i in $(seq 1 254); do
        ping -c 1 -W 1 "${prefix}.${i}" >/dev/null 2>&1 &
        if (( i % 50 == 0 )); then
            wait
        fi
    done
    wait
}

main() {
    local ip cidr

    if ip="$(lookup_arp)"; then
        echo "$ip"
        exit 0
    fi

    if ip="$(try_dimos_discover)"; then
        echo "$ip"
        exit 0
    fi

    cidr="$(default_lan_subnet)" || {
        echo "find_go2_ip: no default IPv4 route" >&2
        exit 1
    }

    echo "find_go2_ip: scanning ${cidr} for ${GO2_MAC} ..." >&2
    ping_sweep "$cidr"

    if ip="$(lookup_arp)"; then
        echo "$ip"
        exit 0
    fi

    echo "find_go2_ip: Go2 not found on the local network (MAC ${GO2_MAC})" >&2
    echo "find_go2_ip: ensure the dog is on the same Wi-Fi and powered on" >&2
    exit 1
}

main "$@"
