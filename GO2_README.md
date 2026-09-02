# Unitree Go2: the two computers and how to reach them

The Go2 is **two independent Linux computers** on one internal Ethernet switch (`192.168.123.0/24`). They do **not** share Wi‑Fi. The Unitree app only talks to one of them. That is why “the dog is on `IOT_ROS`” and “I can SSH the board over the LAN cable” feel like they contradict each other.

```
                    IOT_ROS  (your lab Wi‑Fi)
                          │
          ┌───────────────┼──────────────────────────────┐
          │               │  STA / Wi‑Fi-mode            │
          │               ▼  (app controls THIS radio)   │
          │         Built-in computer (MCU)              │
          │         192.168.123.161   + DHCP on IOT_ROS  │
          │               │                              │
          │               │  internal switch             │
          │               │  192.168.123.0/24            │
          │     ┌─────────┼──────────┬─────────────┐     │
          │     ▼         ▼          ▼             ▼     │
          │  Docking   RFID       LiDAR         your     │
          │  station   .2         .20           laptop   │
          │  Jetson    (optional) (optional)    .1 /     │
          │  .18                                 .222    │
          │  (no Wi‑Fi unless you add a USB dongle)      │
          └──────────────────────────────────────────────┘
```

---

## Names (what to call each board)

### 1. Docking station — `192.168.123.18` — the board you already SSH into

**Official names:** docking station, expansion dock, expansion module.

**Community names:** Jetson, NX, Orin, “the EDU computer”, “the external computer”.

**Hardware:** NVIDIA Jetson Orin NX (100 TOPS, typical EDU) or Orin Nano (40 TOPS). Ubuntu / JetPack, customized by Unitree.

**What it is for:** your programs, SLAM, ROS/DimOS onboard, LiDAR drivers, the RFID HTTP server in this repo. It is the only computer Unitree expects you to log into.

**Refer to it as:** “the docking station” or “the Jetson”. In this repo, “on the Go2” for `rfid_scanner_server.py` means **this** machine.

| | |
|---|---|
| Ethernet IP | `192.168.123.18` (static, `eth0`) |
| SSH | `ssh unitree@192.168.123.18` |
| Password | `123` |
| Physical port | RJ45 on the **rear of the expansion dock** (the cable you already use) |

### 2. Built-in computer / MCU — `192.168.123.161` — the other board

**Official names:** built-in computer, onboard computer.

**Community names:** MCU, motion-control unit, robot controller, “the internal board”.

**What it is for:** motors, balance, sport/walk modes, the Unitree Go app, camera / WebRTC, CycloneDDS (`unitree_sdk2`). It is a closed appliance. You do **not** develop on it.

**Refer to it as:** “the MCU” or “the built-in computer”. Do **not** call it “the Jetson”.

| | |
|---|---|
| Ethernet IP | `192.168.123.161` (static) |
| Wi‑Fi IP | DHCP on `IOT_ROS` when the app is in **Wi‑Fi-mode** (STA) |
| SSH | Not supported. Port 22 is closed, or credentials are unpublished. |
| App / WebRTC | This is the machine the phone talks to |

Reserved addresses on the internal LAN (do not assign these to your laptop): `.18` (Jetson), `.161` (MCU), `.2` (Vulcan RFID in this project), `.20` (Mid-360 / Hesai if fitted). `.222` or `.51` are the usual laptop choices; `.1` works if nothing else uses it as a gateway.

---

## Ethernet: how to connect to each

1. LAN cable: robot RJ45 ↔ laptop.
2. Laptop IPv4 **manual**: e.g. `192.168.123.222/24` (or your current `192.168.123.1/24`). No gateway required.

```bash
ping -c 2 192.168.123.18     # docking station — should reply
ssh unitree@192.168.123.18   # password: 123

ping -c 2 192.168.123.161    # MCU — see ICMP section below
```

From inside the Jetson you are on the same switch, so you can also:

```bash
ping -c 2 192.168.123.161
ping -c 2 192.168.123.2      # RFID reader, if fitted
```

You cannot SSH the MCU. Talk to it with the Unitree app, WebRTC (`ROBOT_IP`), or `unitree_sdk2` / DDS — not a shell.

---

## Why ICMP to the MCU fails (and why you usually cannot “enable” it)

Two different failures get mixed together.

### A. You ping `192.168.123.161` from `IOT_ROS`

That **must** fail. `.161` only exists on the robot’s private Ethernet. Your Wi‑Fi network has no route into `192.168.123.0/24`. This is not a broken ping; it is the wrong address for that network.

The MCU’s address **on `IOT_ROS`** is a **different** IP (DHCP). Ping that one, not `.161`.

### B. You ping `192.168.123.161` from the Ethernet cable and get no reply

Then either ICMP is filtered on the MCU, or you are not actually L2-adjacent.

Check from the Jetson first:

```bash
ssh unitree@192.168.123.18
ping -c 3 192.168.123.161
```

| Result | Meaning |
|---|---|
| Jetson can ping `.161`, laptop cannot | Laptop NIC/firewall/wrong interface. Confirm `ip route` shows `192.168.123.0/24` on the ethernet dongle. |
| Neither can ping `.161` | MCU is dropping ICMP. Common on some firmware. The robot can still walk and still speak DDS. |
| Ping works | ICMP is allowed. You still cannot SSH. |

There is **no supported switch** to turn ICMP on. You have no shell on the MCU, so you cannot change its firewall. Do not treat ping as the health check for this board.

**Better liveness checks**

- Unitree app still connected and the dog walks.
- From a host on `192.168.123.0/24`, run a `unitree_sdk2` example (DDS to `.161`).
- DimOS / WebRTC using the MCU’s **Wi‑Fi** IP (`ROBOT_IP`).

The Jetson at `.18` should always answer ping on the cable. If `.18` pings and `.161` does not, the docking station is fine and the MCU is simply not echoing ICMP.

---

## Can you SSH the MCU?

**No, not in any supported way.** Unitree does not publish a login. On stock firmware SSH is closed or the password is unknown. Jailbreaks exist in the wild; they are unsupported, they break OTA, and they are out of scope here.

Use:

| Goal | Use |
|---|---|
| Shell, install packages, run `rfid_scanner_server.py` | SSH the **Jetson** (`.18`) |
| Walk / sport / camera / app | MCU via app or WebRTC |
| Low-level / high-level SDK | DDS to `192.168.123.161` from a host on the internal LAN (laptop or Jetson) |

---

## App Wi‑Fi-mode vs AP-mode: which device joined `IOT_ROS`?

The Unitree Go app configures **only the MCU’s built-in Wi‑Fi radio**. It never configures the Jetson.

| App mode | What happens |
|---|---|
| **AP mode** | MCU broadcasts a hotspot (SSID like `Unitree_Go…`, default password often `00000000`). Phone joins the dog. MCU is typically `192.168.12.1` on that hotspot. Jetson is **not** on that hotspot. |
| **Wi‑Fi-mode (STA)** | MCU joins **your** AP (`IOT_ROS` / `aaaaaaaa`). MCU gets a DHCP lease on `IOT_ROS`. Jetson still has **only** `192.168.123.18` and does **not** appear on `IOT_ROS`. |

So when you set Wi‑Fi-mode and the dog shows up on `IOT_ROS`:

- **Connected to `IOT_ROS`:** the **MCU** (built-in computer). That is the host whose IP you use for the app, WebRTC, and `ROBOT_IP`.
- **Not on `IOT_ROS`, no ICMP there:** the **docking station (Jetson)**. It never received the SSID. It is still only on the internal cable network.

That is the device that “doesn’t receive ICMP” on Wi‑Fi: it is not a Wi‑Fi client at all.

---

## Put each computer on `IOT_ROS` and find its IP

### MCU (already on `IOT_ROS` via the app)

1. Phone: Unitree Go → Wi‑Fi-mode → SSID `IOT_ROS`.
2. Read the IP from the app (connection / robot info), **or** your AP’s DHCP lease list, **or** scan:

```bash
# from a laptop that is also on IOT_ROS
ip neigh
# or
nmap -sn 192.168.0.0/24    # use YOUR IOT_ROS prefix
```

Look for a new lease when the dog connects (hostname is often generic). Confirm it is the MCU by opening the app at the same time, or by using that IP as `ROBOT_IP` for WebRTC.

Ping to this DHCP IP **may** work. If it does not, the MCU can still be alive (app + WebRTC). SSH to this IP will not give you a usable login.

### Jetson (does not join until you add Wi‑Fi)

The dock has **no usable STA radio**. Plug a Linux-compatible USB Wi‑Fi adapter into the docking station, then SSH in over the **cable** and join `IOT_ROS`:

```bash
ssh unitree@192.168.123.18   # still on the LAN cable

nmcli device                # wait until wlan0 (or similar) appears
nmcli device wifi list
sudo nmcli device wifi connect "IOT_ROS" password "aaaaaaaa"
ip -4 addr show             # note the IOT_ROS address, e.g. 192.168.x.y
```

Persist it (NetworkManager usually does). Then from any host on `IOT_ROS`:

```bash
ping -c 2 192.168.x.y
ssh unitree@192.168.x.y     # same password 123
```

Discover that IP later from the AP DHCP list, or from the cable session (`ip -4 addr show`). Do **not** expect `192.168.123.18` to exist on `IOT_ROS`.

Optional: after the Jetson is on `IOT_ROS`, you can NAT/forward from Wi‑Fi into `192.168.123.0/24` so a laptop can reach `.161` and the RFID reader without a cable. That is extra routing (`ip_forward`, iptables `FORWARD`/`MASQUERADE`). By default the Jetson’s `FORWARD` policy is `DROP`, so it will **not** happen until you add those rules. Not required for DimOS if the laptop only needs WebRTC + `http://<jetson-wifi-ip>:8765`.

---

## What DimOS needs (two IPs, two computers)

DimOS on the **laptop** talks to **both** boards over `IOT_ROS`. They are not interchangeable.

```
Laptop  (also on IOT_ROS)
  │
  ├── ROBOT_IP              ──WebRTC──►  MCU          (walk, camera, sport)
  └── RFID_API_BASE :8765   ──HTTP───►  Jetson        (rfid_scanner_server.py)
                                          └── Ethernet ► reader 192.168.123.2
```

Set them in `.env` (this repo loads it) or export them in the shell:

| Variable | Whose IP | What DimOS does with it |
|---|---|---|
| `ROBOT_IP` | **MCU** Wi‑Fi address on `IOT_ROS` | `GO2Connection` / WebRTC: video, lidar-over-WebRTC, and motion commands. Same path the Unitree app uses. |
| `RFID_API_BASE` | **Jetson** Wi‑Fi address, with path | HTTP poll of `rfid_scanner_server.py` (`http://<jetson>:8765/api/v1`). The Jetson, not the laptop, talks to the reader at `192.168.123.2`. |

DimOS does **not** need `192.168.123.18` or `192.168.123.161` when you run from the laptop on Wi‑Fi. Those are cable-only addresses. It also does **not** need a route to the RFID reader; only the Jetson does.

### This lab (`IOT_ROS`)

| Device | `IOT_ROS` IP (DHCP — can change) | Put it in |
|---|---|---|
| Docking station (Jetson) | `10.42.200.240` | `RFID_API_BASE=http://10.42.200.240:8765/api/v1` |
| MCU | the **other** Go2 lease (example in `.env`: `10.42.203.26`) | `ROBOT_IP=<mcu-iot-ros-ip>` |
| Laptop | whatever `IOT_ROS` gave you | nothing — just be on the same SSID |

Example `.env` on the laptop:

```bash
ROBOT_IP=10.42.203.26
RFID_API_BASE=http://10.42.200.240:8765/api/v1
```

Replace `10.42.203.26` with whatever the MCU actually has **today**. Confirm in the Unitree app, the AP DHCP list, or:

```bash
./scripts/find_go2_ip.sh          # MCU, by Wi‑Fi MAC
ssh unitree@10.42.200.240 'hostname -I'   # Jetson; you already know this one
```

### Sanity checks before `dimos run`

Laptop must be on `IOT_ROS`. Then:

```bash
# Jetson HTTP API (RFID). Must succeed or RfidModule has nothing to poll.
curl http://10.42.200.240:8765/api/v1/health

# MCU WebRTC target. Ping is optional (MCU may drop ICMP); the app / DimOS
# connection is the real test. Do NOT point this at 10.42.200.240.
ping -c 2 "$ROBOT_IP"
```

Then:

```bash
uv run dimos run unitree-go2-rfid
```

### Wrong IPs (common mix-ups)

| If you set… | What breaks |
|---|---|
| `ROBOT_IP=10.42.200.240` (Jetson) | WebRTC / camera / walking fail. The Jetson is not the Unitree app endpoint. |
| `ROBOT_IP=192.168.123.161` from Wi‑Fi | Unreachable. `.161` is cable-only. |
| `ROBOT_IP=192.168.123.18` | That is the Jetson’s **cable** IP, not WebRTC. |
| `RFID_API_BASE=http://<mcu>:8765/...` | Nothing listens on the MCU. Flask runs on the Jetson. |
| `RFID_API_BASE=http://192.168.123.2:...` | That is the reader, not the HTTP API. Laptop usually cannot reach it. |
| Same IP for both variables | Only correct if you somehow run the scanner on the MCU (you do not). |

DHCP leases on `IOT_ROS` move after reboot unless you reserve them on the AP. If DimOS “cannot find the robot” after a power cycle, re-read both IPs; do not assume `.240` and `.26` are still valid.

### `rfid-demo` vs full Go2 stack

| Command | Needs `ROBOT_IP` | Needs `RFID_API_BASE` |
|---|---|---|
| `dimos run rfid-demo` | no | yes (Jetson `:8765`) |
| `dimos run unitree-go2-rfid` | yes (MCU) | yes (Jetson `:8765`) |
| `python -m dimos_rfid go2-dataset` | yes | yes |

---

## What to use for this repo

| Traffic | Target |
|---|---|
| `ssh unitree@…` , `rfid_scanner_server.py`, port `8765` | **Jetson** — cable `.18` or `IOT_ROS` `10.42.200.240` |
| `ROBOT_IP`, Unitree app, WebRTC camera/control | **MCU** — its `IOT_ROS` DHCP IP (not `.240`) |
| Vulcan reader `192.168.123.2` | Only hosts on the internal Ethernet (Jetson always; laptop only with a cable, or with forwarding set up) |

---

## Quick cheat sheet

| | Docking station (Jetson) | Built-in computer (MCU) |
|---|---|---|
| Call it | docking station / Jetson / NX | MCU / built-in computer |
| Cable IP | `192.168.123.18` | `192.168.123.161` |
| SSH | yes, `unitree` / `123` | no |
| App Wi‑Fi-mode | **not** joined | **this** is what joins `IOT_ROS` |
| Ping on cable | should work | often works; if not, use DDS/app |
| Ping on `IOT_ROS` | yes after USB Wi‑Fi (`10.42.200.240` here) | ping the DHCP IP, not `.161` |
| How to get Wi‑Fi IP | `ip -4 addr` on the Jetson, or DHCP list | app, `find_go2_ip.sh`, or DHCP list |
| DimOS variable | `RFID_API_BASE=http://<this>:8765/api/v1` | `ROBOT_IP=<this>` |
