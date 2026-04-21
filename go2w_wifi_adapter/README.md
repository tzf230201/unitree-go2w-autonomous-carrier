# 📘 **README — Installing TP-Link Archer T3U Nano WiFi on Unitree Go2-W (Ubuntu 22 / Jetson Orin NX)**

This guide explains how to install the TP-Link Archer T3U Nano USB WiFi adapter (RTL88x2BU chipset) on the **Unitree Go2-W**, connect to WiFi, and enable permanent static IP & SSH access.

Tested on:

* **Unitree Go2-W**
* **Ubuntu 22.04 (Jetson Orin NX)**
* **TP-Link Archer T3U Nano (ID: 2357:012e)**
* **Realtek RTL88x2BU driver**

---

# 🚀 1. Verify the USB WiFi Adapter

Plug the TP-Link Archer T3U Nano into the robot.

Check if it is detected:

```bash
lsusb
```

Expected output:

```
2357:012e TP-Link 802.11ac NIC
```

---

# 🔧 2. Install Required Packages

```bash
sudo apt update
sudo apt install -y git dkms build-essential linux-headers-$(uname -r)
```

(If kernel headers are missing, Jetson will still be able to compile.)

---

# 📥 3. Download Driver (Offline or USB Transfer)

Clone driver on laptop → copy via USB, OR download ZIP directly.

```
git clone https://github.com/RinCat/RTL88x2BU-Linux-Driver.git
```

---

# 🛠 4. Build & Install Driver

Enter the driver folder:

```bash
cd ~/RTL88x2BU-Linux-Driver
```

Clean old builds:

```bash
sudo make clean
```

Compile for Jetson ARM64:

```bash
make ARCH=arm64
```

Install:

```bash
sudo make install
```

Load module:

```bash
sudo modprobe 88x2bu
```

---

# 📡 5. Verify wlan0 Interface

```bash
ip a
```

You should see:

```
8: wlan0: <UP,LOWER_UP> ...
```

If wlan0 appears → driver works.

---

# 📶 6. Scan WiFi Networks

```bash
sudo rfkill unblock all
sudo ip link set wlan0 up
sudo nmcli device wifi list
```

You should now see available SSIDs.

---

# 🌐 7. Connect to a WiFi Network

Example: connect to **TLL_RoboFi** and the password is **12345678**

```bash
sudo nmcli device wifi connect "TLL_RoboFi" password "12345678" ifname wlan0
```

Check IP:

```bash
ip a show wlan0
```

Example output:

```
inet 192.168.0.90/24
```

---

# 🔒 8. Assign a Permanent Static IP

First get the connection name:

```bash
nmcli connection show
```

Example result:

```
TLL_RoboFi
```

Set static IPv4:

```bash
sudo nmcli connection modify "TLL_RoboFi" ipv4.addresses "192.168.0.90/24"
sudo nmcli connection modify "TLL_RoboFi" ipv4.gateway "192.168.0.1"
sudo nmcli connection modify "TLL_RoboFi" ipv4.dns "8.8.8.8"
sudo nmcli connection modify "TLL_RoboFi" ipv4.method manual
```

Reconnect:

```bash
nmcli connection down "TLL_RoboFi"
nmcli connection up "TLL_RoboFi"
```

Check:

```bash
ip a show wlan0
```

`valid_lft forever` = static IP successful.

---

# 🔑 9. SSH into the Robot via WiFi

From your laptop connected to the same WiFi:

```bash
ssh unitree@192.168.0.90
```

Default password: `123` (unless changed)

---

# 💡 Optional Enhancements

### Enable auto-connect on boot:

```bash
sudo nmcli connection modify "TLL_RoboFi" connection.autoconnect yes
```

### Install mDNS to SSH using a hostname:

```bash
sudo apt install avahi-daemon
```

Then:

```bash
ssh unitree@go2.local
```

---

# ✅ Summary

You now have:

✔ TP-Link Archer T3U Nano driver installed
✔ WiFi fully working on Unitree Go2-W
✔ Connected to TLL_RoboFi
✔ Static IP: **192.168.0.90**
✔ SSH accessible from WiFi

---
sudo gedit /etc/X11/xorg.conf

#Section "Device"
#    Identifier "Configured Video Device"
#    Driver "modesetting"
#EndSection
#
#Section "Monitor"
#    Identifier "Configured Monitor"
#    HorizSync 28-80
#    VertRefresh 48-75
#EndSection
#
#Section "Screen"
#    Identifier "Default Screen"
#    Device "Configured Video Device"
#    Monitor "Configured Monitor"
#    DefaultDepth 24
#    SubSection "Display"
#        Depth 24
#        Modes "1920x1080"
#    EndSubSection
#EndSection