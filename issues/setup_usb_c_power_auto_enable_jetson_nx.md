# Unitree Go2 Jetson Expansion Dock Type-C Power Auto-Enable

This README explains how to fix the **Expansion Dock Type-C port not supplying power** on the Unitree Go2 Jetson Payload.

The issue can occur when the Type-C enable pin is not activated during boot. According to the Unitree documentation, the enable pin is **PP.06**. On this Jetson system, `/sys/class/gpio` is not available, so the older sysfs GPIO method such as `echo 446 > /sys/class/gpio/export` cannot be used. Instead, this system uses the **GPIO character device** interface through `gpiod`.

Official Unitree reference:

https://support.unitree.com/home/en/developer/module_update

---

## Problem

The Expansion Dock Type-C port does not supply power, so devices such as a depth camera, RealSense camera, or other USB-C devices are not detected.

Example symptoms:

```bash
lsusb
lsusb -t
```

The device connected to the Type-C port does not appear.

When following the Unitree instruction:

```bash
cat /sys/class/gpio/PP.06/value
```

the following error appears:

```bash
cat: /sys/class/gpio/PP.06/value: No such file or directory
```

Checking the GPIO sysfs directory also fails:

```bash
ls /sys/class/gpio
```

Output:

```bash
ls: cannot access '/sys/class/gpio': No such file or directory
```

This means the system does not use the old `/sys/class/gpio` interface.

---

## Confirmed Type-C Enable Pin

On this system, the Type-C enable pin was found as:

```bash
gpiochip0 line 98
```

The GPIO line name is:

```bash
PP.06
```

Check with:

```bash
gpioinfo | grep -i "PP.06"
```

Example output:

```bash
line  98:      "PP.06"       unused   input  active-high
```

Then confirm the chip and line number:

```bash
gpiofind "PP.06"
```

Example output:

```bash
gpiochip0 98
```

---

## Manual Enable Test

Run as root:

```bash
sudo -i
```

Apply the register setting from the Unitree reference:

```bash
busybox devmem 0x02430030
busybox devmem 0x02430030 w 0x004
```

Enable GPIO PP.06:

```bash
gpioset -m wait gpiochip0 98=1
```

If your `gpiod` version does not support `-m wait`, use:

```bash
gpioset --mode=wait gpiochip0 98=1
```

Do not close the terminal running `gpioset -m wait`, because this mode keeps the GPIO line HIGH while the process is running.

Open another terminal and check whether the Type-C device appears:

```bash
lsusb
lsusb -t
```

If the Type-C power is successfully enabled, the USB-C device should appear.

---

## Auto Enable at Boot Using rc.local

This method follows the same idea as Unitree's `/etc/rc.local` solution, but it is adapted for systems that use `gpiod` instead of `/sys/class/gpio`.

### 1. Install gpiod

```bash
sudo -i
apt update
apt install -y gpiod
```

### 2. Create or Edit rc-local Service

Create or edit:

```bash
nano /etc/systemd/system/rc-local.service
```

Use the following content:

```ini
[Unit]
Description=/etc/rc.local Compatibility
ConditionPathExists=/etc/rc.local

[Service]
Type=forking
ExecStart=/etc/rc.local start
TimeoutSec=0
RemainAfterExit=yes
GuessMainPID=no

[Install]
WantedBy=multi-user.target
```

Do **not** add this line:

```ini
Alias=rc-local.service
```

If it already exists, remove it. Otherwise, `systemctl enable rc-local.service` may fail with an alias conflict.

---

### 3. Create `/etc/rc.local`

Create or edit:

```bash
nano /etc/rc.local
```

Use the following content:

```bash
#!/bin/sh

# Enable Unitree Expansion Dock Type-C Power
# Reference:
# https://support.unitree.com/home/en/developer/module_update

busybox devmem 0x02430030
busybox devmem 0x02430030 w 0x004

# PP.06 = gpiochip0 line 98
# Keep GPIO high in the background.
gpioset -m signal gpiochip0 98=1 &

exit 0
```

If `gpioset -m signal` is not supported, use:

```bash
gpioset --mode=signal gpiochip0 98=1 &
```

---

### 4. Make rc.local Executable

```bash
chmod +x /etc/rc.local
```

---

### 5. Enable and Start the Service

```bash
systemctl daemon-reload
systemctl enable rc-local.service
systemctl start rc-local.service
```

If this error appears:

```bash
Failed to enable unit: File /etc/systemd/system/rc-local.service already exists.
```

edit the service file again:

```bash
nano /etc/systemd/system/rc-local.service
```

Make sure the following line does not exist:

```ini
Alias=rc-local.service
```

Then run:

```bash
systemctl daemon-reload
systemctl enable rc-local.service
systemctl restart rc-local.service
```

---

## Check Status

Check the service:

```bash
systemctl status rc-local.service
```

Check the GPIO line:

```bash
gpioinfo | grep -i "PP.06"
```

Before enabling, it may look like this:

```bash
line  98:      "PP.06"       unused   input  active-high
```

After enabling, it should show that the line is used by `gpioset`, or that it is configured as an output.

Check the Type-C device:

```bash
lsusb
lsusb -t
```

---

## Reboot Test

Reboot the Jetson:

```bash
reboot
```

After booting, check again:

```bash
systemctl status rc-local.service
gpioinfo | grep -i "PP.06"
lsusb
lsusb -t
```

If the Type-C device appears automatically after boot, the auto-enable setup is successful.

---

## Notes

- The Unitree documentation uses `/sys/class/gpio`, but this system does not have that directory.
- Therefore, GPIO is controlled through `/dev/gpiochip*` using `gpiod`.
- On this system, the Type-C enable pin is `PP.06`, found as `gpiochip0 line 98`.
- The command `busybox devmem 0x02430030 w 0x004` is still used according to the Unitree reference to configure the required register/pinmux.
- Do not run multiple `gpioset` processes for the same GPIO line at the same time.
- If the Type-C port still does not supply power after PP.06 is enabled, continue hardware troubleshooting: cable, dock board, Type-C connector, or the USB-C device itself.

---

## Quick Command Summary

Manual test:

```bash
sudo -i
apt update
apt install -y gpiod

busybox devmem 0x02430030
busybox devmem 0x02430030 w 0x004

gpiofind "PP.06"
gpioset -m wait gpiochip0 98=1
```

For automatic boot enable, use `/etc/rc.local` with:

```bash
gpioset -m signal gpiochip0 98=1 &
```

---

## Reference

- Unitree Support: Comparison of Jetson Module Specifications / Expansion Dock Type-C repair method  
  https://support.unitree.com/home/en/developer/module_update
