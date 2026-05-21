# Jetson NX Auto Time Sync with systemd-timesyncd

This README explains how to enable automatic time synchronization on the Jetson NX using `systemd-timesyncd`.

The setup was tested on the Unitree Jetson payload. The system time sync was successfully enabled with NTP.

Verified result:

```bash
timedatectl
```

```text
System clock synchronized: yes
NTP service: active
```

The active NTP server was:

```text
133.243.238.163 (ntp.nict.jp)
```

---

## Problem

The Jetson NX clock was not updating automatically.

Initial status:

```text
System clock synchronized: no
NTP service: inactive
```

The `systemd-timesyncd` service was also not enabled:

```text
disabled
inactive
```

Because the clock was not synchronized, time-dependent features such as logs, network services, certificates, ROS data timestamps, and remote debugging could use the wrong time after reboot.

---

## Enable NTP Auto Update

Run:

```bash
sudo timedatectl set-ntp true
sudo systemctl enable --now systemd-timesyncd
```

Check the service:

```bash
systemctl is-enabled systemd-timesyncd
systemctl is-active systemd-timesyncd
```

Expected output:

```text
enabled
active
```

---

## Configure Reliable NTP Servers

The default pool server timed out on this system. Internet and DNS worked, but `systemd-timesyncd` could not get a reply from the first default NTP pool server.

Create a drop-in config:

```bash
sudo mkdir -p /etc/systemd/timesyncd.conf.d
sudo tee /etc/systemd/timesyncd.conf.d/jetson-ntp.conf >/dev/null <<'EOF'
[Time]
NTP=ntp.nict.jp time.google.com ntp.ubuntu.com pool.ntp.org
FallbackNTP=ntp.ubuntu.com time.cloudflare.com
EOF
```

Restart the service:

```bash
sudo systemctl restart systemd-timesyncd
```

---

## Verify Time Sync

Check basic status:

```bash
timedatectl
```

Expected:

```text
System clock synchronized: yes
NTP service: active
```

Check detailed NTP status:

```bash
timedatectl timesync-status
```

Example successful output:

```text
Server: 133.243.238.163 (ntp.nict.jp)
Poll interval: 32s
Leap: normal
Version: 4
Stratum: 1
Reference: NICT
Packet count: 1
```

Check service status:

```bash
systemctl --no-pager --full status systemd-timesyncd
```

Example successful message:

```text
Initial synchronization to time server 133.243.238.163:123 (ntp.nict.jp).
```

---

## Optional: Set Timezone

The tested system used:

```text
Asia/Tokyo
```

To change the timezone, use one of these:

```bash
sudo timedatectl set-timezone Asia/Tokyo
```

or:

```bash
sudo timedatectl set-timezone Asia/Jakarta
```

Then verify:

```bash
timedatectl
```

---

## Troubleshooting

If NTP is active but not synchronized:

```text
System clock synchronized: no
NTP service: active
Packet count: 0
```

Check internet connectivity:

```bash
ping -c 3 8.8.8.8
ping -c 3 ntp.nict.jp
```

Check DNS:

```bash
resolvectl status
```

Check logs:

```bash
journalctl -u systemd-timesyncd --no-pager -n 50
```

If logs show timeout to an NTP server, keep the custom drop-in config above and restart:

```bash
sudo systemctl restart systemd-timesyncd
```

---

## Final Confirmed Status

After applying the setup:

```text
systemd-timesyncd: enabled
systemd-timesyncd: active
System clock synchronized: yes
NTP service: active
NTP server: ntp.nict.jp
```

This means the Jetson NX will automatically update time after boot when network access is available.
