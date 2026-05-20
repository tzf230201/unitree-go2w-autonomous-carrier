# Virtual Display for NoMachine on Jetson (Go2W Payload)

Setup notes to make the Jetson serve a **virtual 1920×1080 desktop** over NoMachine when no physical monitor is connected to the mini PC.

## Problem

The Jetson uses the **Tegra `nvidia`** Xorg driver. With no physical monitor attached, the driver can't read an EDID → display `:0` is stuck at the fallback **640×480** resolution. Connecting via NoMachine only shows that tiny, unusable desktop.

## Solution

Enable NoMachine's built-in `CreateDisplay` feature. NoMachine spawns its own virtual X display (separate from the physical `:0`) via `nxagent` at the resolution you configure. No hardware HDMI dummy plug, no `xserver-xorg-video-dummy` driver required.

## Steps

### 1. Edit `/usr/NX/etc/server.cfg`

Locate these three keys (commented out by default) and set them as follows:

```
CreateDisplay 1
DisplayOwner "unitree"
DisplayGeometry 1920x1080
```

Or apply them all in one shot with `sed`:

```bash
sudo cp /usr/NX/etc/server.cfg /usr/NX/etc/server.cfg.bak.createdisplay

sudo sed -i -E '
  s|^#?CreateDisplay .*|CreateDisplay 1|;
  s|^#?DisplayOwner .*|DisplayOwner "unitree"|;
  s|^#?DisplayGeometry .*|DisplayGeometry 1920x1080|
' /usr/NX/etc/server.cfg
```

Verify:

```bash
grep -nE "^(CreateDisplay|DisplayOwner|DisplayGeometry) " /usr/NX/etc/server.cfg
```

Expected output:

```
CreateDisplay 1
DisplayOwner "unitree"
DisplayGeometry 1920x1080
```

### 2. Restart the NoMachine server

```bash
sudo /etc/NX/nxserver --restart
```

### 3. Verify

Confirm the virtual display exists:

```bash
ls /tmp/.X11-unix/
# you should see X1002 (or similar) alongside X0

sudo /usr/NX/bin/nxserver --list
```

Expected output:

```
Display Username Remote IP       Session ID                       Node
------- -------- --------------- -------------------------------- --------------
0       unitree  -               ...                              localhost:4000
1002    unitree  127.0.0.1       ...                              localhost:4000
```

Check the virtual display's resolution:

```bash
DISPLAY=:1002 xrandr | head -5
# Screen 0: minimum 640 x 480, current 1920 x 1080, maximum 16384 x 16384
# nxoutput0 connected primary 1920x1080+0+0 ...
```

## How to Connect

Open the NoMachine client → select the Jetson connection → the "Available sessions" list will show two entries:

- **Display 0** — physical desktop (stuck at 640×480 when no monitor is attached; ignore it)
- **Display 1002** — the 1920×1080 virtual desktop; use this one

If you later plug in a real monitor, Display 0 will automatically pick up the monitor's resolution (Tegra reads the EDID). The virtual display keeps running in the background.

## Change the Resolution

Edit the `DisplayGeometry` value in [/usr/NX/etc/server.cfg](file:///usr/NX/etc/server.cfg), then:

```bash
sudo /etc/NX/nxserver --restart
```

Format is `WxH`, e.g. `2560x1440`, `1600x900`, etc.

## Rollback

```bash
sudo cp /usr/NX/etc/server.cfg.bak.createdisplay /usr/NX/etc/server.cfg
sudo /etc/NX/nxserver --restart
```

## Related Config

- `/usr/NX/etc/server.cfg` — NoMachine server config (where `CreateDisplay`, `DisplayOwner`, `DisplayGeometry` live).
- `/usr/NX/etc/node.cfg` — node config (session types, etc.).
- `/etc/X11/xorg.conf` — Tegra Xorg config. Already contains `AllowEmptyInitialConfiguration true`, which is required so Xorg starts even without a monitor.

## Baseline State Before Setup

- OS: Ubuntu on Jetson (kernel `5.15.148-tegra`)
- NoMachine: `9.3.7-1` arm64
- User: `unitree`
- Xorg driver: `nvidia` (Tegra)
- Output ports: `DP-0`, `DP-1` (both can stay disconnected after this setup)
