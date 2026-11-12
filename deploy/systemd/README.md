# Deploying on Ubuntu 24.04

Target: an Ubuntu 24.04 machine wired to the Huayan E05-L1345_Pro controller
and the RealSense D435i, on the same network as the tablet.

Two ways in: a scripted install (recommended) or the manual steps, which are
also the reference for what the script does.

---

## 1. Scripted install

```bash
git clone <repo-url> /tmp/heacon-src
sudo /tmp/heacon-src/deploy/install-ubuntu.sh
```

The script installs packages, creates the `heacon` user, copies the tree to
`/opt/heacon`, makes a virtualenv, generates a TLS certificate, installs the
udev rules and the three systemd units, and starts them.

Re-running it updates the code and restarts the services. It does not touch
`/etc/heacon` (calibration, environment) once those exist.

---

## 2. Manual install

### 2.1 System packages

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip git \
                    libgl1 libglib2.0-0 usbutils v4l-utils openssl
```

`libgl1` and `libglib2.0-0` are what `opencv-python-headless` links against.
`v4l-utils` gives you `v4l2-ctl` for checking the camera before the app runs.

### 2.2 User and directories

```bash
sudo useradd --system --create-home --home-dir /opt/heacon --shell /usr/sbin/nologin heacon
sudo usermod -aG video heacon

sudo mkdir -p /opt/heacon /etc/heacon /var/lib/heacon /var/log/heacon
sudo chown -R heacon:heacon /opt/heacon /var/lib/heacon /var/log/heacon
sudo chown root:heacon /etc/heacon
sudo chmod 750 /etc/heacon
```

### 2.3 Code

```bash
sudo rsync -a --delete \
  --exclude .git --exclude .venv --exclude runtime \
  /tmp/heacon-src/ /opt/heacon/

sudo -u heacon python3 -m venv /opt/heacon/.venv
sudo -u heacon /opt/heacon/.venv/bin/pip install -U pip
sudo -u heacon /opt/heacon/.venv/bin/pip install -r /opt/heacon/requirements.txt
sudo -u heacon /opt/heacon/.venv/bin/pip install pyrealsense2
```

If `pyrealsense2` has no wheel for your Python, build librealsense from source
and install its Python bindings; see `requirements-camera.txt`.

### 2.4 Camera permissions

```bash
sudo cp /opt/heacon/deploy/udev/99-realsense-d435i.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then confirm the camera is visible **as the heacon user**:

```bash
sudo -u heacon /opt/heacon/.venv/bin/python - <<'PY'
import pyrealsense2 as rs
print([d.get_info(rs.camera_info.name) for d in rs.context().query_devices()])
PY
```

An empty list means the USB link is the problem, not the code: reseat the
cable, prefer a USB 3 port, and avoid unpowered hubs. This is the single most
common field failure.

### 2.5 Configuration

```bash
sudo cp /opt/heacon/.env.example /etc/heacon/heacon.env
sudo chown root:heacon /etc/heacon/heacon.env
sudo chmod 640 /etc/heacon/heacon.env
sudo editor /etc/heacon/heacon.env        # robot IP, ports
```

Calibration files are machine specific and are not in the repo. Copy the
examples and then replace them with the values measured on this arm:

```bash
sudo cp /opt/heacon/config/handeye.example.json    /etc/heacon/handeye.json
sudo cp /opt/heacon/config/color_calib.example.json /etc/heacon/color_calib.json
sudo cp /opt/heacon/config/work_home.example.json   /etc/heacon/work_home.json
```

> The shipped examples are from the Mac field machine. Using them unchanged on
> a different arm will put the tool in the wrong place. Re-measure before the
> first real service.

### 2.6 Certificate for the tablet

```bash
sudo -u heacon HEACON_RUNTIME_DIR=/var/lib/heacon /opt/heacon/scripts/gen-cert.sh
```

It includes every IP on the machine. If the tablet reaches this host through
an address added later, rerun with `FORCE=1` and the address as an argument.

### 2.7 Speech model (optional)

```bash
sudo -u heacon HEACON_RUNTIME_DIR=/var/lib/heacon /opt/heacon/scripts/fetch-voice-model.sh
```

65 MB. Skipping it only disables the tablet's voice commands.

### 2.8 Services

```bash
sudo cp /opt/heacon/deploy/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now heacon-console heacon-tablet heacon-camera
```

Check them:

```bash
systemctl status heacon-console heacon-tablet heacon-camera
journalctl -u heacon-console -f
tail -f /var/log/heacon/camera.log
```

---

## 3. Verify

Run these in order. Each one isolates a different layer, so a failure tells
you where to look.

```bash
# 1. Config resolves to the right paths and ports
sudo -u heacon /opt/heacon/.venv/bin/python /opt/heacon/main.py --print-config

# 2. Console answers
curl -s http://127.0.0.1:8765/api/state

# 3. Camera is producing frames
curl -s -D - -o /dev/null http://127.0.0.1:8765/api/camera.jpg | grep -i x-heacon-camera
#    live      = current frame
#    stale     = USB dropped, showing the last frame; reseat the cable
#    missing   = no frame yet at all

# 4. Robot is reachable and in standby
curl -s http://127.0.0.1:8765/api/state | python3 -m json.tool | grep -E 'connected|fsm|ready'
#    fsm 33 = standby, which is what you want before starting

# 5. HTTPS door answers for the tablet
curl -sk https://127.0.0.1:8766/api/state >/dev/null && echo https ok
```

Then open `http://<host>:8765/` in a browser on the same network. The page
should show the camera and a live arm state, not "设备已离线".

---

## 4. Operating notes

- **Before the first real service**, re-measure the calibration on this arm.
  Nothing else in the deployment is as likely to cause a wrong tool position.
- **The console holds the arm in standby** only while a session is open. With
  no session it locks the arm down, which is the safe state.
- **Do not run two consoles** against one arm. Only one process may own the
  TCP session.
- **Logs** are in `/var/log/heacon/`, one file per service. They are appended,
  so rotate them with logrotate if the machine runs for months.
- **Restarting the console** drops the tablet session; the operator has to
  reconnect. Restart the camera freely, it affects nothing else.

---

## 5. Updating

```bash
git -C /tmp/heacon-src pull
sudo /tmp/heacon-src/deploy/install-ubuntu.sh
```

The script keeps `/etc/heacon` and the calibration untouched.

---

## 6. No hardware fallback

There is none. Recognition segments the body by depth and force control reads
a real sensor, so neither can be faked without changing what is being tested.
With no camera the page shows "missing"; with no arm it shows "设备已离线".
Both are correct states, not faults.
