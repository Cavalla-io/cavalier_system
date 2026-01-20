# Cavalier System

***The master repo for our forklifts***

Everything necessary to get the forklifts running starts here

TODO:
- make install script for other repos (first_time_setup.py)
- set up system health checks

## Creating a CAN0 Systemd Service

**1. Create the service file:**

```bash
sudo vim /etc/systemd/system/can0.service
```

**2. Add the following content:**

```ini
[Unit]
Description=Setup CAN0 interface
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can0 up type can bitrate 125000
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
```

**3. Reload systemd and enable the service:**

```bash
sudo systemctl daemon-reload
sudo systemctl enable can0.service
sudo systemctl start can0.service
```

**4. Verify it's working:**

```bash
sudo systemctl status can0.service
```
