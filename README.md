# sigyn_testicle_twister

## Overview

ROS2 package for controlling a gripper servo via PWM on Raspberry Pi (tested on Pi 4/5, Ubuntu 24.04, ROS2 Jazzy).

## Setup

1. **Enable PWM in `/boot/firmware/config.txt`:**
   ```
   dtoverlay=pwm
   dtparam=audio=off
   ```
   Reboot after editing.

2. **Permissions:**  
   For now, run the node with `sudo`:
   ```
   sudo ros2 launch sigyn_testicle_twister sigyn_testicle_twister.launch.py
   ```

3. **Topic:**  
   Publish to `/cmd_vel_testicle_twister` (`geometry_msgs/msg/Twist`).  
   - `linear.x = -1000` → 1.1ms pulse (open)
   - `linear.x = 0`     → 1.5ms pulse (center)
   - `linear.x = 1000`  → 2.0ms pulse (close)

4. **Daemon/Service:**  
   To run as a daemon, create a systemd service that runs the above launch command as root.

5. **Swapping Drivers:**  
   The PWM driver is modular; replace `pwm_driver.py` for other hardware.

## Additional Setup: PWM Permissions Script

If you do not want to run the node with `sudo`, you can automate fixing permissions for the PWM sysfs files at boot using a systemd service:

1. **Create the script:**
```bash
   sudo tee /usr/local/bin/fix_pwm_permissions.sh > /dev/null <<'EOF'
#!/bin/bash
chmod 666 /sys/class/pwm/pwmchip0/export
chmod 666 /sys/class/pwm/pwmchip0/unexport
if [ -d /sys/class/pwm/pwmchip0/pwm0 ]; then
    chmod 666 /sys/class/pwm/pwmchip0/pwm0/period
    chmod 666 /sys/class/pwm/pwmchip0/pwm0/duty_cycle
    chmod 666 /sys/class/pwm/pwmchip0/pwm0/enable
fi
EOF
sudo chmod +x /usr/local/bin/fix_pwm_permissions.sh
```

2. **Create the systemd service:**
   ```bash
   sudo tee /etc/systemd/system/fix-pwm-permissions.service > /dev/null <<'EOF'
[Unit]
Description=Fix permissions for PWM sysfs files
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/fix_pwm_permissions.sh
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF
   ```

3. **Enable and start the service:**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable fix-pwm-permissions.service
   sudo systemctl start fix-pwm-permissions.service
   ```

This will ensure your user can access the PWM files after every boot, and you won't need to run the node as root.

## Gripper Control Note

- Sending a `linear.x` value of **1000** fully **opens** the gripper.
- Sending a `linear.x` value of **-1000** fully **closes** the gripper.
- A value of `0` centers the gripper.

```bash
ros2 topic pub -1 /cmd_vel_testicle_twister geometry_msgs/msg/Twist "{linear: {x: 1000.000, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Troubleshooting

- If you get permission errors, check your udev rules or run with `sudo`.
- If no PWM signal, ensure `dtparam=audio=off` and reboot.
