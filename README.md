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

## Troubleshooting

- If you get permission errors, check your udev rules or run with `sudo`.
- If no PWM signal, ensure `dtparam=audio=off` and reboot.
