# sigyn_testicle_twister

## Overview

ROS2 package for controlling a gripper servo via PWM on Raspberry Pi. 

**IMPORTANT**: This package now includes both hardware and software PWM drivers. **Software PWM is recommended for Raspberry Pi 5** due to hardware PWM issues with Ubuntu 24.04 kernel 6.8.0.

## Quick Start

For Raspberry Pi 5 (recommended):
```bash
sudo ros2 launch sigyn_testicle_twister sigyn_testicle_twister.launch.py use_software_pwm:=true gpio_pin:=18
```

For Raspberry Pi 4 (hardware PWM):
```bash
sudo ros2 launch sigyn_testicle_twister sigyn_testicle_twister.launch.py use_software_pwm:=false
```

## Complete Setup Guide for Ubuntu 24.04 on Raspberry Pi 5

### 1. Initial System Setup

After installing Ubuntu 24.04 on your Raspberry Pi 5, perform these steps:

#### Update the system:
```bash
sudo apt update && sudo apt upgrade -y
```

#### Install essential packages:
```bash
sudo apt install -y build-essential cmake git python3-pip
sudo apt install -y ros-jazzy-desktop ros-jazzy-geometry-msgs
```

#### Enable camera (if needed):
```bash
# Edit /boot/firmware/config.txt
sudo nano /boot/firmware/config.txt

# Add these lines:
camera_auto_detect=0
dtoverlay=imx708,cam0
# OR for older cameras:
# dtoverlay=imx219,cam0
```

#### Setup GPIO permissions:
```bash
sudo usermod -a -G gpio $USER
sudo usermod -a -G dialout $USER
# Log out and log back in for groups to take effect
```

### 2. PWM Configuration

#### For Raspberry Pi 5 (Software PWM - Recommended):
No device tree overlay changes needed! **Hardware PWM is not supported on Pi 5 with Ubuntu 24.04.** The software PWM driver handles everything. Do not attempt to edit `/boot/firmware/config.txt` for PWM on Pi 5.

#### For Raspberry Pi 4 (Hardware PWM):
Edit `/boot/firmware/config.txt`:
```bash
sudo nano /boot/firmware/config.txt

# Add these lines:
dtoverlay=pwm
dtparam=audio=off
```

### 3. ROS2 Workspace Setup

#### Create and build workspace:
```bash
mkdir -p ~/sigyn_testicle_twister_ws/src
cd ~/sigyn_testicle_twister_ws/src

# Clone this repository
git clone <your-repo-url> sigyn_testicle_twister

# Build the workspace
cd ~/sigyn_testicle_twister_ws
colcon build

# Source the workspace
echo "source ~/sigyn_testicle_twister_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. GPIO Pin Connections

| GPIO Pin | Physical Pin | Description |
|----------|-------------|-------------|
| GPIO 18  | Pin 12      | PWM Signal (default) |
| GPIO 19  | Pin 35      | Alternative PWM |
| GND      | Pin 6/14/20/25/30/34/39 | Ground |
| 5V       | Pin 2/4     | Servo Power (if needed) |

Connect your servo:
- **Signal wire** → GPIO 18 (Pin 12)
- **Power wire** → 5V (Pin 2) or external 5V supply
- **Ground wire** → GND (Pin 6)

### 5. Testing the Setup

#### Test with software PWM (Pi 5):
```bash
sudo ros2 launch sigyn_testicle_twister sigyn_testicle_twister.launch.py use_software_pwm:=true gpio_pin:=18
```

#### Test servo control:
```bash
# In another terminal:
# Open gripper
ros2 topic pub --once /cmd_vel_testicle_twister geometry_msgs/msg/Twist "{linear: {x: -1000.0}}"

# Center position
ros2 topic pub --once /cmd_vel_testicle_twister geometry_msgs/msg/Twist "{linear: {x: 0.0}}"

# Close gripper  
ros2 topic pub --once /cmd_vel_testicle_twister geometry_msgs/msg/Twist "{linear: {x: 1000.0}}"
```

### 6. Running as a Service (Optional)

Create a systemd service to run automatically:

```bash
sudo nano /etc/systemd/system/sigyn-testicle-twister.service
```

Add this content:
```ini
[Unit]
Description=Sigyn Testicle Twister ROS2 Node
After=network.target

[Service]
Type=simple
User=root
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source /home/ros/sigyn_testicle_twister_ws/install/setup.bash && ros2 launch sigyn_testicle_twister sigyn_testicle_twister.launch.py use_software_pwm:=true"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable sigyn-testicle-twister.service
sudo systemctl start sigyn-testicle-twister.service

# Check status
sudo systemctl status sigyn-testicle-twister.service
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_software_pwm` | `true` | Use software PWM (recommended for Pi 5) |
| `gpio_pin` | `18` | GPIO pin for software PWM |
| `pwm_chip` | `1` | PWM chip for hardware PWM |
| `pwm_channel` | `0` | PWM channel for hardware PWM |

## Control Interface

### Topic: `/cmd_vel_testicle_twister`
**Type**: `geometry_msgs/msg/Twist`

**Usage**:
- `linear.x = -1000` → 1.1ms pulse (fully open)
- `linear.x = 0` → 1.5ms pulse (center position)  
- `linear.x = 1000` → 2.0ms pulse (fully closed)

Values are linearly interpolated between -1000 and 1000.

## Troubleshooting

### Software PWM Issues:
1. **Permission denied**: Run with `sudo`
2. **GPIO already in use**: Check if another process is using the GPIO pin
3. **No servo movement**: Check connections and servo power supply

### Hardware PWM Issues (Pi 4):
1. **Permission denied**: Run with `sudo` 
2. **No PWM device**: Check device tree overlay in `/boot/firmware/config.txt`
3. **PWM export fails**: Reboot after config changes

### Pi 5 Specific Issues:
- Hardware PWM does **not** work due to RP1 clock driver issues in kernel 6.8.0 and Ubuntu 24.04. Attempts to use hardware PWM overlays in `/boot/firmware/config.txt` will not work.
- Use software PWM instead: `use_software_pwm:=true`
- GPIO mapping is different on Pi 5 (handled automatically by software driver)

### Checking PWM Status:
```bash
# Check hardware PWM (Pi 4)
ls -la /sys/class/pwm/

# Check GPIO status (Pi 5 software PWM)
cat /sys/kernel/debug/gpio | grep -A5 -B5 "GPIO18\|GPIO19"
```

## Development Notes

See `BACKGROUND_NOTES.md` for detailed information about the PWM implementation challenges and solutions for Raspberry Pi 5.  
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
