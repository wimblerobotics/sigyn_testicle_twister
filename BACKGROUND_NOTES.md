## Raspberry Pi 5 and Ubuntu 24.04 PWM Notes

- Hardware PWM on Pi 5 with Ubuntu 24.04 (kernel 6.8.0) and the RP1 chip does **not** work. All attempts to use device tree overlays (e.g., `dtoverlay=pwm`, `dtparam=audio=off` in `/boot/firmware/config.txt`) failed to provide a working hardware PWM device.
- Only software PWM using GPIO (via libgpiod) is supported and tested on this platform.
- Ignore any instructions for editing `/boot/firmware/config.txt` for PWM on Pi 5. These are not required and do not work.