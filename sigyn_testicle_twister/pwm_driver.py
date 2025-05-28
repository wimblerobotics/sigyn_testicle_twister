import os
import time

class PWMDriver:
    def __init__(self, chip=0, channel=0, logger=None):
        self.base = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
        self.export_path = f"/sys/class/pwm/pwmchip{chip}/export"
        self.unexport_path = f"/sys/class/pwm/pwmchip{chip}/unexport"
        self.channel = channel
        self.logger = logger
        self._export()

    def _export(self):
        if not os.path.exists(self.base):
            with open(self.export_path, 'w') as f:
                f.write(str(self.channel))
            # Wait for sysfs to create the pwm directory
            for _ in range(10):
                if os.path.exists(self.base):
                    break
                time.sleep(0.1)

    def set_pwm(self, period_ns, duty_ns):
        if self.logger:
            self.logger.info(f"[DEBUG] set_pwm called with period_ns={period_ns}, duty_ns={duty_ns}")
        else:
            print(f"[DEBUG] set_pwm called with period_ns={period_ns}, duty_ns={duty_ns}")
        try:
            # Disable PWM before changing period/duty_cycle
            with open(f"{self.base}/enable", 'w') as f:
                f.write('0')
        except Exception as e:
            if self.logger:
                self.logger.warn(f"[WARN] Could not disable PWM before update: {e}")
            else:
                print(f"[WARN] Could not disable PWM before update: {e}")
        try:
            with open(f"{self.base}/period", 'w') as f:
                f.write(str(period_ns))
        except Exception as e:
            if self.logger:
                self.logger.error(f"[ERROR] Failed to write period: {e}")
            else:
                print(f"[ERROR] Failed to write period: {e}")
        try:
            with open(f"{self.base}/duty_cycle", 'w') as f:
                f.write(str(duty_ns))
        except Exception as e:
            if self.logger:
                self.logger.error(f"[ERROR] Failed to write duty_cycle: {e}")
            else:
                print(f"[ERROR] Failed to write duty_cycle: {e}")
        try:
            with open(f"{self.base}/enable", 'w') as f:
                f.write('1')
        except Exception as e:
            if self.logger:
                self.logger.error(f"[ERROR] Failed to enable PWM: {e}")
            else:
                print(f"[ERROR] Failed to enable PWM: {e}")

    def cleanup(self):
        # Do not unexport the PWM channel on cleanup to allow inspection after program exit
        print("[DEBUG] cleanup called, but not unexporting PWM channel for debugging purposes.")
        # If you want to re-enable unexporting, uncomment the following lines:
        # try:
        #     with open(self.unexport_path, 'w') as f:
        #         f.write(str(self.channel))
        # except Exception as e:
        #     print(f"[ERROR] Failed to unexport PWM channel: {e}")
