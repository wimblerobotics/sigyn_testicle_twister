import os
import time

class PWMDriver:
    def __init__(self, chip=0, channel=0):
        self.base = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
        self.export_path = f"/sys/class/pwm/pwmchip{chip}/export"
        self.unexport_path = f"/sys/class/pwm/pwmchip{chip}/unexport"
        self.channel = channel
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
        with open(f"{self.base}/period", 'w') as f:
            f.write(str(period_ns))
        with open(f"{self.base}/duty_cycle", 'w') as f:
            f.write(str(duty_ns))
        with open(f"{self.base}/enable", 'w') as f:
            f.write('1')

    def cleanup(self):
        with open(self.unexport_path, 'w') as f:
            f.write(str(self.channel))
