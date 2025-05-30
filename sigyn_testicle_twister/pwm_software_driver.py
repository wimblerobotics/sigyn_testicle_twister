import os
import time
import threading
import atexit
import gpiod

class SoftwarePWMDriver:
    """
    Software PWM driver for Raspberry Pi 5 using GPIO libgpiod interface.
    
    This is a workaround for hardware PWM issues on Pi 5 with Ubuntu 24.04
    where the RP1 PWM clock driver has known issues with kernel 6.8.0.
    """
    
    def __init__(self, gpio_pin=18, logger=None):
        """
        Initialize Software PWM Driver using libgpiod
        
        Args:
            gpio_pin: Logical GPIO pin number (e.g., 18 for Pin 12)
            logger: Optional ROS logger for debugging
        """
        self.logical_gpio = gpio_pin
        self.logger = logger
        self.is_running = False
        self.pwm_thread = None
        self.period_ns = 20000000  # Default 20ms (50Hz)
        self.duty_ns = 1500000     # Default 1.5ms (servo center)
        self._stop_event = threading.Event()
        self.chip = None
        self.line = None

        # Register cleanup on exit
        atexit.register(self.cleanup)

        # Open the GPIO chip and request the line
        self._open_gpio()
        self._write_value(0)  # Initialize to low

        if self.logger:
            self.logger.info(f"Software PWM (libgpiod) initialized on GPIO {self.logical_gpio}")
        else:
            print(f"Software PWM (libgpiod) initialized on GPIO {self.logical_gpio}")

    def _open_gpio(self):
        """Open the GPIO chip and request the line for output (Pi 5: always /dev/gpiochip4)"""
        try:
            chip_path = "/dev/gpiochip4"
            if not os.path.exists(chip_path):
                raise Exception(f"{chip_path} does not exist")
            chip = gpiod.Chip(chip_path)
            line = chip.get_line(self.logical_gpio)  # logical_gpio is the line offset
            line.request(consumer="sigyn_pwm", type=gpiod.LINE_REQ_DIR_OUT)
            self.chip = chip
            self.line = line
            if self.logger:
                self.logger.info(f"GPIO {self.logical_gpio} opened on {chip_path}")
            else:
                print(f"GPIO {self.logical_gpio} opened on {chip_path}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to open GPIO {self.logical_gpio} on /dev/gpiochip4: {e}")
            else:
                print(f"Failed to open GPIO {self.logical_gpio} on /dev/gpiochip4: {e}")
            raise

    def _write_value(self, value):
        """Write a value (0 or 1) to the GPIO pin using libgpiod"""
        try:
            if self.line:
                self.line.set_value(int(value))
                return True
            else:
                raise Exception("GPIO line not initialized")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to write GPIO value: {e}")
            return False

    def _pwm_loop(self):
        """Main PWM generation loop running in a separate thread"""
        while not self._stop_event.is_set():
            # Calculate timing
            period_us = self.period_ns / 1000.0  # Convert to microseconds
            duty_us = self.duty_ns / 1000.0      # Convert to microseconds
            
            if period_us <= 0:
                time.sleep(0.01)  # 10ms delay if invalid period
                continue
            
            high_time_us = duty_us
            low_time_us = period_us - duty_us
            
            # High phase
            if high_time_us > 0:
                self._write_value(1)
                time.sleep(high_time_us / 1000000.0)  # Convert to seconds
            
            # Low phase  
            if low_time_us > 0:
                self._write_value(0)
                time.sleep(low_time_us / 1000000.0)   # Convert to seconds
        
        # Ensure GPIO is low when stopped
        self._write_value(0)

    def set_pwm(self, period_ns, duty_ns):
        """
        Set PWM parameters and start/restart PWM generation
        
        Args:
            period_ns: PWM period in nanoseconds
            duty_ns: PWM duty cycle (high time) in nanoseconds
        """
        if self.logger:
            self.logger.info(f"[DEBUG] Software PWM: period_ns={period_ns}, duty_ns={duty_ns}")
        else:
            print(f"[DEBUG] Software PWM: period_ns={period_ns}, duty_ns={duty_ns}")
        
        # Validate inputs
        if period_ns <= 0 or duty_ns < 0 or duty_ns > period_ns:
            if self.logger:
                self.logger.error(f"Invalid PWM parameters: period={period_ns}ns, duty={duty_ns}ns")
            return
        
        # Update parameters
        self.period_ns = period_ns
        self.duty_ns = duty_ns
        
        # Start PWM if not already running
        if not self.is_running:
            self._start_pwm()

    def _start_pwm(self):
        """Start the PWM generation thread"""
        if self.is_running:
            return
        
        self._stop_event.clear()
        self.pwm_thread = threading.Thread(target=self._pwm_loop, daemon=True)
        self.pwm_thread.start()
        self.is_running = True
        
        if self.logger:
            self.logger.info(f"Software PWM started on GPIO {self.logical_gpio}")

    def _stop_pwm(self):
        """Stop the PWM generation"""
        if not self.is_running:
            return
        
        self._stop_event.set()
        if self.pwm_thread and self.pwm_thread.is_alive():
            self.pwm_thread.join(timeout=1.0)
        
        self.is_running = False
        self._write_value(0)  # Ensure GPIO is low
        
        if self.logger:
            self.logger.info(f"Software PWM stopped on GPIO {self.logical_gpio}")

    def cleanup(self):
        """Clean up resources"""
        self._stop_pwm()
        if self.line:
            try:
                self.line.set_value(0)
                self.line.release()
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Error during GPIO line release: {e}")
        if self.chip:
            try:
                self.chip.close()
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Error during GPIO chip close: {e}")
        if self.logger:
            self.logger.info("[DEBUG] cleanup called, GPIO released (libgpiod)")
        else:
            print("[DEBUG] cleanup called, GPIO released (libgpiod)")

    def get_status(self):
        """Get current PWM status for debugging"""
        return {
            'logical_gpio': self.logical_gpio,
            'is_running': self.is_running,
            'period_ns': self.period_ns,
            'duty_ns': self.duty_ns,
            'frequency_hz': 1000000000.0 / self.period_ns if self.period_ns > 0 else 0,
            'duty_percent': (self.duty_ns / self.period_ns * 100) if self.period_ns > 0 else 0
        }
