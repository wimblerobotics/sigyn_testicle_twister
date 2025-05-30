import os
import time
import threading
import atexit
import subprocess
import gpiod
import ctypes
import ctypes.util

class SoftwarePWMDriver:
    """
    High-precision Software PWM driver for Raspberry Pi 5 using GPIO libgpiod interface.
    
    This implementation uses clock_nanosleep for precise timing and minimizes jitter
    even under high CPU load conditions.
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

        # Setup high-precision timing
        self._setup_high_precision_timing()

        # Register cleanup on exit
        atexit.register(self.cleanup)

        # Try to set real-time scheduling for better PWM timing
        self._setup_realtime_scheduling()

        # Open the GPIO chip and request the line
        self._open_gpio()
        self._write_value(0)  # Initialize to low

        if self.logger:
            self.logger.info(f"High-precision Software PWM (libgpiod) initialized on GPIO {self.logical_gpio}")
        else:
            print(f"High-precision Software PWM (libgpiod) initialized on GPIO {self.logical_gpio}")

    def _setup_high_precision_timing(self):
        """Setup high-precision timing using clock_nanosleep"""
        try:
            # Load libc for clock_nanosleep
            libc_name = ctypes.util.find_library('c')
            if libc_name:
                self.libc = ctypes.CDLL(libc_name)
                # Define clock_nanosleep function signature
                self.libc.clock_nanosleep.argtypes = [
                    ctypes.c_int,    # clockid_t clock_id
                    ctypes.c_int,    # int flags  
                    ctypes.POINTER(ctypes.c_long * 2),  # const struct timespec *request
                    ctypes.POINTER(ctypes.c_long * 2)   # struct timespec *remain
                ]
                self.libc.clock_nanosleep.restype = ctypes.c_int
                
                # CLOCK_MONOTONIC = 1, TIMER_ABSTIME = 1
                self.CLOCK_MONOTONIC = 1
                self.TIMER_ABSTIME = 1
                
                # Get current time function
                self.libc.clock_gettime.argtypes = [
                    ctypes.c_int,
                    ctypes.POINTER(ctypes.c_long * 2)
                ]
                self.libc.clock_gettime.restype = ctypes.c_int
                
                self.high_precision_available = True
                if self.logger:
                    self.logger.info("High-precision timing (clock_nanosleep) available")
            else:
                self.high_precision_available = False
                if self.logger:
                    self.logger.warn("High-precision timing not available, falling back to time.sleep")
        except Exception as e:
            self.high_precision_available = False
            if self.logger:
                self.logger.warn(f"Failed to setup high-precision timing: {e}")

    def _get_monotonic_time_ns(self):
        """Get current monotonic time in nanoseconds"""
        if self.high_precision_available:
            ts = (ctypes.c_long * 2)()
            self.libc.clock_gettime(self.CLOCK_MONOTONIC, ts)
            return ts[0] * 1000000000 + ts[1]
        else:
            return int(time.time_ns())

    def _sleep_until_ns(self, target_time_ns):
        """Sleep until target time with high precision"""
        if self.high_precision_available:
            ts = (ctypes.c_long * 2)()
            ts[0] = target_time_ns // 1000000000
            ts[1] = target_time_ns % 1000000000
            self.libc.clock_nanosleep(self.CLOCK_MONOTONIC, self.TIMER_ABSTIME, ts, None)
        else:
            current_time = time.time_ns()
            if target_time_ns > current_time:
                sleep_time = (target_time_ns - current_time) / 1000000000.0
                if sleep_time > 0:
                    time.sleep(sleep_time)

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

    def _setup_realtime_scheduling(self):
        """Set up real-time scheduling for better PWM timing"""
        try:
            pid = os.getpid()
            
            # Set real-time FIFO scheduling with highest priority (99)
            result = subprocess.run([
                'sudo', 'chrt', '-f', '-p', '99', str(pid)
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                if self.logger:
                    self.logger.info(f"Real-time FIFO scheduling (priority 99) set for PID {pid}")
                else:
                    print(f"Real-time FIFO scheduling (priority 99) set for PID {pid}")
            else:
                if self.logger:
                    self.logger.warn(f"Failed to set real-time scheduling: {result.stderr}")
                else:
                    print(f"Failed to set real-time scheduling: {result.stderr}")
            
            # Set CPU affinity to CPU 3 (last core) for isolation
            result = subprocess.run([
                'sudo', 'taskset', '-cp', '3', str(pid)
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                if self.logger:
                    self.logger.info(f"CPU affinity set to CPU 3 for PID {pid}")
                else:
                    print(f"CPU affinity set to CPU 3 for PID {pid}")
            else:
                if self.logger:
                    self.logger.warn(f"Failed to set CPU affinity: {result.stderr}")
                else:
                    print(f"Failed to set CPU affinity: {result.stderr}")
            
            # Set I/O priority to real-time class
            result = subprocess.run([
                'sudo', 'ionice', '-c', '1', '-n', '0', '-p', str(pid)
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                if self.logger:
                    self.logger.info(f"Real-time I/O priority set for PID {pid}")
                else:
                    print(f"Real-time I/O priority set for PID {pid}")
            else:
                if self.logger:
                    self.logger.warn(f"Failed to set I/O priority: {result.stderr}")
                    
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Real-time scheduling setup failed: {e}")
            else:
                print(f"Real-time scheduling setup failed: {e}")

    def _pwm_loop(self):
        """High-precision PWM generation loop with jitter compensation"""
        # Get initial time
        start_time = self._get_monotonic_time_ns()
        cycle_count = 0
        
        while not self._stop_event.is_set():
            # Calculate current cycle timing
            current_period_ns = self.period_ns
            current_duty_ns = self.duty_ns
            
            if current_period_ns <= 0:
                time.sleep(0.01)  # 10ms delay if invalid period
                continue
            
            # Calculate absolute timing for this cycle
            cycle_start = start_time + (cycle_count * current_period_ns)
            high_end = cycle_start + current_duty_ns
            cycle_end = cycle_start + current_period_ns
            
            # High phase
            if current_duty_ns > 0:
                self._write_value(1)
                self._sleep_until_ns(high_end)
            
            # Low phase
            if current_duty_ns < current_period_ns:
                self._write_value(0)
                self._sleep_until_ns(cycle_end)
            
            cycle_count += 1
            
            # Reset cycle counter periodically to prevent overflow
            if cycle_count > 1000000:  # Reset every 1M cycles
                start_time = self._get_monotonic_time_ns()
                cycle_count = 0
        
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
