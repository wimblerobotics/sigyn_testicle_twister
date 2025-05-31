import time
import math

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False

class PCA9685PWMDriver:
    """
    PCA9685 PWM driver for 16-channel servo control via I2C.
    
    The PCA9685 is a popular choice for servo control on Raspberry Pi,
    providing 16 channels of PWM with 12-bit resolution (4096 steps).
    
    Features:
    - 16 independent PWM channels
    - I2C interface (default address 0x40)
    - 12-bit PWM resolution (4096 steps)
    - Internal oscillator (25 MHz)
    - Supports servo control (20ms period, 1-2ms pulse width)
    """
    
    # PCA9685 Register definitions
    MODE1 = 0x00
    MODE2 = 0x01
    SUBADR1 = 0x02
    SUBADR2 = 0x03
    SUBADR3 = 0x04
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    ALL_LED_ON_L = 0xFA
    ALL_LED_ON_H = 0xFB
    ALL_LED_OFF_L = 0xFC
    ALL_LED_OFF_H = 0xFD
    
    # Mode1 register bits
    RESTART = 0x80
    EXTCLK = 0x40
    AI = 0x20
    SLEEP = 0x10
    SUB1 = 0x08
    SUB2 = 0x04
    SUB3 = 0x02
    ALLCALL = 0x01
    
    # Mode2 register bits
    INVRT = 0x10
    OCH = 0x08
    OUTDRV = 0x04
    OUTNE1 = 0x02
    OUTNE0 = 0x01

    def __init__(self, i2c_address=0x40, i2c_bus=1, channel=0, frequency=50, logger=None):
        """
        Initialize PCA9685 PWM Driver
        
        Args:
            i2c_address: I2C address of PCA9685 (default: 0x40)
            i2c_bus: I2C bus number (default: 1 for Pi)
            channel: PWM channel to use (0-15)
            frequency: PWM frequency in Hz (default: 50Hz for servos)
            logger: Optional ROS logger for debugging
        """
        if not SMBUS_AVAILABLE:
            raise ImportError("smbus2 library not found. Install with: pip install smbus2")
        
        self.i2c_address = i2c_address
        self.i2c_bus = i2c_bus
        self.channel = channel
        self.frequency = frequency
        self.logger = logger
        
        # Connect to I2C bus
        try:
            self.bus = smbus2.SMBus(i2c_bus)
            # Test connection by attempting to read a register
            time.sleep(0.05)  # 50ms initial delay for stabilization
            test_read = self.bus.read_byte_data(i2c_address, self.MODE1)
            if self.logger:
                self.logger.info(f"PCA9685 connection test successful, MODE1=0x{test_read:02x}")
            else:
                print(f"[INFO] PCA9685 connection test successful, MODE1=0x{test_read:02x}")
        except Exception as e:
            error_msg = f"Failed to initialize I2C bus {i2c_bus} or connect to PCA9685 at 0x{i2c_address:02x}: {e}"
            if self.logger:
                self.logger.error(error_msg)
                self.logger.error("Possible causes: 1) Wiring issues 2) Power supply problems 3) I2C timing issues 4) Address conflicts")
            else:
                print(f"[ERROR] {error_msg}")
                print("[ERROR] Possible causes: 1) Wiring issues 2) Power supply problems 3) I2C timing issues 4) Address conflicts")
            raise
        
        # Initialize PCA9685
        self._initialize()
        
        # Set PWM frequency
        self.set_frequency(frequency)
        
        if self.logger:
            self.logger.info(f"PCA9685 PWM driver initialized at I2C {i2c_address:02x}, channel {channel}, {frequency}Hz")
        else:
            print(f"[INFO] PCA9685 PWM driver initialized at I2C {i2c_address:02x}, channel {channel}, {frequency}Hz")

    def _write_register(self, reg, value):
        """Write a value to a PCA9685 register"""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                self.bus.write_byte_data(self.i2c_address, reg, value)
                time.sleep(0.001)  # 1ms delay after write
                return  # Success
            except Exception as e:
                error_msg = f"Failed to write to register 0x{reg:02x} (attempt {attempt + 1}/{max_retries}): {e}"
                if self.logger:
                    self.logger.warn(error_msg)
                else:
                    print(f"[WARN] {error_msg}")
                
                if attempt < max_retries - 1:
                    time.sleep(0.01)  # 10ms delay before retry
                else:
                    # Final attempt failed
                    if self.logger:
                        self.logger.error(f"All attempts failed for register 0x{reg:02x}")
                    else:
                        print(f"[ERROR] All attempts failed for register 0x{reg:02x}")
                    raise

    def _read_register(self, reg):
        """Read a value from a PCA9685 register"""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                # Add a small delay before reading
                time.sleep(0.001)  # 1ms delay
                value = self.bus.read_byte_data(self.i2c_address, reg)
                return value
            except Exception as e:
                error_msg = f"Failed to read from register 0x{reg:02x} (attempt {attempt + 1}/{max_retries}): {e}"
                if self.logger:
                    self.logger.warn(error_msg)
                else:
                    print(f"[WARN] {error_msg}")
                
                if attempt < max_retries - 1:
                    time.sleep(0.01)  # 10ms delay before retry
                else:
                    # Final attempt failed
                    if self.logger:
                        self.logger.error(f"All attempts failed for register 0x{reg:02x}")
                    else:
                        print(f"[ERROR] All attempts failed for register 0x{reg:02x}")
                    raise

    def _initialize(self):
        """Initialize PCA9685 chip"""
        # Add initial delay for chip stabilization
        time.sleep(0.01)  # 10ms delay
        
        # Reset to default
        self._write_register(self.MODE1, 0x00)
        time.sleep(0.005)  # Wait after reset
        
        self._write_register(self.MODE2, self.OUTDRV)
        time.sleep(0.005)  # Wait after mode2 setup
        
        # Configure MODE1 for auto-increment
        self._write_register(self.MODE1, self.AI)
        
        time.sleep(0.005)  # Wait for oscillator

    def set_frequency(self, frequency):
        """
        Set PWM frequency for all channels
        
        Args:
            frequency: PWM frequency in Hz (24-1526 Hz range)
        """
        if frequency < 24 or frequency > 1526:
            raise ValueError("Frequency must be between 24-1526 Hz")
        
        self.frequency = frequency
        
        # Calculate prescaler value
        # PCA9685 has 25MHz internal oscillator
        # PWM frequency = 25MHz / (4096 * (prescaler + 1))
        prescaler = round(25000000.0 / (4096.0 * frequency)) - 1
        prescaler = max(3, min(255, prescaler))  # Clamp to valid range
        
        # To set prescaler, we need to put chip to sleep
        old_mode = self._read_register(self.MODE1)
        new_mode = (old_mode & 0x7F) | self.SLEEP  # Sleep mode
        self._write_register(self.MODE1, new_mode)
        
        # Set prescaler
        self._write_register(self.PRESCALE, prescaler)
        
        # Restore previous mode
        self._write_register(self.MODE1, old_mode)
        
        time.sleep(0.005)
        
        # Restart if necessary
        if old_mode & self.RESTART:
            self._write_register(self.MODE1, old_mode | self.RESTART)
        
        actual_freq = 25000000.0 / (4096.0 * (prescaler + 1))
        if self.logger:
            self.logger.info(f"PCA9685 frequency set to {actual_freq:.1f} Hz (prescaler={prescaler})")
        else:
            print(f"[INFO] PCA9685 frequency set to {actual_freq:.1f} Hz (prescaler={prescaler})")

    def set_pwm_raw(self, channel, on_time, off_time):
        """
        Set PWM for a channel using raw 12-bit values
        
        Args:
            channel: PWM channel (0-15)
            on_time: When to turn on (0-4095)
            off_time: When to turn off (0-4095)
        """
        if channel < 0 or channel > 15:
            raise ValueError("Channel must be 0-15")
        
        if on_time < 0 or on_time > 4095 or off_time < 0 or off_time > 4095:
            raise ValueError("PWM values must be 0-4095")
        
        # Calculate register addresses for this channel
        base_reg = self.LED0_ON_L + 4 * channel
        
        # Write the 4 registers for this channel
        self._write_register(base_reg, on_time & 0xFF)
        self._write_register(base_reg + 1, on_time >> 8)
        self._write_register(base_reg + 2, off_time & 0xFF)
        self._write_register(base_reg + 3, off_time >> 8)

    def set_pwm_percent(self, channel, duty_percent):
        """
        Set PWM duty cycle as percentage
        
        Args:
            channel: PWM channel (0-15)
            duty_percent: Duty cycle percentage (0.0-100.0)
        """
        if duty_percent < 0.0 or duty_percent > 100.0:
            raise ValueError("Duty cycle must be 0.0-100.0%")
        
        if duty_percent == 0.0:
            # Fully off
            self.set_pwm_raw(channel, 0, 4096)
        elif duty_percent >= 100.0:
            # Fully on
            self.set_pwm_raw(channel, 4096, 0)
        else:
            # Normal PWM
            off_time = int(4096 * duty_percent / 100.0)
            self.set_pwm_raw(channel, 0, off_time)

    def set_pwm(self, period_ns, duty_ns):
        """
        Set PWM using nanosecond timing (compatible with other drivers)
        
        Args:
            period_ns: PWM period in nanoseconds
            duty_ns: PWM duty cycle in nanoseconds
        """
        if self.logger:
            self.logger.info(f"[DEBUG] PCA9685 set_pwm called with period_ns={period_ns}, duty_ns={duty_ns}")
        else:
            print(f"[DEBUG] PCA9685 set_pwm called with period_ns={period_ns}, duty_ns={duty_ns}")
        
        # Calculate duty cycle percentage
        if period_ns <= 0:
            duty_percent = 0.0
        else:
            duty_percent = (duty_ns / period_ns) * 100.0
        
        # Set the PWM
        self.set_pwm_percent(self.channel, duty_percent)
        
        if self.logger:
            self.logger.info(f"[DEBUG] PCA9685 channel {self.channel} set to {duty_percent:.2f}% duty cycle")
        else:
            print(f"[DEBUG] PCA9685 channel {self.channel} set to {duty_percent:.2f}% duty cycle")

    def set_servo_angle(self, channel, angle_degrees):
        """
        Set servo to specific angle (0-180 degrees)
        
        Args:
            channel: PWM channel (0-15)
            angle_degrees: Servo angle (0-180 degrees)
        """
        if angle_degrees < 0 or angle_degrees > 180:
            raise ValueError("Servo angle must be 0-180 degrees")
        
        # Map angle to pulse width (1-2ms for 0-180 degrees)
        min_pulse_ms = 1.0  # 1ms for 0 degrees
        max_pulse_ms = 2.0  # 2ms for 180 degrees
        pulse_ms = min_pulse_ms + (angle_degrees / 180.0) * (max_pulse_ms - min_pulse_ms)
        
        # Convert to duty cycle percentage (assuming 20ms period)
        period_ms = 1000.0 / self.frequency
        duty_percent = (pulse_ms / period_ms) * 100.0
        
        self.set_pwm_percent(channel, duty_percent)

    def set_servo_pulse_width(self, channel, pulse_width_ms):
        """
        Set servo using pulse width in milliseconds
        
        Args:
            channel: PWM channel (0-15)
            pulse_width_ms: Pulse width in milliseconds (typically 1.0-2.0)
        """
        # Convert to duty cycle percentage
        period_ms = 1000.0 / self.frequency
        duty_percent = (pulse_width_ms / period_ms) * 100.0
        
        self.set_pwm_percent(channel, duty_percent)

    def turn_off_channel(self, channel):
        """Turn off a specific PWM channel"""
        self.set_pwm_raw(channel, 0, 4096)

    def turn_on_channel(self, channel):
        """Turn on a specific PWM channel (100% duty)"""
        self.set_pwm_raw(channel, 4096, 0)

    def turn_off_all(self):
        """Turn off all PWM channels"""
        self._write_register(self.ALL_LED_ON_L, 0)
        self._write_register(self.ALL_LED_ON_H, 0)
        self._write_register(self.ALL_LED_OFF_L, 0)
        self._write_register(self.ALL_LED_OFF_H, 0x10)  # Turn off bit

    def cleanup(self):
        """Clean up resources"""
        try:
            # Turn off the current channel
            self.turn_off_channel(self.channel)
            
            if self.logger:
                self.logger.info(f"PCA9685 channel {self.channel} turned off")
            else:
                print(f"[INFO] PCA9685 channel {self.channel} turned off")
                
            # Close I2C bus
            if hasattr(self, 'bus'):
                self.bus.close()
                
        except Exception as e:
            error_msg = f"Error during PCA9685 cleanup: {e}"
            if self.logger:
                self.logger.error(error_msg)
            else:
                print(f"[ERROR] {error_msg}")

    def get_info(self):
        """Get driver information"""
        return {
            'driver_type': 'PCA9685',
            'i2c_address': f"0x{self.i2c_address:02x}",
            'i2c_bus': self.i2c_bus,
            'channel': self.channel,
            'frequency': self.frequency,
            'resolution': '12-bit (4096 steps)'
        }
