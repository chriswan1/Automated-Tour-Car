import time
import smbus
import math

class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  def __init__(self, address=0x40, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    self.write(self.__MODE1, 0x00)
    
  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
      
  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    return result
    
  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    prescale = math.floor(prescaleval + 0.5)


    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H+4*channel, on >> 8)
    self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H+4*channel, off >> 8)
  def setMotorPwm(self,channel,duty):
    self.setPWM(channel,0,duty)
  def setServoPulse(self, channel, pulse):
    "Sets the Servo Pulse,The PWM frequency must be 50HZ"
    pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
    self.setPWM(channel, 0, int(pulse))

class Servo:
    def __init__(self):
        self.pwm_frequency = 50
        self.initial_pulse = 1500
        self.pwm_channel_map = {
            '0': 8,
            '1': 9,
            '2': 10,
            '3': 11,
            '4': 12,
            '5': 13,
            '6': 14,
            '7': 15
        }
        self.pwm_servo = PCA9685(0x40, debug=True)
        self.pwm_servo.setPWMFreq(self.pwm_frequency)
        for channel in self.pwm_channel_map.values():
            self.pwm_servo.setServoPulse(channel, self.initial_pulse)

    def set_servo_pwm(self, channel: str, angle: int, error: int = 10) -> None:
        angle = int(angle)
        if channel not in self.pwm_channel_map:
            raise ValueError(f"Invalid channel: {channel}. Valid channels are {list(self.pwm_channel_map.keys())}.")
        # For channel '0', we assume the servo orientation is reversed.
        pulse = 2500 - int((angle + error) / 0.09) if channel == '0' else 500 + int((angle + error) / 0.09)
        self.pwm_servo.setServoPulse(self.pwm_channel_map[channel], pulse)

def main():
    print("Testing axle turning using servo on channel 0")
    pwm_servo = Servo()
    
    # Define angles based on your specifications:
    # 45° represents straight ahead (0° turning), 46-65 for left, 0-44 for right.
    center_angle = 25     # Straight
    left_max_angle = 65   # Maximum left tilt
    right_max_angle = -15   # Maximum right tilt

    # Delay between each step (in seconds)
    step_delay = 0.1

    try:
        # while True:
        # Set to center (straight)
        print("Setting to center (straight ahead)")
        pwm_servo.set_servo_pwm('0', center_angle)
        time.sleep(1)
        
        # Sweep left from center (45) to maximum left (65)
        print("Sweeping left...")
        for angle in range(center_angle, left_max_angle + 1):
            pwm_servo.set_servo_pwm('0', angle)
            print(f"Left turn: servo angle = {angle}")
            time.sleep(step_delay)
        time.sleep(1)
        
        # Return to center from left
        print("Returning from left to center...")
        for angle in range(left_max_angle, center_angle - 1, -1):
            pwm_servo.set_servo_pwm('0', angle)
            print(f"Returning: servo angle = {angle}")
            time.sleep(step_delay)
        time.sleep(1)
        
        # Sweep right from center (45) to maximum right (0)
        print("Sweeping right...")
        for angle in range(center_angle, right_max_angle - 1, -1):
            pwm_servo.set_servo_pwm('0', angle)
            print(f"Right turn: servo angle = {angle}")
            time.sleep(step_delay)
        time.sleep(1)
        
        # Return to center from right
        print("Returning from right to center...")
        for angle in range(right_max_angle, center_angle + 1):
            pwm_servo.set_servo_pwm('0', angle)
            print(f"Returning: servo angle = {angle}")
            time.sleep(step_delay)
        time.sleep(1)
    except KeyboardInterrupt:
        print("\nEnd of program")


if __name__ == '__main__':
    main()