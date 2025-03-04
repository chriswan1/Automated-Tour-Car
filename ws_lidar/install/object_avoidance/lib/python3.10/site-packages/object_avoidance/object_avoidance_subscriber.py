import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import smbus

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

class Adc:
    def __init__(self):
        # Get I2C bus
        self.bus = smbus.SMBus(1)
        
        # I2C address of the device
        self.ADDRESS            = 0x48
        
        # PCF8591 Command
        self.PCF8591_CMD                        =0x40  #Command
        
        # ADS7830 Command 
        self.ADS7830_CMD                        = 0x84 # Single-Ended Inputs
        
        for i in range(3):
            aa=self.bus.read_byte_data(self.ADDRESS,0xf4)
            if aa < 150:
                self.Index="PCF8591"
            else:
                self.Index="ADS7830" 
    def analogReadPCF8591(self,chn):#PCF8591 read ADC value,chn:0,1,2,3
        value=[0,0,0,0,0,0,0,0,0]
        for i in range(9):
            value[i] = self.bus.read_byte_data(self.ADDRESS,self.PCF8591_CMD+chn)
        value=sorted(value)
        return value[4]   
        
    def analogWritePCF8591(self,value):#PCF8591 write DAC value
        self.bus.write_byte_data(self.ADDRESS,cmd,value)
        
    def recvPCF8591(self,channel):#PCF8591 write DAC value
        while(1):
            value1 = self.analogReadPCF8591(channel)   #read the ADC value of channel 0,1,2,
            value2 = self.analogReadPCF8591(channel)
            if value1==value2:
                break;
        voltage = value1 / 256.0 * 3.3  #calculate the voltage value
        voltage = round(voltage,2)
        return voltage
    def recvADS7830(self,channel):
        """Select the Command data from the given provided value above"""
        COMMAND_SET = self.ADS7830_CMD | ((((channel<<2)|(channel>>1))&0x07)<<4)
        self.bus.write_byte(self.ADDRESS,COMMAND_SET)
        while(1):
            value1 = self.bus.read_byte(self.ADDRESS)
            value2 = self.bus.read_byte(self.ADDRESS)
            if value1==value2:
                break;
        voltage = value1 / 255.0 * 3.3  #calculate the voltage value
        voltage = round(voltage,2)
        return voltage
        
    def recvADC(self,channel):
        if self.Index=="PCF8591":
            data=self.recvPCF8591(channel)
        elif self.Index=="ADS7830":
            data=self.recvADS7830(channel)
        return data
    def i2cClose(self):
        self.bus.close()

class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.time_proportion = 3     #Depend on your own car,If you want to get the best out of the rotation mode, change the value by experimenting.
        self.adc = Adc()
    def duty_range(self,duty1,duty2,duty3,duty4):
        if duty1>4095:
            duty1=4095
        elif duty1<-4095:
            duty1=-4095        
        
        if duty2>4095:
            duty2=4095
        elif duty2<-4095:
            duty2=-4095
            
        if duty3>4095:
            duty3=4095
        elif duty3<-4095:
            duty3=-4095
            
        if duty4>4095:
            duty4=4095
        elif duty4<-4095:
            duty4=-4095
        return duty1,duty2,duty3,duty4
        
    def left_Upper_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(0,0)
            self.pwm.setMotorPwm(1,duty)
        elif duty<0:
            self.pwm.setMotorPwm(1,0)
            self.pwm.setMotorPwm(0,abs(duty))
        else:
            self.pwm.setMotorPwm(0,4095)
            self.pwm.setMotorPwm(1,4095)
    def left_Lower_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(3,0)
            self.pwm.setMotorPwm(2,duty)
        elif duty<0:
            self.pwm.setMotorPwm(2,0)
            self.pwm.setMotorPwm(3,abs(duty))
        else:
            self.pwm.setMotorPwm(2,4095)
            self.pwm.setMotorPwm(3,4095)
    def right_Upper_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(6,0)
            self.pwm.setMotorPwm(7,duty)
        elif duty<0:
            self.pwm.setMotorPwm(7,0)
            self.pwm.setMotorPwm(6,abs(duty))
        else:
            self.pwm.setMotorPwm(6,4095)
            self.pwm.setMotorPwm(7,4095)
    def right_Lower_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(4,0)
            self.pwm.setMotorPwm(5,duty)
        elif duty<0:
            self.pwm.setMotorPwm(5,0)
            self.pwm.setMotorPwm(4,abs(duty))
        else:
            self.pwm.setMotorPwm(4,4095)
            self.pwm.setMotorPwm(5,4095)
            
 
    def setMotorModel(self,duty1,duty2,duty3,duty4):
        duty1,duty2,duty3,duty4=self.duty_range(duty1,duty2,duty3,duty4)
        self.left_Upper_Wheel(duty1)
        self.left_Lower_Wheel(duty2)
        self.right_Upper_Wheel(duty3)
        self.right_Lower_Wheel(duty4)
            
    def Rotate(self,n):
        angle = n
        bat_compensate =7.5/(self.adc.recvADC(2)*3)
        while True:
            W = 2000

            VY = int(2000 * math.cos(math.radians(angle)))
            VX = -int(2000 * math.sin(math.radians(angle)))

            FR = VY - VX + W
            FL = VY + VX - W
            BL = VY - VX - W
            BR = VY + VX + W

            PWM.setMotorModel(FL, BL, FR, BR)
            print("rotating")
            time.sleep(5*self.time_proportion*bat_compensate/1000)
            angle -= 5

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

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.motor = Motor()
        self.servo = Servo()
        self.servo.set_servo_pwm('0', 25)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info("Listening for /cmd_vel messages...")

    def listener_callback(self, msg):
        linear_x = msg.linear.x  # Forward/backward speed
        angular_z = msg.angular.z  # Steering angle request

        if angular_z > 0:  # Left Turn
            steering_angle = 35
        elif angular_z < 0:  # Right Turn
            steering_angle = 15
        else:
            steering_angle = 25  # Keep straight

        self.servo.set_servo_pwm('0', steering_angle)
        self.get_logger().info(f"Steering: {steering_angle}°")

        if linear_x > 0:
            # Move forward
            self.motor.setMotorModel(0, 2000, 0, 2000)
        elif linear_x < 0:
            # Move backward
            self.motor.setMotorModel(0, -2000, 0, -2000)
        else:
            # Stop
            self.motor.setMotorModel(0, 0, 0, 0)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
