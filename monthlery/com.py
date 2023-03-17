#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from covaps.msg import Order
#import RPi.GPIO as GPIO
from time import sleep

PWM_FREQ_DIR = 1000 #in hz
PWM_FREQ_SPEED = 1000 #in hz

PWM_DIR_MIN = 0
PWM_DIR_0 = 50
PWM_DIR_MAX = 100

PWM_SPEED_MIN = 0
PWM_SPEED_0 = 50
PWM_SPEED_MAX = 100


class ComNode(Node):
    def __init__(self):
        super().__init__("com_node")
        self.get_logger().info("com node started")
        self.pathMicro = "/dev/ttyACM0"
        self.baudrate = 115200

        self.stm = serial.Serial()
        self.stm.port = self.pathMicro
        self.stm.baudrate = self.baudrate

        self.dutyCicleDir = PWM_DIR_0
        self.dutyCicleSpeed = PWM_SPEED_0
        self.pinSpeed = 12
        self.pinDirection = 13
        #GPIO.setwarnings(False)	
        #GPIO.setmode(GPIO.BOARD)
        #pwmSpeed = GPIO.PWM(self.pinSpeed, PWM_FREQ_DIR)
        #pwmDirection = GPIO.PWM(self.pinDirection, PWM_FREQ_SPEED)
        #pwmSpeed.start(PWM_SPEED_0)
        #pwmDirection.start(PWM_DIR_0)



        self.cmd_recv = self.create_subscription(Order, "/monthlery/cmd_car", self.rcv_order, 10)

    def rcv_order(self, order: Order):
        if (order.type == "speed"):
            self.changeSpeed(order.val)
        elif (order.type == "angular"):
            self.changeAngular(order.val)
        else :
            self.get_logger().warning("invalide type")


    def changeSpeed(self,  speedValue):
        self.get_logger().info("change speed")
        #self.changePWMSpeed(speedValue)

    def changeAngular(self, angularPos):
        self.get_logger().info("change angular position")
        #self.changePWMDir(angularPos)

    
    def changePWMDir(self, angleValue):
        alpha = 1
        beta = 1
        dutyExpected = alpha*angleValue + beta
        if ((dutyExpected >= PWM_DIR_MIN) and (dutyExpected <= PWM_DIR_MAX)):
            self.pwmDirection.ChangeDutyCycle(dutyExpected)
            self.get_logger().info("pwm chnaged")
        else :
            self.get_logger().warning("direction, pwm limit reached")
        


    def changePWMSpeed(self, speedValue):
        alpha = 1
        beta = 1
        dutyExpected = alpha*speedValue + beta
        if ((dutyExpected >= PWM_SPEED_MIN) and (dutyExpected <= PWM_SPEED_MAX)):
            self.pwmDirection.ChangeDutyCycle(dutyExpected)
            self.get_logger().info("pwm changed")
        else :
            self.get_logger().warning("speed, pwm limit reached")



def main(args=None):
    rclpy.init(args=args)
    node = ComNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()