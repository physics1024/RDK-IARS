#ros头文件
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

#舵机头文件
import sys
import signal
import os
import time

# 导入python串口库
import serial
import serial.tools.list_ports



# 打开串口
def serial_open(n=1):
    global ser
    ser = serial.Serial(f"/dev/ttyS{n}", 9600, timeout=1)
    if ser.isOpen():
        print( "open success")
        return 0
    else:
        print("open failed")
        return 255

# 舵机转动函数
def UARTServo(servonum, angle):
    servonum = 64 + servonum
    date1 = int(angle/100 + 48)
    date2 = int((angle%100)/10 + 48)
    date3 = int(angle%10 + 48)
    cmd=bytearray([36,servonum,date1,date2,date3,35])
    print(cmd)
    ser.write(cmd)
    time.sleep(0.05)


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.subscription = self.create_subscription(
            Int32,
            '/cmd_servo',
            self.cmd_motor_callback,
            10)

        # 初始角度：水平为0度，垂直为0度(对应过来是90,90)
        self.horizontal_angle = 90       # 水平角度
        self.vertical_angle = 90         # 垂直角度

        # 舵机角度限制
        self.max_angle_x = 180
        self.min_angle_x = 0
        
        self.max_angle_y = 160
        self.min_angle_y = 0
        

        # 打开串口
        serial_open(1)

        self.get_logger().info('Servo controller initialized.')


    def cmd_motor_callback(self, msg):
        cmd = msg.data
        if cmd == 1:#云台水平向左转动
            self.horizontal_angle = max(self.min_angle_x, self.horizontal_angle - 30)
        elif cmd == 2:#云台水平向右转动
            self.horizontal_angle = min(self.max_angle_x, self.horizontal_angle + 30)
        elif cmd == 3:#云台垂直向下转动
            self.vertical_angle = min(self.max_angle_y, self.vertical_angle + 30)
        elif cmd == 4:#云台垂直向上转动
            self.vertical_angle = max(self.min_angle_y, self.vertical_angle - 30)
        elif cmd == 5:#水平竖直全回正
            self.horizontal_angle = 90
            self.vertical_angle = 90
        else:
            self.get_logger().warn(f"Invalid command: {cmd}")
            return

        self.get_logger().info(
            f'Updated servo angles: H={self.horizontal_angle}°, V={self.vertical_angle}°'
        )

        #云台水平转动
        UARTServo(3, self.horizontal_angle)
        #云台垂直转动
        UARTServo(2, self.vertical_angle)

   

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()