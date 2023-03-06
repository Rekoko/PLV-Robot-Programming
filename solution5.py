import rclpy  # ROS client library
import os
import subprocess
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from enum import Enum, auto
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class State(Enum):
    TO_THE_FIRST_WALL = auto()
    ROTATING = auto()
    TO_THE_SECOND_WALL = auto()
    STOP = auto()

class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,      # message type
                'cmd_vel',  # topic name
                1)          # history depth

        self.scan_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,  # function to run upon message arrival
                qos_profile_sensor_data)  # allows packet loss

        self.st = State.TO_THE_FIRST_WALL
        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
    


    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
        print()
        print('⬆️ :', msg.ranges[0])
        print('⬇️ :', msg.ranges[180])
        print('⬅️ :', msg.ranges[90])
        print('➡️ :', msg.ranges[-90])
        


        if self.st == State.TO_THE_FIRST_WALL:
            self.vel(20)
            if msg.ranges[0] <= 0.25:
                self.st = State.ROTATING
        elif self.st == State.ROTATING:
            self.vel(0, 20)
            if msg.ranges[-90] <= 0.17:
                self.st = State.TO_THE_SECOND_WALL
        elif self.st == State.TO_THE_SECOND_WALL:
            self.vel(20)
            if msg.ranges[0] < 0.2:
                self.vel(0)
                self.st = State.STOP
        elif self.st == State.STOP:
            self.vel(0)




def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
