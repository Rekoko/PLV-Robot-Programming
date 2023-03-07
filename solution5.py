import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy
import math
from transforms3d import euler
from enum import Enum, auto


class State(Enum):
    DRIVING = auto()
    ROTATING = auto()
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
        
        self.scan_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,  # function to run upon message arrival
                qos_profile_sensor_data)  # allows packet loss
        

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.state = State.STOP
        self.diff = 0
        self.goalCords = (0.5, 0.5)
        self.msg_odom = None
        self.open_paths = []
        self.path_history = []
        self.counter = True
        self.rotate = True
        self.angle = 0
        self.angle_adj = 0

    # Checks if there is a path available along the x-/y-Axes
    def getPaths(self, msg):
        results = [False for i in range(4)]
        if msg.ranges[0] > 1: # up path
            results[0] = True
        if msg.ranges[-90] > 1: # right path
            results[1] = True
        if msg.ranges[180] > 1: # down path
            results[2] = True
        if msg.ranges[90] > 1: # left path
            results[3] = True
        return results

    # Decides where to go next while saving the path where it went and where it can go
    def calculatePath(self, msg):
        if self.msg_odom:
            if (self.goalCords[0] + 0.1) > self.msg_odom[0].x > (self.goalCords[0] - 0.1) and\
            (self.goalCords[1] + 0.1) > self.msg_odom[0].y > (self.goalCords[1] - 0.1):
                current_history = self.getPaths(msg)
                print(current_history)
                self.open_paths.append(current_history)
                while True:
                    last_dir = self.open_paths.pop()
                    x, y = self.goalCords
                    if last_dir[0]:
                        self.goalCords = (x, y + 1)
                        self.driveToCords(self.goalCords)
                        last_dir[0] = False
                        break
                    elif last_dir[1]:
                        self.goalCords = (x + 1, y)
                        self.driveToCords(self.goalCords)
                        last_dir[1] = False
                    elif last_dir[2]:
                        self.goalCords = (x, y - 1)
                        self.driveToCords(self.goalCords)
                        last_dir[2] = False
                    elif last_dir[3]:
                        self.goalCords = (x - 1, y)
                        self.driveToCords(self.goalCords)
                        last_dir[3] = False
                    else:
                        pass
                    self.open_paths.append(last_dir)
        return None


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
        # print()
        # print('Distances:')
        # print('⬆️ :', msg.ranges[0])
        # print('⬇️ :', msg.ranges[180])
        # print('⬅️ :', msg.ranges[90])
        #print('➡️ :', msg.ranges[-90])
        #self.calculatePath(msg)
        

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        euler_angles = euler.quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        angles = [euler_angles[0] * (180/math.pi), euler_angles[1] * (180/math.pi), euler_angles[2] * (180/math.pi)]

        # print("Aktuelle Ausrichtung: ", angles)
        # print("Aktuelle Position: ", position)

        self.msg_odom = [position, angles]
        ur_angle = angles[2]
        if ur_angle < 0:
            ur_angle = ur_angle + 360

        # print(ur_angle)
        # print(self.angle)
        if self.counter == True:
            self.setRotation((1.5, 0.5, "RIGHT"))
            self.counter = False
            self.state = State.ROTATING

        if self.state == State.ROTATING:
            self.vel(0,-5)
            if ur_angle+self.angle_adj+0.1 > self.angle > ur_angle-self.angle_adj-0.1:
                self.state = State.DRIVING
        elif self.state == State.DRIVING:
            self.vel(20,0)
        elif self.state == State.STOP:
            self.vel(0,0)

        pass

    
    #Turning to direction the bot will drive too
    #Setting velocity to 20 while on path to given Cords    
    def setRotation(self, coords):
        x,y,direction = coords
        self.rotate = True    
        if self.rotate == True:
            position = self.msg_odom[0]

            origin_x, origin_y = position.x, position.y
            goal_xy = numpy.array((x, y))
            origin_xy = numpy.array((origin_x, origin_y))


            vektor_OG = [x- origin_x, y - origin_y]
            dist_OG = numpy.linalg.norm(origin_xy-goal_xy)


            self.angle_adj = numpy.arccos(vektor_OG[0] / dist_OG)

            if direction == "UP":
                self.angle = 90 + self.angle_adj
            if direction == "DOWN":
                self.angle = 270 - self.angle_adj
            if direction == "LEFT":
                self.angle = 180 + self.angle_adj
            if direction == "RIGHT":  
                self.angle = 360 + self.angle_adj
            self.rotate = False

        return None
    
    def speedRegulation(self):
        return None
    
    def checkGoald(self):
        return None
    

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