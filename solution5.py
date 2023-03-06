import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy

from transforms3d import euler
from enum import Enum, auto


class State(Enum):
    TO_THE_FIRST_WALL = auto()
    ROTATING_LEFT = auto()
    TO_THE_SECOND_WALL = auto()
    STOP = auto()
    CHECK_ROTATION = auto()


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
        self.state = State.ROTATING_LEFT
        self.diff = 0
        self.goalCords = (0.5, 0.5)
        self.msg_odom = None
        self.open_paths = []
        self.path_history = []
        self.counter = 0

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
                    if last_dir[0]:
                        x, y = self.goalCords
                        self.goalCords = (x, y + 1)
                        self.driveToCords(self.goalCords)
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
        self.calculatePath(msg)
        

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        angles = euler.quat2euler([orientation.x, orientation.y, orientation.z, orientation.w])
    
        #print("Aktuelle Ausrichtung: " + angles)
        #print("Aktuelle Position: " + position)

        self.msg_odom = [position, angles]
        if self.counter == True:
            self.driveToCords((0.5, 1.5))
            self.counter = False
        pass


    def roundFloat(self, float_number):
        ntr = float_number
        i, d = divmod(ntr, 1)
        d = round(d*10) 
        ntr = i + (d/10)
        return ntr
    
    #Turning to direction the bot will drive too
    #Setting velocity to 20 while on path to given Cords    
    def driveToCords(self, coords):
        x,y = coords

        position = self.msg_odom[0]
        angles = self.msg_odom[1]

        origin_x = position.x
        origin_y = position.y

        goal_xy = numpy.array((x, y))
        origin_xy = numpy.array((origin_x, origin_y))
        real_xy = numpy.array((self.roundFloat(origin_x), self.roundFloat(origin_y)))
    
        dist_OG = numpy.linalg.norm(origin_xy-goal_xy)
        dist_RG = numpy.linalg.norm(real_xy-goal_xy)
        dist_OR = numpy.linalg.norm(origin_xy-real_xy)

        print(dist_OG, dist_OR, dist_RG)
    
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