import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy
import math
from transforms3d import euler
from enum import Enum, auto

import networkx as nx


class State(Enum):
    DRIVING = auto()
    ROTATING = auto()
    STOP = auto()


# If we have seen a Crossway it will become a Crossway object, otherwise it will be None
# dict or do we need connections to "up" "down" "left" "right"? - None since these are the edges
# Do we need any object at all? no? yes? We dont know what direction the edge goes
# We can do weighted edges and save the change of the coordinates in the edge to then add to the goal coordinate upon goin onto that edge
# We never need another class
# Weights as a tupel for the goal  change?

class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
            Twist,      # message type
            'cmd_vel',  # topic name
            1)          # history depth
        self.scan_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_image_callback,
            qos_profile_sensor_data)
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

        # Graph variables
        self.graph = nx.Graph()
        self.path_history = []  # What direction is back?
        self.visited_nodes = []  # Where have we been
        self.node_number = 0  # Naming nodes different names
        self.current_node = 0  # Current position
        self.direction = "UP"

        # Matrix varibales
        self.matrix = [[{"DIRECTIONS": [], "VISITED": False} for i in range(10)] for i in range(10)]
        self.current_cords = (0, 0)

        self.counter = True

        self.angle = 0
        self.angle_adj = 0

    # Checks if there is a path available along the x-/y-Axes
    def getPaths(self, msg, origin):
        results = []
        if msg.ranges[0] > 1:  # up path
            results.append("UP")
        if msg.ranges[-90] > 1:  # right path
            results.append("RIGHT")
        if msg.ranges[180] > 1:  # down path
            results.append("DOWN")
        if msg.ranges[90] > 1:  # left path
            results.append("LEFT")

        if self.direction == "LEFT":
            for i in range(3):
                results = self.shiftOnce(results)
        if self.direction == "DOWN":
            for i in range(2):
                results = self.shiftOnce(results)
        if self.direction == "RIGHT":
            for i in range(1):
                results = self.shiftOnce(results)

        if origin in results:
            results.remove(origin)
        return results

    def shiftOnce(self, dir_list):
        single_shift_dic = {"UP": "RIGHT",
                            "RIGHT": "DOWN", "DOWN": "LEFT", "LEFT": "UP"}
        new_list = []
        for i in dir_list:
            new_list.append(single_shift_dic[i])
        return new_list

    # Sets a new goal based on the given direction
    def setGoal(self, dir):
        x, y = self.goalCords
        if dir == "UP":
            self.goalCords = x, y + 1
        if dir == "RIGHT":
            self.goalCords = x + 1, y
        if dir == "DOWN":
            self.goalCords = x, y - 1
        if dir == "LEFT":
            self.goalCords = x - 1, y

    def reverseDirection(self, dir):
        if dir == "UP":
            return "DOWN"
        if dir == "DOWN":
            return "UP"
        if dir == "RIGHT":
            return "LEFT"
        if dir == "LEFT":
            return "RIGHT"

    # Problem right now:
    # It cant realize when to go back since it doesnt really understand when
    # a direction is undiscovered or when it has already been visited # Still a problem? We can make a list of visited nodes? easy
    # Linked List or List
    # We could just add a variable for the path that we came from and another for possible open paths
    # Linked List seems better and easier

    # Decides where to go next while saving the path where it went and where it can go
    # !!!! UNFINISHED KEPT IN CASE IT FINISHES !!!!!

    def calculatePathGraph(self, msg):
        if self.msg_odom:
            if (self.goalCords[0] + 0.1) > self.msg_odom[0].x > (self.goalCords[0] - 0.1) and\
                    (self.goalCords[1] + 0.1) > self.msg_odom[0].y > (self.goalCords[1] - 0.1):
                print(
                    "----------------------------------------\nNEW ITERATION\n----------------------------------------")
                if self.current_node not in self.visited_nodes:  # if the node we are at is a new node

                    # Add current node to the Graph

                    if nx.is_empty(self.graph):
                        # We kinda only want to do this in the beginning
                        self.graph.add_node(self.node_number)
                        self.node_number += 1
                        print("added first node")
                        origin = None
                    else:
                        # print("________DEBUG______________")
                        # print(list(self.graph.edges))
                        # print(list(self.graph.nodes))
                        # print(self.visited_nodes)
                        # print(self.current_node)
                        # print(self.visited_nodes[-1])
                        # print("________DEBUG--END______________")
                        origin = self.reverseDirection(
                            self.graph[self.path_history[-1]][self.current_node]["weight"])
                    # Add node to the list of seen nodes
                    self.visited_nodes.append(self.current_node)

                    # When to set the path history and the visited nodes:
                    # Visited Nodes: upon reaching a new node/ Only to be called when reaching a new node which is correct
                    # Path history: upon leaving a node
                    # self.path_history.append(self.current_node)
                    # self.visited_nodes.append(self.current_node)

                    # For every direction that the turtle sees as open;
                    # we make a new node and an edge to that node from the current node
                    if msg == None:
                        print("msg is none")
                    if origin == None:
                        print("origin is none")
                    for direction in self.getPaths(msg, origin):
                        self.graph.add_node(self.node_number)
                        self.graph.add_edge(
                            self.node_number, self.current_node, weight=direction)
                        self.node_number += 1

                    # How do we find back? Make Path History
                    # How do we know where to still go? Check every path --
                    # We only have one direction in the edge since we assume that there are no loops in the maze
                    # Look for a new node at the current position
                    #   If Yes: Go to that node
                    #   If No: Go back to the node where we came from (Node History) and repeat step

                # For all connections we have to the current node,
                # Probably have to filter this one a little bit more since the destination will not always be at the second pos
                for _, dest, dir in list(self.graph.edges(self.current_node, data=True)):
                    if dest not in self.visited_nodes:
                        self.path_history.append(self.current_node)
                        self.current_node = dest
                        dir = dir["weight"]
                        print(self.goalCords)
                        self.setGoal(dir)
                        x, y = self.goalCords
                        self.setRotation((x, y, dir))
                        print(dir)
                        print(list(self.graph.nodes))
                        print(list(self.graph.edges))
                        print(self.goalCords)
                        return

                # Go backwards and set current node + destination to that node
                goal_node = self.path_history.pop()
                direc = self.reverseDirection(
                    self.graph[goal_node][self.current_node]["weight"])
                self.setGoal(direc)
                x, y = self.goalCords
                self.current_node = goal_node
                self.setRotation((x, y, direc))
                if direc == "DOWN":
                    print("DIREC: DOWN")
                print(f"going back where we came from with:\nx: %s\ny: %s\ndirec: %s" % (
                    x, y, direc))

        return None
    
    def targetIndices(self, x, y, dir):
        if dir == "UP":
            y=  y + 1
        if dir == "RIGHT":
            x = x + 1
        if dir == "DOWN":
            y = y - 1
        if dir == "LEFT":
            x = x - 1
        return x, y
    
    def debug_prints(self):
        print(f"DIRECTION: %s"%self.direction)
        print(f"GOAL: %s"%self.current_cords)
        print(f"PATH TO THE BEGINNING: %s"%self.path_history)

    def calculatePathMatrix(self, msg):
        # If a message exists
        if self.msg_odom:
            # If we are in a not driving state (Logic for this could be moved)
            if (self.current_cords[0] + 0.7) > self.msg_odom[0].x > (self.current_cords[0] + 0.3) and\
                    (self.current_cords[1] + 0.7) > self.msg_odom[0].y > (self.current_cords[1] + 0.3):
                print(
                    "----------------------------------------\nNEW ITERATION\n----------------------------------------")
                current_square = self.matrix[self.current_cords[0]][self.current_cords[1]]
                if current_square["VISITED"] == False:  # if the node we are at is a new node, gather information

                    # For every direction that the turtle sees as open;
                    # we not that in the matrix
                    # We have to keep track of the orientation to correct the falsely categorized information
                    # When we are turned it will always be "off"
                    

                    current_square["VISITED"] = True

                    # add the directions to the Matrix-Square
                    self.matrix[self.current_cords[0]][self.current_cords[1]]["DIRECTIONS"] = self.getPaths(msg, self.reverseDirection(self.direction))

                    # How do we find back? Make Path History
                    # How do we know where to still go? Check every path --
                    # We only have one direction in the edge since we assume that there are no loops in the maze
                    # Look for a new node at the current position
                    #   If Yes: Go to that node
                    #   If No: Go back to the node where we came from (Node History) and repeat step

                # For all connections we have to the current node,
                # Probably have to filter this one a little bit more since the destination will not always be at the second pos // It is 
                for direction in self.matrix[self.current_cords[0]][self.current_cords[1]]["DIRECTIONS"]:
                    dest_x, dest_y = self.targetIndices(self.current_cords[0],self.current_cords[1], direction)
                    if not self.matrix[dest_x][dest_y]["VISITED"]:
                        self.path_history.append(self.reverseDirection(direction))
                        self.current_cords = dest_x, dest_y
                        self.setRotation((dest_x,dest_y, direction))
                        self.debug_prints()
                        return
                # Dont add backwards way to path_history

                # Go backwards and set current node + destination to that node
                direction = self.path_history.pop()
                dest_x, dest_y = self.targetIndices(self.current_cords[0], self.current_cords[1], direction)
                self.current_cords = dest_x, dest_y
                self.setRotation((dest_x, dest_y, direction))
                self.debug_prints()
                print("going back where we came from")

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

    def camera_image_callback(self, msg):
        # print(msg.encoding)
        return

    def camera_info_callback(self, msg):

        return None

    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
        # print()
        # print('Distances:')
        # print('⬆️ :', msg.ranges[0])
        # print('⬇️ :', msg.ranges[180])
        # print('⬅️ :', msg.ranges[90])
        # print('➡️ :', msg.ranges[-90])
        # self.calculatePathGraph(msg)
        self.calculatePathMatrix(msg)

    def odom_callback(self, msg):
        self.rotateLogic(msg)
    
    def rotateLogic(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        euler_angles = euler.quat2euler(
            [orientation.w, orientation.x, orientation.y, orientation.z])
        angles = [euler_angles[0] * (180/math.pi), euler_angles[1]
                  * (180/math.pi), euler_angles[2] * (180/math.pi)]

        # print("Aktuelle Ausrichtung: ", angles)
        # print("Aktuelle Position: ", position)

        self.msg_odom = [position, angles]
        ur_angle = angles[2]
        if ur_angle < 0:
            ur_angle = ur_angle + 360

        # print(ur_angle)
        # print(self.angle)
        # if self.counter == True:
        #     self.setRotation((1.5, 0.5, "RIGHT"))
        #     self.counter = False
        #     self.state = State.ROTATING

        if self.state == State.ROTATING:

            if -1 < (ur_angle+self.angle_adj - self.angle) < 1:
                self.state = State.DRIVING

            # if abs(self.angle - (ur_angle+self.angel_adj)) > 181:
            #     self.vel(0, -10)
            # else:
            self.vel(0, 10)
            if ur_angle+self.angle_adj+0.1 > self.angle > ur_angle-self.angle_adj-0.1:
                self.state = State.DRIVING

        elif self.state == State.DRIVING:
            self.vel(40, 0)
        elif self.state == State.STOP:
            self.vel(0, 0)

        pass


    # Turning to direction the bot will drive too
    # Setting velocity to 20 while on path to given Cords

    def setRotation(self, coords):
        self.state = State.ROTATING
        x, y, direction = coords
        self.direction = direction
        self.rotate = True
        if self.rotate == True:
            position = self.msg_odom[0]

            origin_x, origin_y = position.x, position.y
            goal_xy = numpy.array((x, y))
            origin_xy = numpy.array((origin_x, origin_y))

            vektor_OG = [x - origin_x, y - origin_y]
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
