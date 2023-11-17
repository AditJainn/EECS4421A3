import math
from math import atan2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import cv2
import random
from scipy.spatial import KDTree
import time

numberOfNodes = 1000
map = cv2.imread('map.jpg')

# colour scheme
purple = (86,70,115)
black = (38,22,27)
darkBlue = (64,38,42)
orange = (68,149,242)
red = (70,78,166)


class point:
    def __init__(self, x, y,next = None, prev = None,radius = 1):
        self.x = x
        self.y = y
        self.next = next
        self.prev = prev
        self.radius = radius
        self.color = (127,127,0)

#=======================================================================================================================================

# USE THIS METHOD TO DETERMINE IF A GENERATED POINT IS ON AN OBSTACLE
def point_obst_overlap(map, p):
    def is_not_free(x, y):
        overlap = False
        if all(map[y, x] == [0, 0, 0]):
            overlap = True
        return overlap

    mapx, mapy = p.x, p.y

    if is_not_free(mapx, mapy):
        return True    
    return False

# USE THIS METHOD TO DETIRMINE IF A LINE BETWEEN TWO VALID POINTS CROSSES OVER AN OBSTACLE
def line_color_intersection(map, v1, v2):
    # we are going to check if the pixel at the specified coordinates is white
    def is_not_white(x, y):
        lineOverlap = False
        if all(map[y, x] == [0, 0, 0]):
            lineOverlap = True
            print('LINE COLOR INTERSECTION HAS OCCURED')
        return lineOverlap
    # first, we are going to assign the point coordinates to new variables
    x1, y1, x2, y2 = v1.x, v1.y, v2.x, v2.y
    dx, dy = abs(x2 - x1), abs(y2 - y1)
    # now, the x and y coordinates will be that of the first point
    x, y = x1, y1
    print('X COORD TO BE CHECKED = ',x)
    print('Y VALUE TO BE CHECKED = ', y)
    # here, we do some shifting by steps. If x1 > x2. For example, if x1 is greater than x2 then we
    # will take a negative step in the x axis with sx. Similar for the y axis with sy
    sx = -1 if x1 > x2 else 1
    sy = -1 if y1 > y2 else 1
    # err represents the direction we will move in along the line to get from one point to the next
    err = dx - dy
    while True:
        if x == x2 and y == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
        if is_not_white(x, y):
            return True
    print('NO LINE COLOR INTERSECTION')
    return False

#=======================================================================================================================================


#=======================================================================================================================================

# GET THE DISTANCE BETWEEN TWO POINTS
def findDistace (p1, p2):
    distance = math.sqrt((p1.x - p2.x)**2 + (p2.y - p1.y)**2)
    return distance

# DRAW LINE BETWEEN POINTS
def drawLine(v1,v2,color =(128,0,128), thickness=2):
    cv2.line(map, (v1.x,v1.y), (v2.x,v2.y), color, thickness)


def bigBrainAlgo(exploredVertexList,listOfVertix):
    newConnection = 0 # This will be two different vertexes we will be returning
    smallestDistance = float('inf')
    for exploredV in exploredVertexList:
        for unexploredVertix in listOfVertix:
            calculateDistance = findDistace(exploredV,unexploredVertix)
            if calculateDistance < smallestDistance and line_color_intersection(map, exploredV, unexploredVertix) == False:
                smallestDistance = calculateDistance
                newConnection = (exploredV,unexploredVertix)

    return newConnection

# FIND THE CLOSEST NODE TO A GRAPH
def findClosestNodeToGraph(exploredVertexList, listOfVertix):
    # Convert vertex points to a list of tuples
    unexploredPoints = [(v.x, v.y) for v in listOfVertix]

    # Create a KD-tree with unexplored vertices
    tree = KDTree(unexploredPoints)

    smallestDistance = float('inf')
    newConnection = None

    for exploredV in exploredVertexList:
        # Find the nearest neighbor to exploredV in the KD-tree
        distance, index = tree.query((exploredV.x, exploredV.y))

        #if line_color_intersection(map, exploredV, listOfVertix[index]):
        if distance < smallestDistance and line_color_intersection(map, exploredV, listOfVertix[index]) == False:

            smallestDistance = distance
            newConnection = (exploredV, listOfVertix[index])
            print('NEW CONNECTION MADE')
       
   
        # Early exit if the distance is zero
        if distance == 0:
            break
    if newConnection == None:
        newConnection = bigBrainAlgo(exploredVertexList,listOfVertix)
    return newConnection

# HERE WE WILL GENERATE RANDOM POINTS AND ADD THEM TO A LIST OF POINTS
def buildMap():
    # first need to define the list of generated points
    randomPoints = [(np.random.randint(10, map.shape[1]-10), np.random.randint(10, map.shape[0]-10)) for _ in range(numberOfNodes)]
    listOfVertix = []
    # now, populate the map
    for i in range(0,numberOfNodes):
        v = point(x = randomPoints[i][0], y = randomPoints[i][1])
        # if a point is generated on an obstacle, change its colour and do NOT add it to new list
        if point_obst_overlap(map,v):
            v.color = (0, 255, 255)
        else:
            listOfVertix.append(v)
        # print(v.x)
        cv2.circle(map, (v.x,v.y), v.radius, v.color, thickness=-1)

    return listOfVertix

#=======================================================================================================================================
def mainRead(start_x = 25, start_y = 25, finish_x = 475, finish_y = 475):
   
    # get the list of points
    listOfVertix = buildMap()

    # this is essentially our RRT list of nodes
    exploredVertexList = []

    # RRT list to return to drive_to_goal
    rrt = []
    # starting index will be the first index of the list (its always random since the list is always randomly generated)
    startVertex = point(start_x, start_y)
   
    # INSERT A POINT AT A RANDOM SPOT IN THE LIST (this will be replaced by the robot odometry position in gazebo)
    random_index = random.randint(0, len(listOfVertix))

    finishPoint = point(finish_x, finish_y)
    finishPoint.color=(255, 255, 0)
   
    cv2.circle(map, (startVertex.x,startVertex.y), 6, (0,255,0), thickness=-1)
    cv2.circle(map, (finishPoint.x,finishPoint.y), 6, (255,255,0), thickness=-1)

    listOfVertix.insert(random_index, finishPoint)
    exploredVertexList.append(startVertex)

    # iterate through the list of points (vertices) until we reach the goal vertex
    while(len(listOfVertix) > 0):

        # graphNode is the node we are searching FROM and newNode is the node we are searching FOR
        graphNode, newNode = findClosestNodeToGraph(exploredVertexList, listOfVertix)

        graphNode.next = newNode
        newNode.prev = graphNode
        # if line_color_intersection(map, graphNode, newNode) == False:
        drawLine(graphNode, newNode)
        exploredVertexList.append(newNode)
        listOfVertix.remove(newNode)
        # random.shuffle(listOfVertix)


        print('ELEMENTS IN LIST OF VERTIX: ', len(listOfVertix))
        print('GRAPH NODE: ', graphNode.x, graphNode.y)
        print('NEW NODE: ', newNode.x, newNode.y)
        print('*******NODE ADDED TO RRT*******')
       

        # check if we have reached the goal            
        if newNode.x == finishPoint.x and newNode.y == finishPoint.y:
            print('FINISH POINT REACHED. BREAK OUT OF LOOP')
            break
       
        print('\n')  
    while(newNode.prev != None):
        rrt.append((newNode.x/100.0,newNode.y/100.0))
        print(f"Location: {newNode.x} , {newNode.y}")
        drawLine(newNode, newNode.prev,(0, 0, 255), 4)
        newNode = newNode.prev

    # cv2.imshow('colour-based', map)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    for r in rrt:
        print(r)

    return rrt

# import readImage as RI
#===========================================================================================
def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        self.cylinderList = []

        self._cur_x = 0.0
        self._cur_y= 0.0
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_t = 0.0
        self.max_vel = 0.2
        self.max_gain = 5.0
        self.newGoal= "0.0&0.0"
        # self.createMap = ""

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('goal_x', value=self._goal_x)
        self.declare_parameter('goal_y', value=self._goal_y)
        self.declare_parameter('goal_t', value=self._goal_t)
        self.declare_parameter('max_vel', value=self.max_vel)
        self.declare_parameter('max_gain', value=self.max_gain)
        self.declare_parameter('newGoal', value=self.newGoal)
        # self.declare_parameter('createMap', value=self.createMap)
        self.max_vel = 3.0
        self.max_gain = 5.0

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

    class roundObject:
        def __init__(self, x, y, r):
            self.x = x
            self.y = y
            self.r = r
    # Parameters are basically useless now for vel_Gain and max_vel
    def _listener_callback(self, msg, vel_gain=5.0, max_vel=5.2, max_pos_err=0.05):
       
        vel_gain = self.max_gain
        max_vel = self.max_vel
        pose = msg.pose.pose
        cur_x = pose.position.x
        cur_y = pose.position.y

        self._cur_x = cur_x
        self._cur_y = cur_y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        cur_t = yaw
   

        x_diff = self._goal_x - cur_x
        y_diff = self._goal_y - cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)

        angleToGoal = atan2(y_diff,x_diff)
        twist = Twist()


        if dist > max_pos_err: # is the distance far enough to travel to ?
            # The X speed should be the distance

            vector = [x_diff * vel_gain, y_diff * vel_gain]

            mag = math.sqrt(vector[0]**2 + vector[1]**2 )
            if mag > max_vel:
                vector[0] /= mag
                vector[1] /= mag

                vector[0] *= max_vel
                vector[1] *= max_vel

            x = vector[0]
            y = vector[1]

            twist.linear.x = x * math.cos(cur_t) + y * math.sin(cur_t)
            twist.linear.y = -x * math.sin(cur_t) + y * math.cos(cur_t)
        self._publisher.publish(twist)

    def parameter_callback(self, params):
        self.get_logger().info(f'move_robot_to_goal parameter callback')
        changedGoal = False
        for param in params:
            if param.name == 'goal_x' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_x = param.value
                changedGoal = True
            elif param.name == 'goal_y' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_y = param.value
                changedGoal = True
            elif param.name == 'goal_t' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_t = param.value
                changedGoal = True
            elif param.name == 'max_vel' and param.type_ == Parameter.Type.DOUBLE:
                self.max_vel = param.value
            elif param.name == 'max_gain' and param.type_ == Parameter.Type.DOUBLE:
                self.max_gain = param.value
            elif param.name == 'newGoal' and param.type_ == Parameter.Type.STRING:
                self._goal_x = float(param.value.split("&")[0])
                self._goal_y = float(param.value.split("&")[1])
                changedGoal = True
            # elif param.name == 'createMap' and param.type == Parameter.Type.STRING:
            #     pose = msg.pose.pose
            #     cur_x = pose.position.x
            #     cur_y = pose.position.y
            #     makeMap(self.)
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
           
            if changedGoal:
                self.get_logger().warn(f"Changing goal {self._goal_x} {self._goal_y} {self._goal_t}")
            else:
                self.get_logger().warn(f"Max Velocity Change: {self.max_vel} Max gain changed: {self.max_gain}")
        return SetParametersResult(successful=True)


def makeMap ():
    pass
   

def driver_callback(node, params):
    node.get_logger().info(f'MOVING TO GOAL')
   
    for param in params:
        if param.name == 'goal_x' and param.type == Parameter.Type.DOUBLE:
            node._goal_x = param.value
            changedGoal = True
        elif param.name == 'goal_y' and param.type == Parameter.Type.DOUBLE:
            node._goal_y = param.value
            changedGoal = True
        else:
            node.get_logger().warn(f'INVALID PARAMETER FROM readImage')
           
        if changedGoal:
            node.get_logger().warn(f"Changing goal {node._goal_x} {node._goal_y}")
        else:
            node.get_logger().warn(f"Max Velocity Change: {node.max_vel} Max gain changed: {node.max_gain}")
    return SetParameterResult(successful=True)

import os
import cv2
def main(args=None):
   
    rclpy.init(args=args)
    node = MoveToGoal()
   
    #node._goal_x = 3.0
    list12 = mainRead(int(node._cur_x),int(node._cur_y))
    list12.reverse()
    for p in list12:
        node.get_logger().info(f'{p}')
        node._goal_x = p[0]
        node._goal_y = p[1]
       
        node.add_on_set_parameters_callback(driver_callback)
       
        node.get_logger().info(f'_goal_x = {node._goal_x}')
        node.get_logger().info(f'_goal_y = {node._goal_y}')
       
   
   
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
if __name__ == '__main__':
    main()

