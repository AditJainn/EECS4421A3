import json
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import math
import random

numberOfNodes = 700
radiusOfNodes = 1

class vertex:
    def __init__(self, x, y,radius = 1):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (255,0,0)
    
def doesConnect(listOfCircles, vert):
    doesConnectBool = False
    x1 = vert.x 
    y1 = vert.y
    for obstruction in listOfCircles:
        x2 = listOfCircles[obstruction]["x"]*100
        y2 = listOfCircles[obstruction]["y"]*100
        r1 = listOfCircles[obstruction]["r"]*100

        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        doesConnectBool = doesConnectBool or (distance <= (r1 + 1))

    # Calculate distance between circle centers
    return doesConnectBool

def line_color_intersection(world, v1, v2):

    # we are going to check if the pixel at the specified coordinates is white
    def is_not_white(x, y):
        return not all(world[y, x] == [255, 255, 255])


    # first, we are going to assign the point coordinates to new variables
    x1, y1, x2, y2 = v1.x, v1.y, v2.x, v2.y
    

    dx, dy = abs(x2 - x1), abs(y2 - y1)
    
    # now, the x and y coordinates will be that of the first point
    x, y = x1, y1

    # here, we do some shifting by steps. If x1 > x2. For example, if x1 is greater than x2 then we 
    # will take a negative step in the x axis with sx. Similar for the y axis with sy
    sx = -1 if x1 > x2 else 1
    sy = -1 if y1 > y2 else 1

    # err represents the direction we will move in along the line
    err = dx - dy

    while True:
        if is_not_white(x, y):
            return True

        if x == x2 and y == y2:
            break

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy

    return False

file_path = 'map.json'

with open(file_path, 'r') as file:
    json_list = json.load(file)

white_image = np.ones((1000, 1000, 3), dtype=np.uint8) * 255
color = (100, 100, 100) 

for json_object in json_list:
    center = (json_list[json_object]["x"]*100,json_list[json_object]["y"]*100)
    radius = int(json_list[json_object]["r"]*100)
    cv.circle(white_image, center, radius, color, thickness=cv.FILLED)


randomPoints = [(np.random.randint(10, 990), np.random.randint(10, 990)) for _ in range(numberOfNodes)]
listOfVertix = []
for i in range(0,numberOfNodes):
    v = vertex(x = randomPoints[i][0], y = randomPoints[i][1])
    if (doesConnect(json_list,v)):
        v.color = (0, 0, 255)   # Mark a red dot 
    else: 
        listOfVertix.append(v)
    # print(v.x)
    cv.circle(white_image, (v.x,v.y), 3, v.color, thickness=cv.FILLED)


def findDistace (v1, v2):
    distance = math.sqrt((v1.x - v2.x)**2 + (v2.y - v1.y)**2)
    return distance

def findClosetVertex(exploredList, newVertex): 
    closestVertex = 0 
    distance = -1 
    for v in exploredList:
        calculatedDistance = findDistace(v,newVertex) 
        if distance == -1 or calculatedDistance < distance:
            closestVertex = v
    return closestVertex
def drawLine(v1,v2):
    cv.line(white_image, (v1.x,v1.y), (v2.x,v2.y), (0,0,0), thickness=2)
    # white_image
    
        # Find shortest distance 

def findClosetNodeToGraph(exploredVertexList,listOfVertix):
    newConnection = 0 # This will be two different vertexes we will be returning
    smallestDistance = -1 
    for exploredV in exploredVertexList:
        for unexploredVertix in listOfVertix:
            calculateDistance = findDistace(exploredV,unexploredVertix)
            if smallestDistance == -1 or calculateDistance < smallestDistance:
                smallestDistance = calculateDistance
                newConnection = (exploredV,unexploredVertix)

    return newConnection


exploredVertexList = []
startVertex = listOfVertix.pop()


cv.circle(white_image, (startVertex.x,startVertex.y), 6, (0,255,0), thickness=cv.FILLED)

exploredVertexList.append(startVertex)

while(len(listOfVertix) > 0 ): 
    graphNode, newNode= findClosetNodeToGraph(exploredVertexList, listOfVertix)
    
    drawLine(graphNode,newNode)
    exploredVertexList.append(newNode)
    listOfVertix.remove(newNode)
    

def main():
    cv.imshow('world', white_image)
    cv.waitKey(0)
    cv.destroyAllWindows()
    # plt.figure(figsize=(10, 15))
    # plt.imshow(white_image)
    # plt.show()

main()