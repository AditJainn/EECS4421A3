import cv2
import numpy as np
import math
import random

numberOfNodes = 500

class point:
    def __init__(self, x, y,radius = 1):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (255,0,0)

# USE THIS METHOD TO DETERMINE IF A GENERATED POINT IS ON AN OBSTACLE
def point_obst_overlap(map, p):
    def is_not_free(x, y):
        overlap = False
        if all(map[y, x] == [0, 0, 0]) or all(map[y, x] == [0, 0, 255]):
            overlap = True
        return overlap

    mapx, mapy = p.x, p.y
    if is_not_free(mapx, mapy):
        return True    
    return False

# USE THIS METHOD TO DETIRMINE IF A LINE BETWEEN TWO VALID POINTS CROSSES OVER AN OBSTACLE
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


#=========================================================================

# first need to define the list of generated points

map = cv2.imread('map.jpg')

randomPoints = [(np.random.randint(10, 990), np.random.randint(10, 990)) for _ in range(numberOfNodes)]
listOfVertix = []
for i in range(0,numberOfNodes):
    v = point(x = randomPoints[i][0], y = randomPoints[i][1])
    if point_obst_overlap(map,v):
        v.color = (0, 0, 255)   # Mark a red dot 
    else: 
        listOfVertix.append(v)
    # print(v.x)
    cv2.circle(map, (v.x,v.y), 3, v.color, thickness=-1)



#=========================================================================

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
    cv2.line(map, (v1.x,v1.y), (v2.x,v2.y), (0,0,0), thickness=2)

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


cv2.circle(map, (startVertex.x,startVertex.y), 6, (0,255,0), thickness=-1)

exploredVertexList.append(startVertex)

while(len(listOfVertix) > 0 ): 
    graphNode, newNode= findClosetNodeToGraph(exploredVertexList, listOfVertix)
    
    drawLine(graphNode,newNode)
    exploredVertexList.append(newNode)
    listOfVertix.remove(newNode)
    

def main():
    cv2.imshow('colour-based', map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # plt.figure(figsize=(10, 15))
    # plt.imshow(white_image)
    # plt.show()

main()
