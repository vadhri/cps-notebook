#RRT algorithm
import numpy as np
import random

#TODO corresponds to student section, you have to complete this
#DONE------------ corresponds to sections already completed, you do not have to change anything

from Asn1_DubinsPath import norm

#DONE------------ treeNode class
class treeNode():
    def __init__(self, locationX, locationY, heading):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.heading = heading                    #Heading (rad)
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 
        self.parentDistance = 0                   #distance to parent node
        self.xTP = None                           #x values of dubins path to parent
        self.yTP = None                           #y values of dubins path to parent
        
#RRT Algorithm class
class RRTAlgorithm():
    def __init__(self, start, goal, numIterations, grid, turnRadius):
        self.randomTree = treeNode(start[0], start[1], 0)           #The RRT (root position) and heading 
        self.goal = treeNode(goal[0], goal[1], 0)                   #goal position and heading
        self.turnRadius = turnRadius                                #Dubins path turn radius
        self.nearestNode = None                                     #nearest node            
        self.iterations = min(numIterations, 200)                   #number of iterations to run
        self.grid = grid                                            #the map
        self.rho = 5*self.turnRadius                                #length of each branch 
        self.path_distance = 0                                      #total path distance  
        self.nearestDist = 10000                                    #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                       #number of waypoints
        self.Waypoints = []                                         #the waypoints
        self.xPathTrajectory = []                                   #x trajectory of entire path
        self.yPathTrajectory = []                                   #y trajectory of entire path        
        
    #TODO: add the node to the nearest node, and add goal if necessary    
    def addChild(self, locationX, locationY, heading, distanceToParent, xTP, yTP):
        if (locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
            self.goal.parentDistance = distanceToParent
            self.goal.xTP = xTP
            self.goal.yTP = yTP
        else:    
            tempNode = treeNode(locationX, locationY, heading)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
            tempNode.parentDistance = distanceToParent
            tempNode.xTP = xTP
            tempNode.yTP = yTP
        
    #DONE------------ sample random point within grid limits
    def sampleAPoint(self):
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        point = np.array([x, y])
        return point
    
    #TODO: steer a distance stepSize from start location to end location (keep in mind the grid limits)
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        #TODO: constrain above point in x and y respectively (remember x, y is grid[y,x]), check min and max limits
        point[0] = min(self.grid.shape[1]-1, point[0])
        point[1] = min(self.grid.shape[0]-1, point[1])
        point[0] = max(0, point[0])
        point[1] = max(0, point[1])

        return point
    
    #DONE------------ check if obstacle lies between the start and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = min(self.grid.shape[1]-1,locationStart.locationX + i*u_hat[0])
            testPoint[1] = min(self.grid.shape[0]-1,locationStart.locationY + i*u_hat[1])
            if self.grid[round(testPoint[1]),round(testPoint[0])] == 1:
                return True
        return False
    
    #TODO: check if the Dubins trajectory falls in an obstacle
    def isDubinsInObstacle(self, xTrajectory, yTrajectory):
        #TODO: iterate through all xTrajectory,yTrajectory points and if any point lies in an obstacle, return True
        #be sure to check for grid limits, again remember x,y is grid[y,x]
        for i in range(len(xTrajectory)):
            xpt = round(min(self.grid.shape[0]-1, yTrajectory[i]))
            ypt = round(min(self.grid.shape[1]-1, xTrajectory[i]))
            if self.grid[xpt, ypt] == 1:
                return True
        return False    

    #DONE------------ find the unit vector between 2 locations
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/norm(v)
        return u_hat
    
    #DONE------------ find the nearest node from a given (unconnected) point (Euclidean distance)
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        #update nearest node
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)

    #DONE------------ find euclidean distance between a node and an XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)         
        return dist
    
    #DONE------------ check if the goal is reached within step size
    def goalFound(self,point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False
    
    #DONE------------ reset nearestNode and nearestDistance
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000
    
    #TODO: trace the path from goal to start
    def retraceRRTPath(self,goal):
        #TODO: return if goal X location is the tree (start) X location
        if goal.locationX == self.randomTree.locationX:
            return 

        #TODO: add 1 to numWaypoints
        self.numWaypoints += 1
        currentPoint = np.array([goal.locationX, goal.locationY])
        self.Waypoints.insert(0,currentPoint)
        #TODO: add each element of goal.xTP and goal.yTP to xPathTrajectory and yPathTrajectory in reverse, from the beginning (insert)
        self.xPathTrajectory = goal.xTP + self.xPathTrajectory
        self.yPathTrajectory = goal.yTP + self.yPathTrajectory

        #TODO: increment path_distance by the goal.parentDistance
        self.path_distance += goal.parentDistance

        #Recursive call
        self.retraceRRTPath(goal.parent)   
                
#end of class definitions
#---------------------------------------------------------------------------------------------------------#
