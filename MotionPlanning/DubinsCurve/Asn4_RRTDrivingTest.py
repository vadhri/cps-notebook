#RRT for driving algorithm
import numpy as np
import matplotlib.pyplot as plt
import random
mod = np.mod
from Asn1_DubinsPath import atan2, returnDubinsPath
from Asn4_RRTDubinsDriving import RRTDrivingAlgorithm

#TODO corresponds to student section, you have to complete this
#DONE------------ corresponds to sections already completed, you do not have to change anything

grid = np.load('cspaceRoadmap.npy')
fig = plt.figure("RRT Driving Algorithm")
plt.imshow(grid, cmap='binary', origin = 'lower')

#DONE------------ find near target position
def findNearTarget(i):
    #Drivability map near target positions
    xLocation = [10, 250, 500, 750, 1000, 1250, 1500, 1750, 1990]
    yLocation = [600, 365, 189, 254, 425, 675, 725, 675, 425]
    headingSetpoint = [-0.7854, -0.7854, 0, 0.7854, 0.7854, 0.7854, 0, -0.7854, -0.7854]
    return xLocation[i], yLocation[i], headingSetpoint[i]

#DONE------------ set start and other parameters 
X = 5
Y = 600
start = np.array([X,Y])
psi = -0.7854
turnRadius = 25
numIterations = 50
rrTrees = []

for i in range(8):
    #DONE------------ update near target
    xTarget, yTarget, psiTarget = findNearTarget(i+1)
    plt.plot(xTarget, yTarget, marker="x", markersize=8, markeredgecolor="g", markerfacecolor="g")
    start = np.array([X,Y])
    goal = np.array([xTarget, yTarget])
    
    #TODO: initialize tree
    rrt = RRTDrivingAlgorithm(start, goal, numIterations, grid, turnRadius, psiTarget)
    
    #TODO: for the time period of DT, expand tree (assume 1 sample per timestep)
    for k in range (rrt.iterations):
        
        #TODO: reset nearest values, sample point, findNearest, steer to new, get heading 
        rrt.resetNearestValues()
        #algorithm begins here
        #TODO: sample a point and assign to "point"
        point = rrt.sampleNewPoint(X,Y,psi)
        #TODO: find nearest node
        rrt.findNearest(rrt.randomTree, point)
        
        #steer from the nearest node to point, and assign to "new"
        new = rrt.steerToPoint(rrt.nearestNode, point)
        
        #TODO: find heading to the goal from this new point and add a random value to it (update "0" in line 48)
        heading = atan2(goal[1]-new[1], goal[0]-new[0]) + random.uniform(-1,1)

        if not rrt.isInObstacle(rrt.nearestNode, new):
            #TODO: get Dubins path distance, and if it's not in obstacle and not none, add child
            xTP, yTP, pathDistance = returnDubinsPath(rrt.nearestNode.locationX, 
                                                    rrt.nearestNode.locationY, 
                                                    rrt.nearestNode.heading, 
                                                    new[0], 
                                                    new[1], 
                                                    heading, 
                                                    rrt.turnRadius)                 
            if pathDistance is not None and not rrt.isDubinsInObstacle(xTP, yTP): 
                #TODO: add new as a child with other attributes
                rrt.addChild(new[0], new[1], heading, pathDistance, xTP, yTP)
                plt.pause(0.05)
                plt.plot(xTP,yTP,'k')
                plt.plot(new[0], new[1], marker="x", markersize=3, markeredgecolor="k", markerfacecolor="k")
                
                #TODO: check if there's a direct Dubins path from new point to goal, if so then set nearest node to new, and add goal to it, then find cost
                xTP, yTP, pathDistance = returnDubinsPath(new[0], 
                                                        new[1], 
                                                        heading, 
                                                        goal[0], 
                                                        goal[1], 
                                                        psiTarget, 
                                                        rrt.turnRadius)   
                             
                #TODO: if pathDistance is not None and xTP, yTP are not in an obstacle
                if pathDistance is not None and not rrt.isDubinsInObstacle(xTP, yTP): 
                    #TODO: get projected cost which is distance from new node to root, + pathDistance
                    #this time you have to obtain the newNode (treeNode) object, what will this be? (hint: it is added as a child to the....)
                    projectedCost = rrt.findPathDistance(rrt.nearestNode.children[-1]) + pathDistance
                    
                    #TODO: if projected cost is less than the last value of rrtStar.goalCosts
                    if projectedCost < rrt.goalCosts[-1]:
                        #set the nearestNode to the newNode (again.. how can you find the newNode (treeNode) object)
                        rrt.nearestNode = rrt.nearestNode.children[-1]
                        #add goal as the child with xTP, yTP, pathDistance
                        rrt.addChild(goal[0], goal[1], 0, pathDistance, xTP, yTP)
                        #retrace path
                        rrt.retracePath()
                        plt.plot(xTP, yTP, 'k') 
                        
    #DONE------------ display lowest cost trajectory, and append the tree to rrTrees
    plt.plot(rrt.xPathTrajectory, rrt.yPathTrajectory,"b") 
    rrt.goalCosts.pop(0)
    rrTrees.append(rrt)
    #DONE------------ update X Y location to goal
    X = xTarget
    Y = yTarget  
    psi = psiTarget                     

#DONE------------ Collect final trajectory
xFinalTrajectory = []
yFinalTrajectory = []
totalDistance = 0
for i in range(8):
    for j in range(len(rrTrees[i].xPathTrajectory)):
        xFinalTrajectory.append(rrTrees[i].xPathTrajectory[j])
        yFinalTrajectory.append(rrTrees[i].yPathTrajectory[j])
    print (rrTrees[i].goalCosts)
    totalDistance += rrTrees[i].path_distance
    
#DONE------------ print final information
print("Total Distance", totalDistance)   
plt.figure("Final Trajectory")
grid = np.load('cspaceRoadmap.npy')
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(xFinalTrajectory, yFinalTrajectory)    
plt.show()
