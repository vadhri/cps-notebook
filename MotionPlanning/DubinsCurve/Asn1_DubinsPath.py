#Dubin's path 
import numpy as np
import matplotlib.pyplot as plt
import math
cos = np.cos
sin = np.sin
norm = np.linalg.norm
atan2 = np.arctan2
r2d = np.rad2deg
cross = np.cross
atan = math.atan
sqrt = np.sqrt
pi = np.pi
acos = math.acos

#TODO corresponds to student section, you have to complete this
#DONE------------ corresponds to sections already completed, you do not have to change anything

#return Dubin's path
def returnDubinsPath(x1, y1, psi1, x2, y2, psi2, R):
    #print([x1, y1, psi1], [x2, y2, psi2], R)
    #find turn types (RSL, LSR, RSR, or LSL)

    u_vec1 = np.array([cos(psi1), sin(psi1)])
    u_vec2 = [x2-x1, y2-y1]
    u_vec2 = u_vec2/norm(u_vec2)
    u_vec3 = np.array([cos(psi2), sin(psi2)])
    
    #print("\n--> Step 1: frm U1, U2, U3 vectors")
    #print(u_vec1)
    #print(u_vec2)
    #print(u_vec3)

    #TODO: update cp1 = cross product of u_vec1, u_vec2
    #TODO: update cp2 = cross product of u_vec2 and u_vec3
    cp1 = np.dot(u_vec1, u_vec2)
    cp2 = np.dot(u_vec2, u_vec3)
    
    #print("\n--> Step 2: Find cross product")
    #print("cp1=", cp1)
    #print("cp2=", cp2)

    #DONE------------
    sigma1, sigma2, turnType = findCSCTurnType(cp1, cp2)

    #print("\n--> Step 3: Identify turn type and Sigma values")
    #print("turnType = ", turnType)
    #print("sigma1 =", sigma1)
    #print("sigma2 =", sigma2)

    xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
    xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)
    #print("\n--> Step 4: Find circle centers")
    #print("(xc1, yc1)=", [xc1, yc1])
    #print("(xc2, yc2)=",[xc2, yc2])
    #print("Distance between centers, D = ", distance(xc1, xc2, yc1, yc2), " ;3*R= ", 3*R)

    #DONE------------
    if distance(xc1, xc2, yc1, yc2) >= 3*R:
        xtrajectory, ytrajectory, pathDistance = calculateCSCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, psi1, psi2, R, turnType, sigma1, sigma2)
    #DONE------------    
    elif distance(xc1, xc2, yc1, yc2) < 3*R and distance(xc1, xc2, yc1, yc2) >= 2*R:
        sigma1, sigma2, turn, proceed = findCCCTurnType(turnType)
        if proceed: 
            xtrajectory, ytrajectory, pathDistance = calculateCCCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, R, sigma1, sigma2)
        else:
            xtrajectory = None 
            ytrajectory = None
            pathDistance = None
    #DONE-----------  
    else:
        xtrajectory = None 
        ytrajectory = None       
        pathDistance = None
        
    return xtrajectory, ytrajectory, pathDistance
        
        
#TODO: find CSC turn types    
def findCSCTurnType(cp1, cp2):
    if (cp1 <= 0 and cp2 > 0):
        turn = "RSL"
        sigma1 = -1
        sigma2 = 1
    elif (cp1 <= 0  and cp2 < 0):
        turn = "RSR"
        sigma1 = -1
        sigma2 = -1
    elif (cp1 > 0 and cp2 < 0):
        turn = "LSR"
        sigma1 = 1
        sigma2 = -1
    elif (cp1 > 0 and cp2 > 0):
        turn = "LSL"
        sigma1 = 1
        sigma2 = 1
    
    return sigma1, sigma2, turn

#TODO: find CCC turn types
def findCCCTurnType(turnOld):
    proceed = False
    sigma1 = 0
    sigma2 = 0
    turn = turnOld
    if turnOld[0] == turnOld[2]:
        if turnOld[0] == "R":
            turn = "RLR"
            sigma1 = -1
            sigma2 = -1
        elif turnOld[0] == "L":    
            #TODO: set turn, sigma1 and sigma2, delete line 92
            turn = "LRL"
            sigma1 = 1
            sigma2 = 1        

        proceed = True
    return sigma1, sigma2, turn, proceed

#TODO: return circle center
def returnCircleCenter(x, y, psi, sigma, R):
    #update xc and yc
    xc = x+(R*np.cos(psi + (np.pi/2)*sigma))
    yc = y+(R*np.sin(psi + (np.pi/2)*sigma))
    return xc, yc

#DONE------------ return distance
def distance(x1, x2, y1, y2):
    return sqrt((y2-y1)**2 + (x2-x1)**2)

#TODO: return CSC trajectory
def calculateCSCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, psi1, psi2, R, turnType, sigma1, sigma2):
    #find heading and distance
    psiL = atan2(yc2-yc1, xc2-xc1)    
    Stan = distance(xc1, xc2, yc1, yc2) 
    Sstr = 0 

    if turnType == "RSR" or turnType == "LSL":
        psiD = psiL
    elif turnType == "LSR":
        #TODO: adjust psiD and if it exceeds pi, then reset psiD by subtracting 2pi, delete "pass"
        psiD = psiL + atan(2*R/Stan)
        if psiD > np.pi:
            psiD = psiD - 2*np.pi
    elif turnType == "RSL":
        psiD = psiL - atan(2*R/Stan)
        if psiD < -np.pi:
            psiD = 2 * pi - abs(psiD)

    Sstr = np.sqrt(Stan**2 + 4*R**2)
    #print("\n--> Step 5: calculateCSCTrajectory psiD, ", psiD, (turnType, Sstr))
    #initialize path distance
    pathDistance = 0
        
    #determine cut out point (psi1 to psiD) and turn out trajectory 
    xt1 = [x1]
    yt1 = [y1]
    xco = [x1]
    yco = [y1]
    psiT = psi1
    i = 1
    while (not psiD - 0.1 < psiT < psiD + 0.1):
        i = i + 1
        Mco = np.array([[cos(0.05*i),-sigma1*sin(0.05*i)],[sigma1*sin(0.05*i),cos(0.05*i)]])
        vec1 = np.array([[x1 - xc1],[y1 - yc1]])
        [xco, yco] = Mco @ vec1 + np.array([[xc1],[yc1]])    
        xt1.append(xco[0])
        yt1.append(yco[0])
        
        pathDistance = pathDistance + 0.05*R
        psiT = psiT + sigma1*0.05
        
        if (psiT < -pi):
            psiT = 2 * pi - abs(psiT)
        if (psiT > pi):
            psiT = psiT - 2 * pi
            
    #determine cut in point (psi2 to psiD) and turn in trajectory (reverse)
    xt2 = [x2]
    yt2 = [y2]
    xci = [x2]
    yci = [y2]
    psiT = psi2
    i = 1
    while (not psiD - 0.1 < psiT < psiD + 0.1):
        i = i + 1
        #TODO: Update Mci using sigma2 in opposite direction, and vec2 to the vector from <xc2, yc2> to <x2, y2>), and xci, yci
        Mci = np.array([[cos(0.05*i),sigma2*sin(0.05*i)],[-sigma2*sin(0.05*i),cos(0.05*i)]])
        vec2 = np.array([[x2-xc2],[y2-yc2]])

        [xci, yci] = Mci @ vec2 + np.array([[xc2],[yc2]])
        
        xt2.insert(0,xci[0])
        yt2.insert(0,yci[0])

        pathDistance = pathDistance + R*(0.05)
        psiT = psiT - sigma2*0.05
        
        if (psiT < -pi):
            psiT = 2 * pi - abs(psiT)
        if (psiT > pi):
            psiT = psiT - 2 * pi
    
    ##DONE------------ Get straight trajectory from cut out to cut in points, and the overall trajectory / path distance
    xtrajectory = xt1
    ytrajectory = yt1
    v = np.array([xci[0] - xco[0], yci[0] - yco[0]])
    u_hat = v/norm(v)
    dist = distance(xci[0], xco[0], yci[0], yco[0])
    for j in range(int(dist/3)):
        xtrajectory.append(xtrajectory[-1] + 3*u_hat[0])
        ytrajectory.append(ytrajectory[-1] + 3*u_hat[1])
    for j in range(len(xt2)):
        xtrajectory.append(xt2[j])
        ytrajectory.append(yt2[j])  
    pathDistance = pathDistance + dist
        
    return xtrajectory, ytrajectory, pathDistance

#TODO: return CCC trajectory
def calculateCCCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, R, sigma1, sigma2):
    #find heading and distance
    Stan = distance(xc1, xc2, yc1, yc2)  
    u_vec4 = np.array([[xc2-xc1], [yc2-yc1]])
    u_vec4 = u_vec4/norm(u_vec4)
    
    #DONE--------------
    theta = acos(Stan/(4*R))
    Mco = np.array([[cos(theta),-sigma1*sin(theta)],[sigma1*sin(theta),cos(theta)]])
    [xc3, yc3] = 2*R*(Mco @ u_vec4) + np.array([[xc1],[yc1]])
    [xco, yco] = R*(Mco @ u_vec4) + np.array([[xc1],[yc1]])
    #TODO: Update Mci using sigma2 and theta (opposite rotation direction as Mco)
    Mci = np.array([[cos(theta),sigma2*sin(theta)],[-sigma2*sin(theta),cos(theta)]])
    [xci, yci] = -R*(Mci @ u_vec4) + np.array([[xc2],[yc2]])
    
    #Interpolate and get discrete trajectory points using geometry (not vehicle dynamics)
    xtrajectory = [x1]
    ytrajectory = [y1]
    
    #initialize path distance
    pathDistance = 0
    
    #TODO: first turn towards cut out point
    while distance(xtrajectory[-1], xco, ytrajectory[-1], yco) > 3:
        Mco = np.array([[cos(0.1),-sigma1*sin(0.1)],[sigma1*sin(0.1),cos(0.1)]])
        vec1 = np.array([[xtrajectory[-1] - xc1],[ytrajectory[-1] - yc1]])
        [xnew, ynew] = Mco @ vec1 + np.array([[xc1],[yc1]])
        
        #TODO: append xnew, ynew to xy_trajectory, add 0.1*R to path distance
        xtrajectory.append(xnew[0])
        ytrajectory.append(ynew[0])
    
        pathDistance = pathDistance + 0.1*R

    xtrajectory.append(xco[0])
    ytrajectory.append(yco[0]) 
    
    #TODO: second turn in opposite direction - middle circle
    while distance(xtrajectory[-1], xci, ytrajectory[-1], yci) > 3:  
        #TODO: update Mci using sigma2 in opposite direction (0.1 rad intervals), vec2 (vector from <xc3, yc3> to latest point of xy_trajectory), and xnew, ynew
        Mci = np.array([[cos(0.1),sigma2*sin(0.1)],[-sigma2*sin(0.1),cos(0.1)]])
        vec2 = np.array([[xtrajectory[-1] - xc3[0]],[ytrajectory[-1] - yc3[0]]])
        [xnew, ynew] = Mci @ vec2 + np.array([[xc3[0]],[yc3[0]]])
        #TODO: append xnew, ynew to xy_trajectory, add 0.1*R to path distance
        xtrajectory.append(xnew[0])
        ytrajectory.append(ynew[0])
        pathDistance = pathDistance + 0.1*R

    xtrajectory.append(xci[0])
    ytrajectory.append(yci[0])
    
    #TODO: final turn towards second waypoint  
    while distance(xtrajectory[-1], x2, ytrajectory[-1], y2) > 3:  
        #TODO: update Mc3 using sigma1 (0.1 rad intervals), vec3 (vector from <xc2, yc2> to latest xy_trajectory), and xnew, ynew
        Mc3 = np.array([[cos(0.1),-sigma1*sin(0.1)],[sigma1*sin(0.1),cos(0.1)]])
        vec3 = np.array([[xtrajectory[-1] - xc2],[ytrajectory[-1] - yc2]])
        [xnew, ynew] = Mc3 @ vec3 + np.array([[xc2],[yc2]])
        #TODO: append xnew, ynew to xy_trajectory, add 0.1*R to path distance
        xtrajectory.append(xnew[0])
        ytrajectory.append(ynew[0])
        pathDistance = pathDistance + 0.1*R

    xtrajectory.append(x2)
    ytrajectory.append(y2)
        
    return xtrajectory, ytrajectory, pathDistance
