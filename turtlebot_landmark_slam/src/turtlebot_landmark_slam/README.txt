
"""

Helperscripts are meant to help you carry out the Q3
The below is a quick explanation of each function, its inputs and outputs
##########################################################################################

def Relative2AbsolutePose (robot_abs, u):
    
    '''
    Calculates the robot new pose given previous pose and motion
    Input: absolute coordinate of robot [x1,y1,theta1]
           motion command with respect to robot frame [dx, dy, dtheta]
    Output: absolute coordinate of robot next pose [x2,y2,theta2] 
    '''
#########################################################################################

def Relative2AbsoluteXY(robot_abs,landmark_meas_xy):

    '''
    Calculates Landmark's absolute coordinate
    Input: robot's absolute coordinate [x,y,theta]
           landmark's measurment with repect to robot frame [x,y]
    Output: landmarks's absolute coordinate  [x,y]
    '''
##########################################################################################

def Absolute2RelativeXY(robot_abs,landmark_abs):
    
    '''
    Expresses Landmark's Coordinate on Robot Frame
    Input: robot's absolute coordinate [x,y,theta]
           landmarks absolute coordinate [x,y]
    Output: landmarks's relative coordinate with repect to robot frame [x,y] 
    i.e. landmark's measurement from robot
    '''
#########################################################################################    

def ErrorFunction (solutionFile, GTFile):    
 '''
    Calculates the error between GT data and SLAM obtained  data
    Input: - solution file containing:
               - absolute coordinates of landmark positions
           - GT file containing:
               - GT absolute coordinates of landmark positions
    Output: error = Relative GT landmark positions - Relative estimated landmark positions
    '''

#########################################################################################

def RelativeLandmarkPositions(landmark_abs, next_landmark_abs):
    
    '''
    Calculates the relative landmark positions
    Input: absolute coordinate of landmark [x1,y1]
           absolute coordinate of landmark next position [x2,y2]
    Output: relative position [dx, dy]
    '''
##########################################################################################    

def pi2pi(angle):

"""
Maps angle to the range of [-pi,pi]

    Input: 
        - angle in any range
    Output:
        - angle in the range [-pi, pi] 
"""
###########################################################################################
