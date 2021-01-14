# Fill in the respective functions to implement the controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from util import *

# CustomController class (inherits from BaseController)
class CustomController(BaseController):

    def __init__(self, trajectory):

        super().__init__(trajectory)

        # Define constants
        # These can be ignored in P1
        self.lr = 1.39
        self.lf = 1.55
        self.Ca = 20000
        self.Iz = 25854
        self.m = 1888.6
        self.g = 9.81

        # Add additional member variables according to your need here.
        self.oldDist = 0
        
        self.previousXdotError = 0
        
    def update(self, timestep):

        trajectory = self.trajectory

        lr = self.lr
        lf = self.lf
        Ca = self.Ca
        Iz = self.Iz
        m = self.m
        g = self.g

        # Fetch the states from the BaseController method
        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)

        # look ahead
        _, node = closestNode(X, Y, trajectory)
        forwardIndex = 20
        
        nodeTemp = node+forwardIndex
        
        if nodeTemp >= 8203:
            nodeTemp = 0
            
        # try:
            # desiredX =trajectory[node+forwardIndex,0]
            # desiredY = trajectory[node+forwardIndex,1]
            
        # except:
            # desiredX =trajectory[-1,0]
            # desiredY = trajectory[-1,1]
          
        desiredX =trajectory[nodeTemp,0]
        desiredY = trajectory[nodeTemp,1]

        
        # Design your controllers in the spaces below. 
        # Remember, your controllers will need to use the states
        # to calculate control inputs (F, delta). 

        # ---------------|Lateral Controller|-------------------------
        """
        Please design your lateral controller below.
        .
        .
        .
        .
        .
        .
        .
        .
        .
        """
        errAngle = np.arctan2(desiredY-Y,desiredX-X)
        

        # P = np.array([-7, -0.25, -2, -0.2])
        P = np.array([-7, -0.25, -1.55, -0.15])
        A = np.array([[0,1,0,0], [0,-4*Ca/(m*xdot),4*Ca/m,-2*Ca*(lf-lr)/(m*xdot)], [0,0,0,1], [0,-2*Ca*(lf-lr)/(Iz*xdot),2*Ca*(lf-lr)/Iz,-2*Ca*(lf**2+lr**2)/(Iz*xdot)]])

        B = np.array([[0], [2*Ca/m], [0], [2*Ca*lf/Iz]])

        poles = signal.place_poles(A,B,P)
        k = poles.gain_matrix
        
        disErr = np.sqrt((X-desiredX)**2+(Y-desiredY)**2)
        psiErr = wrapToPi(errAngle-psi)
        
        e1 = disErr
        e2 = psiErr
        e1dot = ydot* np.sin(psiErr) + xdot * np.cos(psiErr)
        e2dot = psidot

        e = np.hstack((e1, e1dot, e2, e2dot))
        
        delta = wrapToPi(np.asscalar(k@e))
 
        print(delta)
        
        
        # ---------------|Longitudinal Controller|-------------------------
        """
        Please design your longitudinal controller below.
        .
        .
        .
        .
        .
        .
        .
        .
        .
        """
        kp = 200
        ki = 10
        kd = 30
        
        
        desiredVelocity = 6
        xdotError = (desiredVelocity - xdot)
        self.integralXdotError += xdotError
        derivativeXdotError = xdotError - self.previousXdotError
        self.previousXdotError = xdotError
        F = kp*xdotError + ki*self.integralXdotError*delT +kd*derivativeXdotError/delT

        # Return all states and calculated control inputs (F, delta)
        return X, Y, xdot, ydot, psi, psidot, F, delta
