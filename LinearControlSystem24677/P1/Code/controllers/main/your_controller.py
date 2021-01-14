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
        self.oldAngle = 0
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
        closeNode = closestNode(X, Y, trajectory);
        NodeTemp = closeNode[1]+5; 
        desiredX =trajectory[NodeTemp,0]
        desiredY = trajectory[NodeTemp,1]
        
        
        errAngle = np.arctan2(desiredY-Y,desiredX-X)
        angle = wrapToPi(errAngle-psi)
        
        kPd = 0.4;
        kDd = 0.03;
        kId = 0.001;
        
        Pd = kPd*angle
        Dd = kDd*(angle-self.oldAngle)/delT
        Id = kId*(angle+self.oldAngle)
        thresh = 0.005
        
        if Id >= thresh:
            Id  = thresh
        elif Id <= -thresh:
             Id  = -thresh
        delta = Pd+Dd+Id
        
       
        # print(psi)
        # print(delta)
        
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
        kPf = 10;
        kDf = 1;
        kIf = 0.001;
        
        
        dist = np.sqrt((X-desiredX)**2+(Y-desiredY)**2)
        Pf = kPf*dist
        Df = kDf*(dist-self.oldDist)/delT
        If = kIf* (dist+self.oldDist)
        threshF = 0.01
        if If >= threshF:
            If  = threshF
        elif If <= -threshF:
             If  = -threshF
        errLong = Pf+Df+If
        
        F = 700+errLong
        # print(If)
        
        self.oldDist = dist
        self.oldAngle = angle
        # Return all states and calculated control inputs (F, delta)
        return X, Y, xdot, ydot, psi, psidot, F, delta
