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
        # look ahead
        _, node = closestNode(X, Y, trajectory)
        forwardIndex = 20
        
        try:
            psiDesired =np.arctan2(trajectory[node+forwardIndex,1]-trajectory[node,1],trajectory[node+forwardIndex,0]-trajectory[node,0])
            e1 =  (Y - trajectory[node+forwardIndex,1])*np.cos(psiDesired) -(X - trajectory[node+forwardIndex,0])*np.sin(psiDesired)
        except:
            psiDesired = np.arctan2(trajectory[-1,1]-trajectory[node,1],trajectory[-1,0]-trajectory[node,0])
            e1 =  (Y - trajectory[-1,1])*np.cos(psiDesired) - (X -trajectory[-1,0])*np.sin(psiDesired)

  
        e2 = wrapToPi(psi - psiDesired)
        e1dot = ydot + xdot*wrapToPi(psi - psiDesired)
        e2dot = psidot

        e = np.hstack((e1, e1dot, e2, e2dot))
        
        
        def dlqr(A,B,Q,R):
            S = np.matrix(linalg.solve_discrete_are(A, B, Q, R))
            #compute the LQR gain
            K = -np.matrix(linalg.inv(B.T@S@B+R)@(B.T@S@A))
            eigVals, eigVecs = linalg.eig(A+B@K)
            return K, X, eigVals

        A = np.array([[0,1,0,0], [0,-4*Ca/(m*xdot),4*Ca/m,-2*Ca*(lf-lr)/(m*xdot)], [0,0,0,1], [0,-2*Ca*(lf-lr)/(Iz*xdot),2*Ca*(lf-lr)/Iz,-2*Ca*(lf**2+lr**2)/(Iz*xdot)]])

        B = np.array([[0], [2*Ca/m], [0], [2*Ca*lf/Iz]])
        
        C = np.eye(4)
        D = np.zeros((4,1))
        
        CT = signal.StateSpace(A, B, C, D)
        DT = CT.to_discrete(delT)
        
        A = DT.A
        B = DT.B
        
        Q = np.eye(4)
        
        R = 0.8
        K,_,_ = dlqr(A,B,Q,R)

        delta = float(K@e)
        
        


        

        
        
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
