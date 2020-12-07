# This file contains the scipy.integrate function called in the runSimulation.py file. 
# I also added the force contraint function here and in the kalman filter (when it's being called)

import numpy as np
from signalGenerator import signalGen
import pendulumParam as P
import runDiffVals as Vals
import scipy
import random

import control

class pendulumCnt:

    def __init__(self, m1=None, m2=None, ell=None, b=None, g=None,
                 Ts=None,angle_limit=None, K=None, param=None, zref=None):
        #Initial state conditions
        
        # Mass of the pendulum, kg
        if not m1 is None:
            self.m1 = m1          
        elif not param is None:
            self.m1 = param.m1
        
        # Mass of the cart, kg
        if not m2 is None:            
            self.m2 = m2
        elif not param is None:
            self.m2 = param.m2
       
         # Length of the rod, m
        if not ell is None:    
            self.ell = ell
        elif not param is None:
            self.ell = param.ell       
            
        # Damping coefficient, Ns 
        if not b is None:
            self.b = b                   
        elif not param is None:
            self.b = param.b            
                
        # Gravity constant
        if not g is None:
            self.g = g            
        elif not param is None:
            self.g = param.g          
            
        # sample rate of controller
        if not Ts is None:    
            self.Ts = Ts          
        elif not param is None:
            self.Ts = param.Ts        
            
        if not K is None:
            self.K = K
        elif not param is None:
            if 'K' in param.__dict__.keys():
                self.K = param.K
            else:
                self.K= None
        else:
            self.K = None
            
        self.angle_limit = np.pi*2.0
        
        if not zref is None:
            self.zref=zref
        else:
            #self.reference = signalGen(amplitude=.5, frequency=0.5, y_offset=0)
            self.reference = signalGen(amplitude=500, frequency=5, y_offset=1)
            #self.zref= self.reference.square
            self.zref = self.reference.sine

    ####################################################
    #               scipy.integrate
    ####################################################
    def cartpendfunc(self,y,t):
        #unpack the state
        z, zdot, theta, thetadot = y

        #don't allow theta to exceed 2pi
        #need this so that equivalnent angles don't cause a difference in 
        #in the reference input
        theta = self.limitTheta(theta)

        #Get reference inputs from signal generators
        zref = self.zref(t)      #control cart z location
        
        #calculting the new control force
        curr_state = np.array([[z], [zdot], [theta], [thetadot]])   #current state
        des_state = np.array([zref, [0.0], [np.pi], [0.0]])         #desired state

        #Feedback control. If there's no gain it assigns the control to be 0
        '''
        Compute the input self.u
        Your Code Here

        '''
        # dpoles = np.array([-1.1, 
        #             -1.2, 
        #             -1.3, 
        #             -1.4])

        # Kr = control.place(P.A,P.B,dpoles)

        # self.u = Kr*np.subtract(des_state, curr_state) 

        # Q = np.matrix([[1, 0.0, 0.0, 0.0],
        #         [0.0, 1, 0.0, 0.0],
        #         [0.0, 0.0, 10, 0.0],
        #         [0.0, 0.0, 0.0, 100]])
        # R = np.matrix([1000])


        theta_noise     = .01
        theta_dot_noise = .01
        x_noise         = .01
        x_dot_noise     = .01

        noise = ([[random.gauss(0, x_noise)], [random.gauss(0, x_dot_noise)], [random.gauss(0, theta_noise)], [random.gauss(0, theta_dot_noise)]])

        noisy_state = np.add(curr_state, noise)
        noisy_state[2] = self.limitTheta(noisy_state[2])



        # Q = np.matrix([[1.0, 0.0, 0.0, 0.0],
        #         [0.0, 1.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.00, 0.0],
        #         [0.0, 0.0, 0.0, 1.000]])
        # R = np.matrix([10000])

        Q = np.matrix([[1.0, 0.0, 0.0, 0.0],
                [0.0, 10.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 10.00]])
        R = np.matrix([1000])
        # Q = Vals.Q
        # Q = Vals.Q
        # R = Vals.R
         #solve algebraic Ricatti equation (ARE)
        S = scipy.linalg.solve_continuous_are(P.A, P.B, Q, R) 
         # Solve the ricatti equation and compute the LQR gain
        Kr = np.linalg.inv(R).dot(P.B.transpose().dot(S)) 
        self.u = Kr*np.subtract(des_state, curr_state)
        #newU = Kr*np.subtract(des_state, curr_state)
        newU = Kr*np.subtract(des_state, noisy_state)
      #  print(newU) 

        if(newU>0): 

            if(newU>1.85): self.u = np.array([1.85]) #saturation

            elif(newU<0.6): self.u = 0#np.array([0.0]) #deadband

            #elif(newU<0.6): self.u = np.array([0.6]) #deadband

            else: self.u = newU #normal

            #self.u = newU

        else: 

            if(newU<-1.85): self.u = np.array([-1.85])

            elif(newU>-0.6): self.u = 0#np.array([0.0]) #deadband

            #elif(newU>-.6): self.u = np.array([-0.6])

            else: self.u = newU
            #self.u = newU
        # #print(self.u)


        #simplifications for the calculations - constants
        Sy = np.sin(theta)
        Cy = np.cos(theta)
        D = self.m1*self.ell*self.ell*(self.m2+self.m1*(1.0-Cy*Cy))

        #calculating state values at the current time step
        ydot0 = zdot
        ydot1 = (1.0/D)*(-self.m1*self.m1*self.ell*self.ell*self.g*Cy*Sy + self.m1*self.ell*self.ell*(self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) + self.m1*self.ell*self.ell*(1.0/D)*self.u
        ydot2 = thetadot
        ydot3 = (1.0/D)*((self.m1+self.m2)*self.m1*self.g*self.ell*Sy    - self.m1*self.ell*Cy*      (self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) - self.m1*self.ell*Cy*(1.0/D)*self.u
        dydt = [ydot0, ydot1, ydot2, ydot3]
        return dydt


    ####################################################
    #              Extra Functions
    ####################################################
    def limitTheta(self, angle):
        if abs(angle) > self.angle_limit:
            rem = (abs(angle) % self.angle_limit)*np.sign(angle)
        elif abs(angle) <= self.angle_limit:
            rem = angle
        return rem
    