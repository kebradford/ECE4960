# This file runs the nonlinea pendulum dynamics 
# You can add process noise here (line 60)

import numpy as np

class Pendulum:

    def __init__(self, m1=None, m2=None, ell=None, b=None, g=None,param=None):
        
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
            
    def cartpendfunc(self,x,u):
        '''Nonlinear dynamics'''
    
        #unpack the state (named x in lectures)
        z, zdot, theta, thetadot = x
        theta = theta+np.pi #treat 0 as upward
        
        #simplifications for the calculations - constants
        Sy = np.sin(theta[0])
        Cy = np.cos(theta[0])
        D = self.m1*self.ell*self.ell*(self.m2+self.m1*(1.0-Cy*Cy))

        #calculating state values at the current time step
        ydot0 = zdot
        ydot1 = (1.0/D)*(-self.m1*self.m1*self.ell*self.ell*self.g*Cy*Sy + self.m1*self.ell*self.ell*(self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) + self.m1*self.ell*self.ell*(1.0/D)*u
        ydot2 = thetadot
        ydot3 = (1.0/D)*((self.m1+self.m2)*self.m1*self.g*self.ell*Sy    - self.m1*self.ell*Cy*      (self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) - self.m1*self.ell*Cy*(1.0/D)*u
        dydt = [ydot0, ydot1, ydot2, ydot3]


        #with process noise:
        
        dydt = [ydot0 + np.random.randn()*0.1, ydot1 + np.random.randn()*0.1, ydot2 + np.random.randn()*0.1, ydot3 + np.random.randn()*0.1]

        return dydt


