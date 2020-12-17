# This file contains physical parameters for the cart and the system state space equations

import numpy as np
import sys
sys.path.append('..')  # add parent directory
import scipy.linalg
from scipy import signal

#Physical parameters of the inverted pendulum known to the controller
m1 = 0.03   # Mass of the pendulum [kg]
m2 = .475   # Mass of the cart [kg] 
ell = 1.21  # Length of the rod [m]
g = -9.81   # Gravity, [m/s^2]
b = 0.78   # Damping coefficient [Ns]
T_update = 0.001 # Controller and Estimator update rate [s]

#parameters for animation
w = 0.15      # Width of the cart [m]
h = 0.07      # Height of the cart [m]
gap = 0.005   # Gap between the cart and x-axis [m]
radius = 0.08 # Radius of circular part of pendulum [m]

#Simulation Parameters
t_start = 0.0   # Start time of simulation [s]
t_end = 30.0    # End time of simulation [s]
Ts = 0.001      # sample time for simulation [s]
t_plot = 0.5    # Animation update rate [s]

#Initial Conditions
z0 = 0.01               # [m]
zdot0 = 0.0             # [m/s]
theta0 = 0              # [rad], starts upright
thetadot0 = 0.0         # [rads/s]

####################################################
#                 State Space
####################################################
#xdot = A*x + B*u
#y = C*x
#x = [z, zdot, theta, thetadot]

A = np.matrix([[0.0, 1.0, 0.0, 0.0],
            [0.0, -b/m2, -m1*g/m2, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, -b/(m2*ell), -(m1+m2)*g/(m2*ell), 0.0]])

B = np.array([[0.0], [1.0/m2], [0.0], [1.0/(m2*ell)]])


#Measure everything
#C = np.matrix([[1.0, 0.0, 0.0, 0.0],[0.0, 1.0, 0.0, 0.0],[0.0, 0.0, 1.0, 0.0],[0.0, 0.0, 0.0, 1.0]]) 
#Measure one state only
C = np.matrix([[0.0, 0.0, 0.0, 1.0]]) 
