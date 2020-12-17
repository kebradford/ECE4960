# This file assumes a reference input for z (named x in the lectures) variable 
# and plots the z reference and z, as well as, the theta value over time. 
# It should not need to be changed by the students

import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np

plt.ion()  # enable interactive drawing


class plotData:
    ''' 
        This class plots the time histories for the pendulum data.
    '''

    def __init__(self):
        #Number of subplots = num_of_rows*num_of_cols
        self.num_rows = 4    # Number of subplot rows
        self.num_cols = 1    # Number of subplot columns

        #Create figure and axes handles
        self.fig, self.ax = plt.subplots(self.num_rows, self.num_cols, sharex=True)

        #Instantiate lists to hold the time and data histories
        self.time_history = []  # time
        self.zref_history = []  # reference position z_r
        self.z_history = []     # position z
        self.zdot_history = []  # velocity
        self.theta_history = [] # angle
        self.thetadot_history = [] # angular velocity

        #create subplots
        self.ax[0].set(xlabel='time(s)', ylabel='z(m)', title='Performance')
        self.ax[1].set(xlabel='time(s)', ylabel='zdot(m/s)')
        self.ax[2].set(xlabel='time(s)', ylabel='theta(deg)')
        self.ax[3].set(xlabel='time(s)', ylabel='thetadot(deg/s)')

    def Plot(self, t, reference, states, lineWidth=1):
        '''
            Add to the time and data histories, and update the plots.
        '''
        #the time history of all plot variables
        self.time_history = t  # time
        self.zref_history = reference[:,0]  # reference base position
        self.z_history = states[:,0]  # base position
        self.zdot_history = states[:,1] # velocity
        self.theta_history = 180.0/np.pi*states[:,2]  # pendulum angle (converted to degrees)
        self.thetadot_history = 180/np.pi*states[:,3] #angular velocity

        #the plots with associated histories 
        line1, = self.ax[0].plot(self.time_history, self.zref_history, label='Z-Reference')
        line2, = self.ax[0].plot(self.time_history, self.z_history, label='Z',linewidth=lineWidth)
        line3, = self.ax[1].plot(self.time_history, self.zdot_history, label='Zdot',linewidth=lineWidth)
        line4, = self.ax[2].plot(self.time_history, self.theta_history, label='theta',linewidth=lineWidth)
        line5, = self.ax[3].plot(self.time_history, self.thetadot_history, label='thetadot',linewidth=lineWidth)

        plt.show()