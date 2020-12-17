# This is a simple implementation of a Kalman Filter
# Edit process noise (sigma_u) and measurement noise (sigma_n) to improve your Kalman Filter

import numpy as np
import pendulumParam as P
import scipy

sigma_u = np.diag([0.1,0.1,0.1,0.1])
sigma_n = np.diag([0.1])
Ad = scipy.linalg.expm(P.A*P.T_update)
Bd = P.B*P.T_update


def kalmanFilter(mu, sigma, u, y):
    
    mu_p = Ad.dot(mu) + Bd.dot(u) 
    sigma_p = Ad.dot(sigma.dot(Ad.transpose())) + sigma_u
    
    y_m = y-P.C.dot(mu_p)
    sigma_m = P.C.dot(sigma_p.dot(P.C.transpose())) + sigma_n
    kkf_gain = sigma_p.dot(P.C.transpose().dot(np.linalg.inv(sigma_m)))

    mu = mu_p + kkf_gain.dot(y_m)    
    sigma=(np.eye(4)-kkf_gain.dot(P.C)).dot(sigma_p)

    return mu,sigma

