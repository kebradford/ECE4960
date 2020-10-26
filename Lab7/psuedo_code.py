# In world coordinates
def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose 

    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    """
    
    #getting vars from input
    distX     = cur_pose[0] - prev_pose[0];
    distY     = cur_pose[1] - prev_pose[1];
    prevTheta = prev_pose[2];
    curTheta  = cur_pose[2];
    
    #calculating results based on lecture 11
    delta_trans = sqrt(distX^2*distY^2);
    delta_rot_1 = atan2(distY, distX) - prevTheta; 
    delta_rot_2 = curTheta-prevTheta-delta_rot_1;
    

    return delta_rot_1, delta_trans, delta_rot_2

# In world coordinates
def odom_motion_model(cur_pose, prev_pose, u):
    """ Odometry Motion Model

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose
        (rot1, trans, rot2) (float, float, float): A tuple with control data in the format 
                                                   format (rot1, trans, rot2) with units (degrees, meters, degrees)


    Returns:
        prob [float]: Probability p(x'|x, u)
    """
    
    #get translation/rotation data
    computed_control = compute_control(cur_pose, prev_pose);
    delta_trans = computed_control[0];
    delta_rot_1 = computed_control[1];
    delta_rot_2 = computed_control[2];
    

    #calculate probability of each condition 
    prob1 = loc.gaussian(delta_trans, u[0], loc.odom_trans_sigma);
    prob2 = loc.gaussian(delta_rot_1, u[1], loc.odom_rot_sigma);
    prob3 = loc.gaussian(delta_rot_2, u[2], loc.odom_rot_sigma);
    
    prob = prob1*prob2*prob3;
    
    return prob

def prediction_step(cur_odom, prev_odom, u):
    """ Prediction step of the Bayes Filter.
    Update the probabilities in loc.bel_bar based on loc.bel from the previous time step and the odometry motion model.

    Args:
        cur_odom  ([Pose]): Current Pose
        prev_odom ([Pose]): Previous Pose
    """
    sumBel = 0;
    for i in range(loc.MAX_CELLS_X):
        for j in range(loc.MAX_CELLS_Y):
            for k in range(loc.MAX_CELLS_A):
                sumBel += loc.bel[i][j][k]*odom_motion_model(cur_odom, prev_odom, u)
                
    loc.bel_bar = sumBel;   
    
    return void
 

def sensor_model(obs, u):
    """ This is the equivalent of p(z|x).


    Args:
        obs ([ndarray]): A 1D array consisting of the measurements made in rotation loop

    Returns:
        [ndarray]: Returns a 1D array of size 18 (=loc.OBS_PER_CELL) with the likelihood of each individual measurements
    """
    
    prob_array = [0]*loc.OBS_PER_CELL;
    
    for i in range(loc.OBS_PER_CELL):
        prob_array[i] = loc.gaussian(obs[i], u, loc.sensor_sigma);

    return prob_array

def update_step(eta, obs, u):
    """ Update step of the Bayes Filter.
    Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
    """
    
    loc.bel = eta*sensor_model(obs, u)*loc.bel_bar;
    