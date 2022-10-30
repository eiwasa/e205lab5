from traj_planner_utils import *

pi = 3.141529
TIME_STEP_SIZE = 0.01 #s
LOOK_AHEAD_TIME = 1.0 #s
MIN_DIST_TO_POINT = 0.1 #m
MIN_ANG_TO_POINT = 0.10 #rad


class TrajectoryTracker():
  """ A class to hold the functionality for tracking trajectories.
      Arguments:
        traj (list of lists): A list of traj points Time, X, Y, Theta (s, m, m, rad).
  """
  current_point_to_track = 0
  traj_tracked = False
  traj = []
  end_point = []

  def __init__(self, traj):
    self.current_point_to_track = 0
    self.traj = traj
    self.end_point = traj[-1]
    self.traj_tracked = False

  def get_traj_point_to_track(self, current_state):
    """ Determine which point of the traj should be tracked at the current time.
        Arguments:
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
        Returns:
          desired_state (list of floats: The desired state to track - Time, X, Y, Theta (s, m, m, rad).
    """
    """STUDENT CODE START"""
    current_point = self.traj[self.current_point_to_track]
    distance_to_current_traj_point = (((current_point[1] - current_state[1]))**2 + ((current_point[2] - current_state[2])**2))**(1/2)
    distance_to_current_traj_angle = abs(current_point[-1] - current_state[-1])
    
    # if self.current_point_to_track == self.end_point:
    #   self.current_point_to_track = self.current_point_to_track

    if (distance_to_current_traj_point < MIN_DIST_TO_POINT) and (distance_to_current_traj_angle < MIN_ANG_TO_POINT):
      self.current_point_to_track += 1

    if self.current_point_to_track == len(self.traj):
      return -1
    # for i in range(len(self.traj)):
    #     print(i,self.traj[i])
    
    """STUDENT CODE END"""
    return self.traj[self.current_point_to_track]

  def print_traj(self):
    """ Print the trajectory points.
    """
    print("Traj:")
    for i in range(len(self.traj)):
        print(i,self.traj[i])

  def is_traj_tracked(self):
    """ Return true if the traj is tracked.
        Returns:
          traj_tracked (boolean): True if traj has been tracked.
    """
    return self.traj_tracked

class PointTracker():
  """ A class to determine actions (motor control signals) for driving a robot to a position.
  """
  def __init__(self):
    pass

  def get_dummy_action(self, x_des, x):
    """ Return a dummy action for now
    """
    action = [0.0, 0.0]
    return action

  def point_tracking_control(self, desired_state, current_state):
    """ Return the motor control signals as actions to drive a robot to a desired configuration
        Arguments:
          desired_state (list of floats): The desired Time, X, Y, Theta (s, m, m, rad).
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
    """
    # zero all of action
    # right wheel velocity
    # left wheel velocity
    # action = [0.0, 0.0]
    """STUDENT CODE START"""
    # going_forward = True
    # current_state_array = np.array(current_state)
    # print(current_state_array)
    # current_state_array = current_state_array
    print(f"[current_state] {current_state}")
    print(f"[desired_state] {desired_state}")
    # calculate alpha, beta, rho
    delta_x = desired_state[1] - current_state[1]
    delta_y = desired_state[2] - current_state[2]

    rho = ((delta_x**2) + (delta_y**2))**(1/2)
    alpha = -1*current_state[-1] + math.atan2(delta_y, delta_x)
    
    print("[alpha] ", alpha)
    # beta = -1*current_state[-1] - alpha + desired_state[-1]

    # pick k_alpha, k_beta, k_rho values
    if rho > 0.3:
      print(f"[far] rho is {rho}")
      k_rho = 1
      k_beta = -1
      k_alpha = 2
    else:
      print(f"[close] rho is {rho}")
      # for up to trial 0-4: rho 0.01, beta -1, aplha 2 (very slow)
      k_rho = 0.01
      k_beta = -1
      k_alpha = 2
      # k_rho = 0.2
      # k_beta = -2
      # k_alpha = 0.3

    
    # change our v and w equations
    # w = (desired_state[-1] - current_state[-1])/time
    # v = ((desired_state[1]-current_state[1])**2 + (desired_state[2]-current_state[2])**2)**(1/2)/time
    if abs(alpha) <= pi/2:
      print("going forward")
      beta = -1*current_state[-1] - alpha + desired_state[-1]
      w = k_alpha*alpha + k_beta*beta
      v = k_rho*rho
    else:
      print("going backward")
      alpha = -1*current_state[-1] + math.atan2(-1*delta_y, -1*delta_x)
      beta = -1*current_state[-1] - alpha - desired_state[-1]
      w = k_alpha*alpha + k_beta*beta
      v = -1*k_rho*rho
    A = np.array([[1,-1],
          [ROBOT_RADIUS, 1*ROBOT_RADIUS]])
    # A = np.array([[1,1],
    #       [1, -1]])
    B = np.array([[w], [v]])
    # pdb.set_trace()
    x = np.dot(np.linalg.inv(A), B)
    # print(x)
    v_1 = 2*ROBOT_RADIUS*x[0]
    v_2 = -2*ROBOT_RADIUS*x[1]
    action = [v_1.item(), v_2.item()]
    """STUDENT CODE END"""
    return action
