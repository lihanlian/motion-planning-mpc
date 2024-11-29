dt = 0.02
N = 25
gamma = 0.8
v_min = -2
v_max = 2
w_min = -1
w_max = 1
# For MPC-DC, it takes around 6 sec to reach the goal
# For MPC-CBF, it takes around 20 second to reach the goal (depends on gamma)
T = 18
total_steps = int(T/dt)
# minimum distance between centers of robot and obstalce
# r_obstacle + r_robot + buffer
safe_distance1 = 1.4 # 1 + 0.25 + 0.15
safe_distance2 = 1.8 # 1.4 + 0.25 + 0.15
obstacle_x1, obstacle_y1 = 2, 1
obstacle_x2, obstacle_y2 = 7, -1