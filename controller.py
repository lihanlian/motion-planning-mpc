import casadi as ca
import numpy as np
from config import safe_distance1, safe_distance2, obstacle_x1, obstacle_y1, obstacle_x2, obstacle_y2

class MPC:
    def __init__(self, dt, N, v_min, v_max, w_min, w_max):
        # System parameters
        self.dt = dt
        self.N = N
        self.v_min = v_min
        self.v_max = v_max
        self.w_min = w_min
        self.w_max = w_max

    def dynamics(self, x, u):
        # Unicycle model dynamics
        theta = x[2]
        V, omega = u[0], u[1]
        x_dot = V * ca.cos(theta)
        y_dot = V * ca.sin(theta)
        theta_dot = omega
        return ca.vertcat(x_dot, y_dot, theta_dot)

    def rk4(self, x, u):
        # RK4 integration step for dynamics
        k1 = self.dynamics(x, u)
        k2 = self.dynamics(x + self.dt / 2 * k1, u)
        k3 = self.dynamics(x + self.dt / 2 * k2, u)
        k4 = self.dynamics(x + self.dt * k3, u)
        x_next = x + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_next

    def mpc_dc(self, current_state, goal_state):
        # Define control decision variables (U only, for shooting method)
        U = ca.MX.sym("U", 2, self.N)

        # Objective and cost weights
        obj = 0
        g = []  # List of constraints
        Q = np.diag([100, 100, 10])  # State cost weights
        R = np.diag([0.1, 0.1])   # Control cost weights
        H = 30*Q

        # Bounds for states (to be enforced as constraints)
        state_min = [0, -3.5, -3.14]
        state_max = [10, 3.5, 3.14]

        # Constraints for control inputs only
        lbu = [self.v_min, self.w_min] * self.N
        ubu = [self.v_max, self.w_max] * self.N

        # Initial state
        x_k = current_state
        x_k = ca.reshape(x_k, -1, 1)
        # Build the objective by simulating forward using rk4 and accumulating cost
        for k in range(self.N):
            # Get control input at step k
            u_k = U[:, k]
            goal_state = ca.reshape(goal_state, -1, 1)

            x_rob, y_rob = x_k[0], x_k[1]

            # Compute the cost
            obj += ca.mtimes((x_k - goal_state).T, Q @ (x_k - goal_state)) + ca.mtimes(u_k.T, R @ u_k)

            # Forward propagate the state using rk4 with the control input
            x_k = self.rk4(x_k, u_k)
            # Enforce state constraints
            g.append(x_k - state_min)  # x_next >= state_min
            g.append(state_max - x_k)  # x_next <= state_max
            # Obstacle avoidance constraint
            dist_to_obstacle1 = ca.sqrt((x_rob - obstacle_x1)**2 +
                                    (y_rob - obstacle_y1)**2)
            dist_to_obstacle2 = ca.sqrt((x_rob - obstacle_x2)**2 +
                                    (y_rob - obstacle_y2)**2)
            # MPC-DC Constraints
            g.append(dist_to_obstacle1 - safe_distance1)
            g.append(dist_to_obstacle2 - safe_distance2)
            
        # Add terminal cost
        obj += ca.mtimes((x_k - goal_state).T, H @ (x_k - goal_state))
        # Combine all constraints
        g = ca.vertcat(*g)
        # Define the optimization problem
        nlp = {'f': obj, 'x': ca.reshape(U, -1, 1), 'g':g}
        opts = {'ipopt.print_level':1,'print_time': 0}
        solver = ca.nlpsol('S', 'ipopt', nlp, opts)

        # Solve the optimization problem
        sol = solver(lbx=lbu, ubx=ubu, lbg=0, ubg=ca.inf)
        u_opt_traj = sol["x"].full()
        u = u_opt_traj[:2]
        return u  # Return only the first control action
    
    def mpc_cbf(self, current_state, goal_state, gamma):
        # Define control decision variables (U only, for shooting method)
        U = ca.MX.sym("U", 2, self.N)

        # Objective and cost weights
        obj = 0
        g = []  # List of constraints
        Q = np.diag([100, 100, 10])  # State cost weights
        R = np.diag([0.1, 0.1])   # Control cost weights
        H = 30*Q

        # Bounds for states (to be enforced as constraints)
        state_min = [0, -3.5, -3.14]
        state_max = [10, 3.5, 3.14]

        # Obstacle properties
        # Constraints for control inputs only
        lbu = [self.v_min, self.w_min] * self.N
        ubu = [self.v_max, self.w_max] * self.N

        # Initial state
        x_k = current_state
        x_k = ca.reshape(x_k, -1, 1)
        # Build the objective by simulating forward using rk4 and accumulating cost
        for k in range(self.N):
            # Get control input at step k
            u_k = U[:, k]
            goal_state = ca.reshape(goal_state, -1, 1)

            x_rob, y_rob, theta_rob = x_k[0], x_k[1], x_k[2]
            v_rob = u_k[0]

            # Compute the cost
            obj += ca.mtimes((x_k - goal_state).T, Q @ (x_k - goal_state)) + ca.mtimes(u_k.T, R @ u_k)

            # Forward propagate the state using rk4 with the control input
            x_k = self.rk4(x_k, u_k)
            # Enforce state constraints
            g.append(x_k - state_min)  # x_next >= state_min
            g.append(state_max - x_k)  # x_next <= state_max
            # Obstacle avoidance constraint
            h1 = ca.sqrt((x_rob - obstacle_x1)**2 +
                                    (y_rob - obstacle_y1)**2) - safe_distance1
            h2 = ca.sqrt((x_rob - obstacle_x2)**2 +
                                    (y_rob -obstacle_y2)**2) - safe_distance2
            
            Lghu1 = (2 * x_rob - 2 * obstacle_x1) * (v_rob * ca.cos(theta_rob)) + \
                 (2 * y_rob - 2 * obstacle_y1) * (v_rob * ca.sin(theta_rob))
            Lghu2 = (2 * x_rob - 2 * obstacle_x2) * (v_rob * ca.cos(theta_rob)) + \
                 (2 * y_rob - 2 * obstacle_y2) * (v_rob * ca.sin(theta_rob))
            # MPC-CBF Constraints
            g.append(Lghu1 + gamma*h1)
            g.append(Lghu2 + gamma*h2)
            
        # Add terminal cost
        obj += ca.mtimes((x_k - goal_state).T, H @ (x_k - goal_state))
        # Combine all constraints
        g = ca.vertcat(*g)
        # Define the optimization problem
        nlp = {'f': obj, 'x': ca.reshape(U, -1, 1), 'g':g}
        opts = {'ipopt.print_level':1,'print_time': 0}
        solver = ca.nlpsol('S', 'ipopt', nlp, opts)

        # Solve the optimization problem
        sol = solver(lbx=lbu, ubx=ubu, lbg=0, ubg=ca.inf)
        u_opt_traj = sol["x"].full()
        u = u_opt_traj[:2]
        return u  # Return only the first control action
