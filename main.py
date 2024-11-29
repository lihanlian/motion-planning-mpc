import numpy as np
from controller import MPC
from config import *
from visualization import plot_traj
import pickle

def simulation(x_0, x_goal):
    state_traj = [x_0]
    control_traj = []

    x_k = x_0
    # Simulation loop
    for k in range(total_steps):
        # Calculate control input
        if method == 'mpc_dc':
            u_k = controller.mpc_dc(x_k, x_goal)
            print(f'{method} N={N} - timestep: {k} finished')
        else:
            u_k = controller.mpc_cbf(x_k, x_goal, gamma)
            print(f'{method} N={N} gamma={gamma} - timestep: {k} finished')
        
        control_traj.append(u_k)
        x_k = controller.rk4(x_k, u_k)
        x_k = x_k.full().flatten().tolist()
        state_traj.append(x_k)
    # Convert state and control trajectories to numpy arrays for plotting
    state_traj = np.array(state_traj)
    control_traj = np.array(control_traj)
    return state_traj, control_traj

if __name__ == '__main__':

    time = np.arange(0, T + N * dt, dt)
    # Choose one of methods for execution
    method = 'mpc_dc'
    method = 'mpc_cbf'
    if method == 'mpc_dc':
        file_name = f"./data/{method}_N{N}.pkl"
    else:
        file_name = f"./data/{method}_N{N}_gamma{gamma}.pkl"

    # Initialize controller and initial state
    controller = MPC(dt, N, v_min, v_max, w_min, w_max)
    robot_x0 = 0; robot_y0 = 0; robot_theta0 = 0
    x_0 = np.array([robot_x0, robot_y0, robot_theta0])
    x_goal = np.array([10.0, 0.0, 0.0])

    # Get simulation result
    state_traj, control_traj = simulation(x_0, x_goal)

    # Plot state and control trajectories
    plot_traj(state_mpc=state_traj, 
                        u_mpc=control_traj, time=time, method=method)

    # Store data for animation
    trajectories_ref = {
        'x_traj': state_traj[:, 0],
        'y_traj': state_traj[:, 1],
        'theta_traj': state_traj[:, 2]
    }
    with open(file_name, "wb") as file:
        pickle.dump(trajectories_ref, file)

