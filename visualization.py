import numpy as np
import matplotlib.pyplot as plt
from config import *
from matplotlib.animation import FuncAnimation
import pickle

def plot_traj(state_mpc, u_mpc, time, method):

    x_mpc = state_mpc[:, 0]
    y_mpc = state_mpc[:, 1]
    theta_mpc = state_mpc[:,2]

    v_mpc = u_mpc[:,0]
    w_mpc = u_mpc[:,1]

    # Plot reference trajectory
    fig, axs = plt.subplots(2, 2, figsize=(8, 6))
    len_state_mpc = len(x_mpc)
    len_u_mpc = len(v_mpc)

    axs[0,0].plot(time[:len_state_mpc], x_mpc, linewidth=5, label="x_traj")
    axs[0,0].set_xlabel("Time (s)",fontsize=16)
    axs[0,0].set_ylabel("X (m)",fontsize=16)
    axs[0,0].legend(fontsize=12)
    axs[0,0].grid(True)

    axs[0, 1].plot(time[:len_state_mpc], y_mpc, linewidth=5, label="y_traj")
    axs[0, 1].set_xlabel("Time (s)",fontsize=16)
    axs[0, 1].set_ylabel("Y (m)",fontsize=16)
    axs[0, 1].legend(fontsize=12)
    axs[0, 1].grid(True)

    axs[1,0].plot(time[:len_state_mpc], theta_mpc, linewidth=5, label="theta_traj")
    axs[1,0].set_xlabel("Time (s)",fontsize=16)
    axs[1,0].set_ylabel("theta",fontsize=16)
    axs[1,0].legend(fontsize=12)
    axs[1,0].grid(True)

    axs[1,1].plot(x_mpc, y_mpc, linewidth=5, label="xy_traj")
    axs[1,1].set_xlabel("X (m)",fontsize=16)
    axs[1,1].set_ylabel("Y (m)",fontsize=16)
    axs[1,1].legend(fontsize=12)
    axs[1,1].grid(True)

    fig.suptitle("State Trajectories (x, y, theta)",fontsize=20)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    
    # Save the figure to the folder
    dir_state_tracking = f'./figs/{method}_N{N}_state_trajectory.png'
    plt.savefig(dir_state_tracking, dpi=300)
    plt.close()  # Close the figure to free memory

    # Plot reference trajectory
    fig, axs = plt.subplots(figsize=(8, 6))

    axs.plot(time[:len_u_mpc], v_mpc, linewidth=5, label="v_mpc")
    axs.plot(time[:len_u_mpc], w_mpc, linewidth=5, label="w_mpc")
    axs.set_xlabel("Time (s)",fontsize=20)
    axs.set_ylabel("Control Input",fontsize=20)
    axs.legend(fontsize=16)
    axs.grid(True)

    dir_control_input = f'./figs/{method}_N{N}_control_trajectory.png'
    plt.savefig(dir_control_input, dpi=300)
    plt.close()

    print(f"Figure saved to {dir_state_tracking}, {dir_control_input}")

def animate(gif_name):

    x_mpc = data['x_traj']
    y_mpc = data['y_traj']
    theta_mpc = data['theta_traj']

    # Create a figure for the animation
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_xlim(min(x_mpc) - 1, max(x_mpc) + 1)
    ax.set_ylim(min(y_mpc) - 1, max(y_mpc) + 1)
    ax.set_aspect('equal')  # Ensure equal scale for x and y axes
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")

    # Add a dynamic trajectory line
    robot_path, = ax.plot([], [], linestyle="--", color="saddlebrown", label="Traveled Path", linewidth=2)

    # Draw the robot components
    start = plt.Circle((0, 0), 0.1, color='green', fill=True, linewidth=2)  # Robot circular base
    goal = plt.Circle((10, 0), 0.1, color='red', fill=True, linewidth=2)  # Robot circular base

    # robot radius 0.25
    robot_body = plt.Circle((0, 0), 0.25, color='cyan', fill=True, linewidth=2)  # Robot circular base
    wheel_width = 0.1
    wheel_height = 0.2

    wheel1 = plt.Rectangle((-wheel_height / 2, -wheel_width / 2), wheel_height, wheel_width, color='gold')  # Left wheel
    wheel2 = plt.Rectangle((-wheel_height / 2, -wheel_width / 2), wheel_height, wheel_width, color='gold')  # Right wheel

    # Add the robot components to the plot
    ax.add_patch(start)
    ax.add_patch(goal)
    ax.add_patch(robot_body)
    ax.add_patch(wheel1)
    ax.add_patch(wheel2)

    # Add the black solid circles to represent obstacles
    circle1 = plt.Circle((2, 1), 1, color='gray', fill=True, label="Obstacle 1") # radius = 1
    circle2 = plt.Circle((7, -1), np.sqrt(2), color='slategray', fill=True, label="Obstacle 2") # radius = 1.4

    # Add the circles to the plot
    ax.add_patch(circle1)
    ax.add_patch(circle2)

    # Set y-axis limits
    ax.set_ylim(-4, 4)
    # Fill regions for y = 3.0 to 3.5 and y = -3.0 to -3.5
    ax.fill_between(np.linspace(min(x_mpc) - 1, max(x_mpc) + 1, 100), 3.5, 4, color='black', label="Restricted Area")
    ax.fill_between(np.linspace(min(x_mpc) - 1, max(x_mpc) + 1, 100), -4, -3.5, color='black')
    ax.legend(loc="upper right")

    # Animation function
    def update(frame):
        # Update the robot's position and orientation
        x, y, theta = x_mpc[frame], y_mpc[frame], theta_mpc[frame]
        
        # Update robot body position
        robot_body.center = (x, y)
        
        # Wheel offsets relative to the robot's center
        wheel_offset = 0.3  # Distance of wheels from center
        
        # Update left wheel position
        wheel1_center_x = x - wheel_offset * np.sin(theta)
        wheel1_center_y = y + wheel_offset * np.cos(theta)
        wheel1.set_xy((
            wheel1_center_x - wheel_height / 2 * np.cos(theta) + wheel_width / 2 * np.sin(theta),
            wheel1_center_y - wheel_height / 2 * np.sin(theta) - wheel_width / 2 * np.cos(theta),
        ))
        wheel1.angle = np.degrees(theta)
        
        # Update right wheel position
        wheel2_center_x = x + wheel_offset * np.sin(theta)
        wheel2_center_y = y - wheel_offset * np.cos(theta)
        wheel2.set_xy((
            wheel2_center_x - wheel_height / 2 * np.cos(theta) + wheel_width / 2 * np.sin(theta),
            wheel2_center_y - wheel_height / 2 * np.sin(theta) - wheel_width / 2 * np.cos(theta),
        ))
        wheel2.angle = np.degrees(theta)
        
        # Update the dynamic trajectory
        robot_path.set_data(x_mpc[:frame], y_mpc[:frame])  # Update only up to the current frame

        return robot_body, wheel1, wheel2, robot_path

    # Create the animation
    anim = FuncAnimation(fig, update, frames=len(x_mpc), interval=20, blit=True)
    ax.legend(loc="upper right")
    anim.save(gif_name, writer="pillow", fps=50)
    # Show the animation
    plt.show()

if __name__ == '__main__':
    # Choose one of method for visulization
    method = 'mpc_dc'
    method = 'mpc_cbf'

    if method == 'mpc_dc':
        data_name = f"./data/{method}_N{N}.pkl"
        gif_name = f"./animation/{method}_N{N}.gif"
    else:
        data_name = f"./data/{method}_N{N}_gamma{gamma}.pkl"
        gif_name = f"./animation/{method}_N{N}_gamma{gamma}.gif"
    with open(data_name, "rb") as file:
        data = pickle.load(file)

    animate(gif_name)