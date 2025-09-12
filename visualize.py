import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

def visualize_trajectory():
    """
    Reads trajectory data from CSV files and generates plots similar to the web UI.
    """
    try:
        # Load data
        waypoints = pd.read_csv("waypoints.csv")
        control_points = pd.read_csv("control_points.csv")
        trajectory = pd.read_csv("trajectory.csv")
    except FileNotFoundError as e:
        print(f"Error: {e}. Please run the C++ test program first to generate the CSV files.")
        return

    # Calculate velocity and acceleration magnitudes
    trajectory['vel_mag'] = np.sqrt(trajectory['vx']**2 + trajectory['vy']**2)
    trajectory['acc_mag'] = np.sqrt(trajectory['ax']**2 + trajectory['ay']**2)

    # Create plots
    plt.style.use('seaborn-v0_8-whitegrid')
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2)

    # Trajectory Plot (Top, spans both columns)
    ax_traj = fig.add_subplot(gs[0, :])
    ax_traj.plot(trajectory['px'], trajectory['py'], label='B-spline Trajectory', color='indigo', linewidth=2.5)
    ax_traj.plot(control_points['x'], control_points['y'], 'o--', label='Control Polygon', color='mediumseagreen', markersize=6, alpha=0.7)
    ax_traj.plot(waypoints['x'], waypoints['y'], 'o', label='Waypoints', color='crimson', markersize=9)
    ax_traj.set_title('Trajectory Visualization', fontsize=16, fontweight='bold')
    ax_traj.set_xlabel('X coordinate')
    ax_traj.set_ylabel('Y coordinate')
    ax_traj.legend()
    ax_traj.axis('equal')

    # Velocity Plot (Bottom Left)
    ax_vel = fig.add_subplot(gs[1, 0])
    ax_vel.plot(trajectory['t'], trajectory['vel_mag'], label='Velocity Magnitude', color='dodgerblue', linewidth=2)
    ax_vel.set_title('Velocity vs. Time', fontsize=14)
    ax_vel.set_xlabel('Time (s)')
    ax_vel.set_ylabel('Velocity Magnitude')
    ax_vel.legend()

    # Acceleration Plot (Bottom Right)
    ax_acc = fig.add_subplot(gs[1, 1])
    ax_acc.plot(trajectory['t'], trajectory['acc_mag'], label='Acceleration Magnitude', color='darkorange', linewidth=2)
    ax_acc.set_title('Acceleration vs. Time', fontsize=14)
    ax_acc.set_xlabel('Time (s)')
    ax_acc.set_ylabel('Acceleration Magnitude')
    ax_acc.legend()

    fig.tight_layout(pad=3.0)
    plt.savefig("trajectory_visualization.png")
    print("Saved visualization to trajectory_visualization.png")
    plt.show()


if __name__ == '__main__':
    visualize_trajectory()
