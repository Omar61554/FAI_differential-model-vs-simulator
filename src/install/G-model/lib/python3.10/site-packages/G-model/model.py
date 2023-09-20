import math
import matplotlib.pyplot as plt

def calculate_position_change(omega_r, omega_l, R, theta):
    delta_theta = R * (omega_r - omega_l) / (2.0 * RADIUS)
    delta_x = R * (omega_r + omega_l) / 2.0 * math.cos(theta + delta_theta / 2.0)
    delta_y = R * (omega_r + omega_l) / 2.0 * math.sin(theta + delta_theta / 2.0)
    return delta_x, delta_y, delta_theta

def plot_robot_trajectory(omega_r, omega_l, R, theta, start_x, start_y):
    # Lists to store x and y coordinates
    x_coords = [start_x]
    y_coords = [start_y]

    for i in range(len(omega_r)):
        # Calculate position and orientation change for each time step
        delta_x, delta_y, delta_theta = calculate_position_change(omega_r[i], omega_l[i], R, theta)
        
        # Update x and y coordinates
        x_coords.append(x_coords[-1] + delta_x)
        y_coords.append(y_coords[-1] + delta_y)
        
        # Update theta (orientation)
        theta += delta_theta

    # Plotting the trajectory
    plt.plot(x_coords, y_coords, '-o')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Trajectory')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    # Constants
    RADIUS = 0.15  # Radius of the wheels in meters
    THETA = 0.0  # Initial orientation angle in radians
    START_X = 0.0  # Initial x-coordinate
    START_Y = 3.0  # Initial y-coordinate
    
    # Wheel velocities for each time step
    time = [i for i in range(0, 41)]  # Time intervals from 0 to 40 seconds
    
    # Wheel velocities for each time interval
    omega_r = []
    omega_l = []
    
    for t in time:
        if t <= 10:
            omega_r.append(1.0)
            omega_l.append(1.0)
        elif t <= 20:
            omega_r.append(1.5)
            omega_l.append(1.0)
        elif t <= 30:
            omega_r.append(1.0)
            omega_l.append(1.5)
        else:
            break

    # Plot the robot trajectory
    plot_robot_trajectory(omega_r, omega_l, RADIUS, THETA, START_X, START_Y)