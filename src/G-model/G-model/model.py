#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class LocationSubscriber(Node):
    def __init__(self):
        super().__init__('location_subscriber')
        self.subscription = self.create_subscription(
            Vector3,
            '/location',
            self.location_callback,
            10
        )
        self.additional_x = []
        self.additional_y = []

    def location_callback(self, msg):
        x = msg.x
        y = msg.y
        self.additional_x.append(x)
        self.additional_y.append(y)
        plot_robot_trajectory(omega_r, omega_l, RADIUS, THETA, START_X, START_Y, self.additional_x, self.additional_y)

def calculate_position_change(omega_r, omega_l, R, theta):
    delta_theta = R * (omega_r - omega_l) / (2.0 * RADIUS)
    delta_x = R * (omega_r + omega_l) / 2.0 * math.cos(theta + delta_theta / 2.0)
    delta_y = R * (omega_r + omega_l) / 2.0 * math.sin(theta + delta_theta / 2.0)
    return delta_x, delta_y, delta_theta

def plot_robot_trajectory(omega_r, omega_l, R, theta, start_x, start_y, additional_x, additional_y):
    # Lists to store all x and y coordinates
    x_coords = [start_x]
    y_coords = [start_y]

    # Plotting the simulator trajectory
    plt.plot(additional_x, additional_y, '-o', label='Simulator', color='blue')

    for i in range(len(omega_r)):
        # Calculate position and orientation change for each time step
        delta_x, delta_y, delta_theta = calculate_position_change(omega_r[i], omega_l[i], R, theta)

        # Update x and y coordinates
        x_coords.append(x_coords[-1] + delta_x)
        y_coords.append(y_coords[-1] + delta_y)

        # Update theta (orientation)
        theta += delta_theta

    # Plotting the model trajectory
    plt.plot(x_coords, y_coords, '-o', label='Model', color='red')

    # Add a legend to the plot
    plt.legend()

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Trajectory')
    plt.grid(True)
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    location_subscriber = LocationSubscriber()
    rclpy.spin(location_subscriber)

    location_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Constants
    RADIUS = 0.32  # Radius of the wheels in meters
    THETA = 0.0  # Initial orientation angle in radians
    START_X = -1.75
  # Initial x-coordinate
    START_Y = -0.32  # Initial y-coordinate

    # Wheel velocities for each time step
    time = [i for i in range(0, 16)]  # Time intervals from 0 to 15 seconds

    # Wheel velocities for each time interval
    omega_r = []
    omega_l = []

    for t in time:
        if t <= 5:
            omega_r.append(1.0)  # Modify the wheel velocity for the right wheel
            omega_l.append(1.0)  # Modify the wheel velocity for the left wheel
        elif t <= 10:
            omega_r.append(1.0)  # Modify the wheel velocity for the right wheel
            omega_l.append(0.5)  # Modify the wheel velocity for the left wheel
        elif t <= 15:
            omega_r.append(0.5)  # Modify the wheel velocity for the right wheel
            omega_l.append(1.0)  # Modify the wheel velocity for the left wheel
        else:
            break
        

    main()