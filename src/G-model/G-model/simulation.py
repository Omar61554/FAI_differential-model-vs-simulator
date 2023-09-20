#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Coppelia(Node):
    def __init__(self):
        super().__init__('coppelia')
        self.pub1 = self.create_publisher(Float32, 'l_w', 10)
        self.pub2 = self.create_publisher(Float32, 'r_w', 10)   
        self.l_w = Float32()
        self.r_w = Float32()
    
        # Wheel velocities for each time step
        time_intervals = [i for i in range(0, 16)]  # Time intervals from 0 to 40 seconds

        # Wheel velocities for each time interval
        omega_r = []
        omega_l = []

        for t in time_intervals:
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

        time.sleep(1.2)  # Introduce a delay of 1 seconds    

        # Publish the wheel velocities for each time interval with a delay
        for i in range(len(omega_r)):
            self.l_w.data = omega_l[i]
            self.r_w.data = omega_r[i]
            self.pub1.publish(self.l_w)
            self.pub2.publish(self.r_w)
            self.get_logger().info('Publishing: "%f, %f"' % (omega_l[i], omega_r[i]))
            time.sleep(1.4)  # Introduce a delay of 1 second

            if i == len(omega_r) - 1:
                # Stop the robot after the last time interval
                self.l_w.data = 0.0
                self.r_w.data = 0.0
                self.pub1.publish(self.l_w)
                self.pub2.publish(self.r_w)
                self.get_logger().info('Stopping the robot')

def main(args=None):
    rclpy.init(args=args)

    coppelia = Coppelia()

    rclpy.spin(coppelia)

    coppelia.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()