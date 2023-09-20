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

        while rclpy.ok():
            self.shape = input("Enter shape: ")
            if self.shape == "forward":
                self.l_w.data = 1.0
                self.r_w.data = 1.0
                self.pub1.publish(self.l_w)
                self.pub2.publish(self.r_w)

            elif self.shape == "backward":
                self.l_w.data = -1.0
                self.r_w.data = -1.0
                self.pub1.publish(self.l_w)
                self.pub2.publish(self.r_w)

            elif self.shape == "left":
                self.l_w.data = -1.0
                self.r_w.data = 1.0
                self.pub1.publish(self.l_w)
                self.pub2.publish(self.r_w)

            elif self.shape == "right":
                self.l_w.data = 1.0
                self.r_w.data = -1.0
                self.pub1.publish(self.l_w)
                self.pub2.publish(self.r_w)

            elif self.shape == "stop":
                self.l_w.data = 0.0
                self.r_w.data = 0.0
                self.pub1.publish(self.l_w)
                self.pub2.publish(self.r_w)

            elif self.shape == "exit":
                break

            else:
                print("Invalid shape")

            self.get_logger().info('Publishing: "%s"' % self.shape)
    
            

def main(args=None):
    rclpy.init(args=args)

    coppelia = Coppelia()

    rclpy.spin(coppelia)

    coppelia.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
       