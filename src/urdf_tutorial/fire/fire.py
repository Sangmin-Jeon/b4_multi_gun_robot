import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class GunController(Node):
    def __init__(self):
        super().__init__('gun_controller')
        self.publisher_ = self.create_publisher(Empty, '/gun/fire', 10)
        self.timer = self.create_timer(1.0, self.fire_gun)  # 1초 간격으로 발사

    def fire_gun(self):
        msg = Empty()
        self.publisher_.publish(msg)
        self.get_logger().info('Fire!')

def main(args=None):
    rclpy.init(args=args)
    node = GunController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()