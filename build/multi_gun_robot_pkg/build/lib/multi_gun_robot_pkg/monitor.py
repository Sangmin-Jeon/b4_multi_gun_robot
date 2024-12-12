import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QPixmap, QImage
from PIL import Image, ImageDraw
import yaml


class RosNode(Node, QObject):
    signal1 = pyqtSignal(list)
    signal2 = pyqtSignal(list)

    def __init__(self, node_name='ros_subscriber_node'):
        Node.__init__(self, node_name)
        QObject.__init__(self)

        self.amcl_pose_x = 0
        self.amcl_pose_y = 0
        self.dot_size = 2
        
        callback_group = ReentrantCallbackGroup()

        # AMCL Pose Subscription
        self.sub_tb1_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/tb1/amcl_pose', 
            self.sub_tb1_amcl_pose_callback, 
            10, 
            callback_group=callback_group
        )

        self.sub_tb2_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/tb2/amcl_pose', 
            self.sub_tb2_amcl_pose_callback, 
            10, 
            callback_group=callback_group
        )

    def sub_tb1_amcl_pose_callback(self, msg):
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        self.get_logger().info(f'Received tb1 amcl_pose x: {self.amcl_pose_x}, y: {self.amcl_pose_y}')
        self.signal1.emit([self.amcl_pose_x, self.amcl_pose_y])

    def sub_tb2_amcl_pose_callback(self, msg):
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        self.get_logger().info(f'Received tb2 amcl_pose x: {self.amcl_pose_x}, y: {self.amcl_pose_y}')
        self.signal2.emit([self.amcl_pose_x, self.amcl_pose_y])


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.ros_node.signal1.connect(self.update_position)

        self.resize = (367, 290)
        self.dot_size = 2

        # Load map
        image = Image.open('/home/rokey/map_fin.pgm')
        self.width, self.height = image.size
        self.image_rgb = image.convert('RGB')

        # Load YAML metadata
        with open('/home/rokey/map_fin.yaml', 'r') as file:
            data = yaml.safe_load(file)

        self.resolution = data['resolution']
        self.map_x = -data['origin'][0]
        self.map_y = +data['origin'][1] + self.height * self.resolution

        self.x = self.map_x
        self.y = self.map_y

        # GUI setup
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("AMCL Pose Monitor")
        self.label = QLabel(self)
        self.label.setGeometry(20, 20, 320, 320)

    def update_position(self, message):
        odom_x = message[0]
        odom_y = message[1]
        self.x = self.map_x + odom_x
        self.y = self.map_y + odom_y

        print(f"x={self.x}, y={self.y}")
        self.update_map()

    def update_map(self):
        image_copy = self.image_rgb.copy()
        draw = ImageDraw.Draw(image_copy)
        draw.ellipse((
            self.x / self.resolution - self.dot_size,
            self.y / self.resolution - self.dot_size,
            self.x / self.resolution + self.dot_size,
            self.y / self.resolution + self.dot_size),
            fill='red'
        )

        image_rotated = image_copy.rotate(90, expand=True)
        image_resized = image_rotated.resize(self.resize)
        pil_image = image_resized.convert('RGBA')
        data = pil_image.tobytes("raw", "RGBA")
        qimage = QImage(data, *self.resize, QImage.Format_RGBA8888)
        pixmap = QPixmap.fromImage(qimage)
        self.label.setPixmap(pixmap)


def main():
    # Initialize ROS
    rclpy.init()

    # Create ROS node
    ros_node = RosNode()

    # Create QApplication
    app = QApplication(sys.argv)
    gui = MainWindow(ros_node)
    gui.show()

    # ROS Executor with MultiThreading
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    ros_thread = QTimer()
    ros_thread.timeout.connect(executor.spin_once)
    ros_thread.start(10)  # Process ROS callbacks every 10 ms

    try:
        sys.exit(app.exec_())
    finally:
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
