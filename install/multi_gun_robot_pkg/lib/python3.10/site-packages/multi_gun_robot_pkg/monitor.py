import sys, yaml, time, os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped

from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer, QTime
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QCheckBox, QGridLayout,
    QTabWidget, QTableWidget, QTableWidgetItem, QSizePolicy, QScrollArea
)
from PIL import Image, ImageDraw


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
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.node.signal1.connect(self.update_tb1_position)
        self.node.signal2.connect(self.update_tb2_position)

        self.map_size = (400, 300)
        self.dot_size = 4

        # map 가져오기
        cur_directory = os.getcwd()
        get_map_pgm = os.path.join(cur_directory, 'src', 'multi_gun_robot', 'map', 'map_final.pgm')
        get_map_yaml = os.path.join(cur_directory, 'src', 'multi_gun_robot', 'map', 'map_final.yaml')
        image = Image.open(get_map_pgm)
        self.width, self.height = image.size
        self.rgb_image = image.convert('RGB')

        with open(get_map_yaml, 'r') as file:
            data = yaml.safe_load(file)

        self.resolution = data['resolution']
        self.map_x = -data['origin'][0]
        self.map_y = +data['origin'][1] + self.height * self.resolution

        self.tb1_x = self.map_x
        self.tb1_y = self.map_y

        self.tb2_x = self.map_x
        self.tb2_y = self.map_y

        # GUI setup
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Kill Machine Monitor")

        # Main central widget layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Title Label
        self.title_label = QLabel('AMCL Pose Map')
        self.title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.title_label)

        # # Horizontal layout for the two real-time images (tb1 and tb2)
        # monitor_layer = QHBoxLayout()  # No need to pass central_widget here

        # # tb1 이미지
        # self.real_time_image_1 = QLabel()  
        # self.real_time_image_1.setFixedSize(500, 500)
        # monitor_layer.addWidget(self.real_time_image_1)

        # # tb2 이미지
        # self.real_time_image_2 = QLabel() 
        # self.real_time_image_2.setFixedSize(500, 500)
        # monitor_layer.addWidget(self.real_time_image_2)
        
        # layout.addLayout(monitor_layer)  # Add monitor_layer to the main layout

        # Control layout (for Map + 버튼)
        control_layout = QVBoxLayout()  # No need to pass central_widget here either

        # Map Display Label
        self.label = QLabel()
        control_layout.addWidget(self.label)

        # Add control_layout to the main layout
        layout.addLayout(control_layout)

        # Render the initial map
        self.render_initial_map()


    def render_initial_map(self):
        image_resized = self.rgb_image.resize(self.map_size) 
        pil_image = image_resized.convert('RGBA')  
        data = pil_image.tobytes("raw", "RGBA") 
        qimage = QImage(data, *self.map_size, QImage.Format_RGBA8888) 
        pixmap = QPixmap.fromImage(qimage)  
        self.label.setPixmap(pixmap)  



    def update_tb1_position(self, message):
        odom_x = message[0]
        odom_y = message[1]
        self.tb1_x = self.map_x + odom_x
        self.tb1_y = self.map_y + (odom_y * -1)

        print(f"tb1 - x={self.tb1_x}, y={self.tb1_y}")
        self.update_positions()

    def update_tb2_position(self, message):
        odom_x = message[0]
        odom_y = message[1]
        self.tb2_x = self.map_x + odom_x
        self.tb2_y = self.map_y + (odom_y * -1)

        print(f"tb2 - x={self.tb2_x}, y={self.tb2_y}")
        self.update_positions()

    def update_positions(self):
        image_copy = self.rgb_image.copy()
        draw = ImageDraw.Draw(image_copy)

        # tb1 (blue) 원 그리기
        draw.ellipse((
            self.tb1_x / self.resolution - self.dot_size,
            self.tb1_y / self.resolution - self.dot_size,
            self.tb1_x / self.resolution + self.dot_size,
            self.tb1_y / self.resolution + self.dot_size),
            fill='blue'
        )

        # tb2 (red) 원 그리기
        draw.ellipse((
            self.tb2_x / self.resolution - self.dot_size,
            self.tb2_y / self.resolution - self.dot_size,
            self.tb2_x / self.resolution + self.dot_size,
            self.tb2_y / self.resolution + self.dot_size),
            fill='red'
        )

        # 이미지를 회전, 크기 조정, 변환하여 표시
        image_resized = image_copy.resize(self.map_size)
        pil_image = image_resized.convert('RGBA')
        data = pil_image.tobytes("raw", "RGBA")
        qimage = QImage(data, *self.map_size, QImage.Format_RGBA8888)
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
