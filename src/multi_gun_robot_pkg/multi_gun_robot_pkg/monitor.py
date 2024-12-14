import sys, yaml, time, os
import numpy as np
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image as TopicImage
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer, QTime
from PyQt5.QtGui import QPixmap, QImage, QPalette, QColor
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QCheckBox, QGridLayout,
    QTabWidget, QTableWidget, QTableWidgetItem, QSizePolicy, QScrollArea
)
from PIL import Image, ImageDraw

class RosNode(Node, QObject):
    signal1 = pyqtSignal(list)
    signal2 = pyqtSignal(list)

    tb1_img_sig = pyqtSignal(QImage)
    tb2_img_sig = pyqtSignal(QImage)

    def __init__(self, node_name='RosNode'):
        Node.__init__(self, node_name)
        QObject.__init__(self)

        callbackGroup = ReentrantCallbackGroup()

        self.tb1_waypoints = None
        self.tb2_waypoints = None
        self.is_detect = False

        # AMCL Pose Subscription
        self.sub_tb1_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/tb1/amcl_pose', 
            self.sub_tb1_amcl_pose_callback, 
            10, 
            callback_group=callbackGroup
        )

        self.sub_tb2_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/tb2/amcl_pose', 
            self.sub_tb2_amcl_pose_callback, 
            10, 
            callback_group=callbackGroup
        )
        
        self.sub_tb1_camera_img = self.create_subscription(
            TopicImage, 
            '/tb1/camera/image_raw', 
            self.sub_tb1_camera_img_callback, 
            10, 
            callback_group=callbackGroup
        )

        self.sub_tb2_camera_img = self.create_subscription(
            TopicImage, 
            '/tb2/camera/image_raw', 
            self.sub_tb2_camera_img_callback, 
            10, 
            callback_group=callbackGroup
        )

        self._action_tb1_client = ActionClient(self, NavigateToPose, '/tb1/navigate_to_pose')
        self._action_rb2_client = ActionClient(self, NavigateToPose, '/tb2/navigate_to_pose')

        self.current_tb1_waypoint_index = 0  # 현재 웨이포인트 인덱스
        self.current_tb2_waypoint_index = 0

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

    def sub_tb2_amcl_pose_callback(self, msg):
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        self.get_logger().info(f'Received tb2 amcl_pose x: {self.amcl_pose_x}, y: {self.amcl_pose_y}')
        self.signal2.emit([self.amcl_pose_x, self.amcl_pose_y])

    def sub_tb1_camera_img_callback(self, msg):
        try:
            # 이미지 포맷 설정
            if msg.encoding == "mono8":
                format = QImage.Format_Grayscale8
            elif msg.encoding == "bgr8":
                # BGR 포맷을 사용하려면 RGB로 변환 (Qt에서 BGR 포맷은 지원하지 않음)
                format = QImage.Format_RGB888
            else:
                self.get_logger().error(f"Image pixel format {msg.encoding} not supported!")
                return

            # QImage로 변환
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

            # QImage 객체 생성
            qimg = QImage(img_data.data, msg.width, msg.height, msg.width * 3, format)

            # BGR에서 RGB로 채널 순서 바꾸기
            if msg.encoding == "bgr8":
                qimg = qimg.rgbSwapped()

            # 유효한 이미지인지 확인
            if qimg.isNull():
                self.get_logger().error("Failed to read image!")
                return
            
            # 이미지를 QLabel 등에 표시하거나 추가 처리
            self.tb1_img_sig.emit(qimg)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")


    def sub_tb2_camera_img_callback(self, msg):
        try:
            # 이미지 포맷 설정
            if msg.encoding == "mono8":
                format = QImage.Format_Grayscale8
            elif msg.encoding == "bgr8":
                # BGR 포맷을 사용하려면 RGB로 변환 (Qt에서 BGR 포맷은 지원하지 않음)
                format = QImage.Format_RGB888
            else:
                self.get_logger().error(f"Image pixel format {msg.encoding} not supported!")
                return

            # QImage로 변환
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

            # QImage 객체 생성
            qimg = QImage(img_data.data, msg.width, msg.height, msg.width * 3, format)

            # BGR에서 RGB로 채널 순서 바꾸기
            if msg.encoding == "bgr8":
                qimg = qimg.rgbSwapped()

            # 유효한 이미지인지 확인
            if qimg.isNull():
                self.get_logger().error("Failed to read image!")
                return
            
            # 이미지를 QLabel 등에 표시하거나 추가 처리
            self.tb2_img_sig.emit(qimg)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")


    def move_to_tb1_patrol(self):
        if self.tb1_waypoints is None:
            return
        
        if self.is_detect:
            self.current_tb1_waypoint_index = 0 
            return

        # 모든 waypoint를 순찰한 경우
        if self.current_tb1_waypoint_index >= len(self.tb1_waypoints):
            self.current_tb1_waypoint_index = 0 
            self.get_logger().info('All waypoints have been reached.')
            # TODO: 순찰을 반복할지, 중단할지 결정하는 로직 추가
            self.move_to_tb1_patrol()
            return

        # 경로 이동
        _waypoint = self.tb1_waypoints[self.current_tb1_waypoint_index]
        x, y, z, orientation_x, orientation_y, orientation_z, orientation_w = _waypoint
        self.send_tb1_goal(x, y, z, orientation_x, orientation_y, orientation_z, orientation_w)



    def send_tb1_goal(self, x, y, z=0.0, orientation_x=0.0, orientation_y=0.0, orientation_z=0.0, orientation_w=1.0):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.x = orientation_x
        goal_msg.pose.pose.orientation.y = orientation_y
        goal_msg.pose.pose.orientation.z = orientation_z
        goal_msg.pose.pose.orientation.w = orientation_w

        self.get_logger().info(f'Sending goal to robot: x={x}, y={y}, z={z}, orientation=({orientation_x}, {orientation_y}, {orientation_z}, {orientation_w})')
        
        self._action_tb1_client.wait_for_server()
        send_goal_future = self._action_tb1_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_tb1_response_callback)

    def goal_tb1_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return

        self.get_logger().info("Goal accepted!")
        goal_handle.get_result_async().add_done_callback(self.result_tb1_callback)

    def result_tb1_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded! Moving to next waypoint.")
            self.current_tb1_waypoint_index += 1
            self.move_to_tb1_patrol()
        else:
            self.get_logger().info(f"Goal failed with status: {status}")


    def move_to_tb2_patrol(self):
        if self.tb2_waypoints is None:
            return
        
        if self.is_detect:
            self.current_tb2_waypoint_index = 0 
            return

        # 모든 waypoint를 순찰한 경우
        if self.current_tb2_waypoint_index >= len(self.tb2_waypoints):
            self.current_tb2_waypoint_index = 0 
            self.get_logger().info('All waypoints have been reached.')
            # TODO: 순찰을 반복할지, 중단할지 결정하는 로직 추가
            self.move_to_tb2_patrol()
            return

        # 경로 이동
        _waypoint = self.tb2_waypoints[self.current_tb2_waypoint_index]
        x, y, z, orientation_x, orientation_y, orientation_z, orientation_w = _waypoint
        self.send_tb2_goal(x, y, z, orientation_x, orientation_y, orientation_z, orientation_w)

    def send_tb2_goal(self, x, y, z=0.0, orientation_x=0.0, orientation_y=0.0, orientation_z=0.0, orientation_w=1.0):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.x = orientation_x
        goal_msg.pose.pose.orientation.y = orientation_y
        goal_msg.pose.pose.orientation.z = orientation_z
        goal_msg.pose.pose.orientation.w = orientation_w

        self.get_logger().info(f'Sending goal to robot: x={x}, y={y}, z={z}, orientation=({orientation_x}, {orientation_y}, {orientation_z}, {orientation_w})')
        
        self._action_rb2_client.wait_for_server()
        send_goal_future = self._action_rb2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_tb2_response_callback)

    def goal_tb2_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return

        self.get_logger().info("Goal accepted!")
        goal_handle.get_result_async().add_done_callback(self.result_tb2_callback)

    def result_tb2_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded! Moving to next waypoint.")
            self.current_tb2_waypoint_index += 1
            self.move_to_tb2_patrol()
        else:
            self.get_logger().info(f"Goal failed with status: {status}")



class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.node.tb1_img_sig.connect(self.update_tb1_image) 
        self.node.tb2_img_sig.connect(self.update_tb2_image)

        self.node.signal1.connect(self.update_tb1_position)
        self.node.signal2.connect(self.update_tb2_position)

        self.map_size = (400, 300)
        self.dot_size = 4

        # map 가져오기
        cur_directory = os.getcwd()
        get_map_pgm = os.path.join(cur_directory, 'src', 'turtlebot3_multi_robot', 'map', 'map_fin_b4.pgm')
        get_map_yaml = os.path.join(cur_directory, 'src', 'turtlebot3_multi_robot', 'map', 'map_fin_b4.yaml')
        image = Image.open(get_map_pgm)
        self.width, self.height = image.size
        self.rgb_image = image.convert('RGB')  # RGB로 변환

        with open(get_map_yaml, 'r') as file:
            data = yaml.safe_load(file)

        self.resolution = data['resolution']
        self.map_x = -data['origin'][0]
        self.map_y = +data['origin'][1] + self.height * self.resolution

        self.tb1_x = self.map_x
        self.tb1_y = self.map_y

        self.tb2_x = self.map_x
        self.tb2_y = self.map_y

        self.set_layout()

    def set_layout(self):
        self.setWindowTitle("Kill Machine Monitor")

        # Main central widget layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Title Label
        self.title_label = QLabel('AMCL Pose Map')
        self.title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.title_label)

        monitor_layer = QHBoxLayout()

        # tb1
        self.real_time_image_1 = QLabel()  
        self.real_time_image_1.setFixedSize(500, 500)
        # 배경을 검정색으로 설정
        palette = self.real_time_image_1.palette()
        palette.setColor(QPalette.Background, QColor(0, 0, 0))  # 검은색
        self.real_time_image_1.setAutoFillBackground(True)  # 배경을 자동으로 채우도록 설정
        self.real_time_image_1.setPalette(palette)
        monitor_layer.addWidget(self.real_time_image_1)

        # tb2
        self.real_time_image_2 = QLabel() 
        self.real_time_image_2.setFixedSize(500, 500)
        # 배경을 검정색으로 설정
        palette = self.real_time_image_2.palette()
        palette.setColor(QPalette.Background, QColor(0, 0, 0))  # 검은색
        self.real_time_image_2.setAutoFillBackground(True)  # 배경을 자동으로 채우도록 설정
        self.real_time_image_2.setPalette(palette)
        monitor_layer.addWidget(self.real_time_image_2)

        layout.addLayout(monitor_layer)

        # Control layout (for Map + 버튼)
        control_layout = QHBoxLayout()
        control_btn_layout = QVBoxLayout()

        # mini map 표시
        self.mini_map_label = QLabel()
        control_layout.addWidget(self.mini_map_label)

        self.patrol_btn = QPushButton("순찰 시작")
        control_btn_layout.addWidget(self.patrol_btn)

        self.goback_btn = QPushButton("복귀")
        control_btn_layout.addWidget(self.goback_btn)

        control_layout.addLayout(control_btn_layout)

        layout.addLayout(control_layout)

        self.patrol_btn.clicked.connect(self.start_patrol_callback)
        self.goback_btn.clicked.connect(self.go_back_callback)

        self.render_initial_map()

    def render_initial_map(self):
        image_resized = self.rgb_image.resize(self.map_size) 
        pil_image = image_resized.convert('RGBA')  
        data = pil_image.tobytes("raw", "RGBA") 
        qimage = QImage(data, *self.map_size, QImage.Format_RGBA8888) 
        pixmap = QPixmap.fromImage(qimage)  
        self.mini_map_label.setPixmap(pixmap)  

    def start_patrol_callback(self):
        print("순찰 시작")
        self.move_patrol()

    def move_patrol(self):
        # TODO: tb1 순찰
        '''
        position:
            x: -4.909138493002045
            y: 0.2814362862384074
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: -0.18987208367643849
            w: 0.9818088367092483

        position:
            x: -3.87519353082685
            y: 7.2925887149866
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.11575822764056436
            w: 0.9932774198246507

        position:
            x: -0.4624558095771992
            y: -3.0876787502594336
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: -0.16318783232986706
            w: 0.9865950189310096

        '''
        tb1_waypoints = [
            (-4.909138493002045, 0.2814362862384074, 0.0, 0.0, 0.0, -0.18987208367643849, 0.9818088367092483), 
            (-3.87519353082685, 7.2925887149866, 0.0, 0.0, 0.0, 0.11575822764056436, 0.9932774198246507),
            (-0.4624558095771992, -3.0876787502594336, 0.0, 0.0, 0.0, -0.16318783232986706, 0.9865950189310096), 
        ]
        self.node.tb1_waypoints = tb1_waypoints
        self.node.move_to_tb1_patrol()

        # tb2 순찰
        # 여러 웨이포인트 정의 (x, y, z, orientation_x, orientation_y, orientation_z, orientation_w)
        '''
        ros2 topic echo /tb2/amcl_pose

        position:
            x: 9.724727147486803
            y: 1.1282920060342905
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.21110228503119507
            w: 0.9774639764485482

        position:
            x: 9.26881605892118
            y: 7.748670610189714
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.16855217845158202
            w: 0.9856927326196668

        position:
            x: 5.606099203609427
            y: -1.3065951301641472
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: -0.16748830071882256
            w: 0.9858740635204485

        '''
        tb2_waypoints = [
            (9.724727147486803, 1.1282920060342905, 0.0, 0.0, 0.0, 0.21110228503119507, 0.9774639764485482), 
            (9.26881605892118, 7.748670610189714, 0.0, 0.0, 0.0, 0.16855217845158202, 0.9856927326196668),
            (5.606099203609427, -1.3065951301641472, 0.0, 0.0, 0.0, -0.16748830071882256, 0.9858740635204485), 
        ]
        self.node.tb2_waypoints = tb2_waypoints
        self.node.move_to_tb2_patrol()


    def go_back_callback(self):
        # TODO: 복귀 기능 구현
        print('복귀')

    def update_tb1_image(self, qimg):
        if qimg is None or qimg.isNull():
            print("Invalid QImage received!")
            return

        # QLabel의 크기 확인
        label_width = self.real_time_image_1.width()
        label_height = self.real_time_image_1.height()
        
        if label_width == 0 or label_height == 0:
            print("Invalid QLabel size!")
            return

        try:
            # # 이미지를 크기 조정
            # scaled_image = qimg.scaled(
            #     label_width,
            #     label_height,
            #     Qt.KeepAspectRatio
            # )

            pixmap = QPixmap.fromImage(qimg)
            self.real_time_image_1.setPixmap(pixmap)
        except Exception as e:
            print(f"Error during image update: {e}")

    def update_tb2_image(self, qimg):
        if qimg is None or qimg.isNull():
            print("Invalid QImage received!")
            return

        # QLabel의 크기 확인
        label_width = self.real_time_image_2.width()
        label_height = self.real_time_image_2.height()
        
        if label_width == 0 or label_height == 0:
            print("Invalid QLabel size!")
            return

        try:
            # # 이미지를 크기 조정
            # scaled_image = qimg.scaled(
            #     label_width,
            #     label_height,
            #     Qt.KeepAspectRatio
            # )

            pixmap = QPixmap.fromImage(qimg)
            self.real_time_image_2.setPixmap(pixmap)
        except Exception as e:
            print(f"Error during image update: {e}")


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
        self.mini_map_label.setPixmap(pixmap)




def main():
    rclpy.init()
    ros_node = RosNode()

    app = QApplication(sys.argv)
    gui = MainWindow(ros_node)
    gui.show()

    # ROS2 Spin을 별도 스레드에서 실행
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    def ros_spin():
        executor.spin()

    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    try:
        sys.exit(app.exec_())
    finally:
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
