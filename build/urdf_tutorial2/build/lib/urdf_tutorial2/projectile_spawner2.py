import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import random


class ProjectileSpawner(Node):
    def __init__(self):
        super().__init__('projectile_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

        # /spawn_entity 서비스가 활성화될 때까지 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.get_logger().info('/spawn_entity service is ready.')

        self.position_publisher = self.create_publisher(String, 'person_topic', 10)

    def spawn_projectile(self, position):
        self.get_logger().info(f"Spawning projectile at position: {position}")

        # SpawnEntity 요청 생성
        request = SpawnEntity.Request()
        request.name = f'projectile_{random.randint(1, 1000)}'
        request.xml = self.get_projectile_sdf()
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = position[0]
        request.initial_pose.position.y = position[1]
        request.initial_pose.position.z = position[2]

        # SpawnEntity 서비스 호출
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Successfully spawned projectile at {position}")
            self.publish_position(position)
        else:
            self.get_logger().error(f"Failed to spawn projectile: {future.exception()}")

    def publish_position(self, position):
        position_str = f"{position[0]}/{position[1]}/{position[2]}/0.0/0.0/1.0"
        self.get_logger().info(f"Publishing position to topic 'person_topic': {position_str}")
        try:
            self.position_publisher.publish(String(data=position_str))
        except Exception as e:
            self.get_logger().error(f"Failed to publish position: {e}")

    def get_projectile_sdf(self):
        # SDF 파일 반환
        return """<sdf version="1.6">
            <model name="projectile">
                <link name="link">
                    <inertial>
                        <mass>0.01</mass>
                        <inertia>
                            <ixx>0.0001</ixx>
                            <iyy>0.0001</iyy>
                            <izz>0.0001</izz>
                        </inertia>
                    </inertial>
                    <collision name="collision">
                        <geometry>
                            <sphere><radius>0.02</radius></sphere>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <sphere><radius>0.02</radius></sphere>
                        </geometry>
                        <material>
                            <ambient>1 0 0 1</ambient>
                            <diffuse>1 0 0 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>"""


def main():
    rclpy.init()
    spawner = ProjectileSpawner()

    try:
        # 발사체를 한 번만 생성
        position = [-1.5, -1.5, 0.1]
        spawner.spawn_projectile(position)
    except Exception as e:
        spawner.get_logger().error(f"An error occurred: {e}")
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()