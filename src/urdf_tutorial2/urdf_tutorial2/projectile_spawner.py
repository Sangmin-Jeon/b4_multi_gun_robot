import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, ApplyLinkWrench
from geometry_msgs.msg import Pose, Wrench
from builtin_interfaces.msg import Duration
import random
from tf2_msgs.msg import TFMessage
import time


class ProjectileSpawner(Node):
    def __init__(self):
        super().__init__('projectile_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.wrench_client = self.create_client(ApplyLinkWrench, '/apply_link_wrench')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        while not self.wrench_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_link_wrench service...')
            
        self.get_logger().info("Ready to spawn and apply forces!")

        # 구독 설정
        self.subscription = self.create_subscription(
            TFMessage,
            'tb2/tf',
            self.sub_tf_callback,
            10
        )
        self.projectile_launched = False

    def sub_tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "base_link" and transform.header.frame_id == "odom":
                translation = transform.transform.translation
                position = [translation.x + 0.085, translation.y, translation.z + 0.035]
                self.get_logger().info(f"Transform received: x={translation.x}, y={translation.y}, z={translation.z}")
                
                # 스폰 후 충돌을 발생시킬 초기 속도
                velocity = [100.0, 0.0, 0.0]
                self.launch_projectile(position=position, velocity=velocity)

                self.projectile_launched = True
                self.destroy_subscription(self.subscription)
                time.sleep(1)
                self.apply_collision_force("projectile::link", [0.0, 0.0, 100.0])  # 충돌 후 힘 추가
                break

    def spawn_projectile(self, position):
        request = SpawnEntity.Request()
        request.name = f'projectile_{random.randint(1, 1000)}'
        request.xml = self.get_projectile_sdf()
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = position[0]
        request.initial_pose.position.y = position[1]
        request.initial_pose.position.z = position[2]

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawned projectile at {position}")
        else:
            self.get_logger().error(f"Failed to spawn projectile: {future.exception()}")

    def apply_velocity(self, body_name, velocity):
        request = ApplyLinkWrench.Request()
        request.link_name = body_name
        request.reference_frame = ''
        request.wrench.force.x = velocity[0]
        request.wrench.force.y = velocity[1]
        request.wrench.force.z = velocity[2]
        request.duration = Duration(sec=1)

        future = self.wrench_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Applied velocity {velocity} to {body_name}")
        else:
            self.get_logger().error(f"Failed to apply velocity: {future.exception()}")

    def apply_collision_force(self, body_name, force):
        request = ApplyLinkWrench.Request()
        request.link_name = body_name
        request.reference_frame = ''
        request.wrench.force.x = force[0]
        request.wrench.force.y = force[1]
        request.wrench.force.z = force[2]
        request.duration = Duration(sec=1)

        future = self.wrench_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Applied collision force {force} to {body_name}")
        else:
            self.get_logger().error(f"Failed to apply collision force: {future.exception()}")

    def get_projectile_sdf(self):
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
                        <surface>
                            <contact>
                                <ode>
                                    <max_vel>100000</max_vel>
                                    <min_depth>0.0000</min_depth>
                                </ode>
                            </contact>
                        </surface>
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

    def launch_projectile(self, position, velocity):
        self.spawn_projectile(position)
        self.apply_velocity("projectile::link", velocity)
        self.get_logger().info(f"Launching projectile with velocity {velocity}")


def main():
    rclpy.init()
    spawner = ProjectileSpawner()

    try:
        while rclpy.ok():
            rclpy.spin_once(spawner)
            if spawner.projectile_launched:
                break
    except KeyboardInterrupt:
        pass
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
