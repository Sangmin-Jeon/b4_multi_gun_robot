import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class ProjectileSpawner(Node):
    def __init__(self):
        super().__init__('projectile_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

    def spawn_projectile(self, position, velocity):
        request = SpawnEntity.Request()
        request.name = f'projectile_{random.randint(1, 1000)}'
        request.xml = self.get_projectile_sdf()
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = position[0]
        request.initial_pose.position.y = position[1]
        request.initial_pose.position.z = position[2]

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawned projectile at {position}")
        else:
            self.get_logger().error(f"Failed to spawn projectile: {future.exception()}")

    def get_projectile_sdf(self):
        return """<sdf version="1.6">
            <model name="projectile">
                <link name="link">
                    <inertial>
                        <mass>0.1</mass>
                        <inertia>
                            <ixx>0.0001</ixx>
                            <iyy>0.0001</iyy>
                            <izz>0.0001</izz>
                        </inertia>
                    </inertial>
                    <collision>
                        <geometry>
                            <sphere><radius>0.05</radius></sphere>
                        </geometry>
                    </collision>
                    <visual>
                        <geometry>
                            <sphere><radius>0.05</radius></sphere>
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
        # Spawn the projectile
        self.spawn_projectile(position, velocity)

        # Use Gazebo services or apply force logic to launch it
        # TODO: Add custom logic for applying velocity if required

def main():
    rclpy.init()
    spawner = ProjectileSpawner()

    # Spawn and launch a projectile
    spawner.launch_projectile(position=[0.0, 0.0, 1.0], velocity=[10.0, 0.0, 0.0])

    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()