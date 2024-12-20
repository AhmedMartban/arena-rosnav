import dataclasses

import rclpy

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose, ControlWorld
from ros_gz_interfaces.msg import EntityFactory, WorldControl
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
import traceback
from task_generator.utils.geometry import quaternion_from_euler
from task_generator.simulators import BaseSimulator

from task_generator.shared import ModelType, Namespace, PositionOrientation, RobotProps

import launch
import launch_ros
import ament_index_python


class GazeboSimulator(BaseSimulator):
    def __init__(self, namespace):
        """Initialize GazeboSimulator

        Args:
            namespace: Namespace for the simulator
        """

        super().__init__(namespace=namespace)

        self.node.get_logger().info(
            f"Initializing GazeboSimulator with namespace: {namespace}")
        self._goal_pub = self.node.create_publisher(
            PoseStamped,
            self._namespace("goal"),
            10
        )
        self.entities = {}

    def before_reset_task(self):
        self.node.get_logger().info("Pausing simulation before reset")
        self.pause_simulation()

    def after_reset_task(self):
        self.node.get_logger().info("Unpausing simulation after reset")
        try:
            self.unpause_simulation()
        except Exception as e:
            self.node.get_logger().error(
                f"Error unpausing simulation: {str(e)}")
            traceback.print_exc()
            raise

    def _wait_for_service(self, service, name, timeout=15.0):
        return True
        self.node.get_logger().info(
            f"Waiting for {name} service (timeout: {timeout}s)...")
        timeout_loop = timeout
        while not service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(
                f"{name} service not available, waiting... ({timeout_loop}s remaining)")
            timeout_loop -= 1.0
            if timeout_loop <= 0:
                self.node.get_logger().error(
                    f"{name} service not available after {timeout} seconds")
                return False
        self.node.get_logger().info(f"{name} service is now available")
        return True

    def move_entity(self, name, position):
        self.node.get_logger().info(
            f"Attempting to move entity: {name}")
        entity = self.entities[name]
        self.delete_entity(name)
        entity = dataclasses.replace(entity, position=position)
        self.spawn_entity(entity)
        return True
        self.node.get_logger().info(
            f"Moving entity {name} to position: {position}")
        request = SetEntityPose.Request()
        request.entity = name
        request.pose = Pose(
            position=Point(x=position.x, y=position.y, z=0),
            orientation=Quaternion(
                *quaternion_from_euler(0.0, 0.0, position.orientation, axes="sxyz"))
        )

        try:
            future = self._set_entity_pose.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error(
                    f"Move service call failed for {name}")
                return False

            self.node.get_logger().info(
                f"Move result for {name}: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error moving entity {name}: {str(e)}")
            traceback.print_exc()
            return False

    def spawn_entity(self, entity):
        self.node.get_logger().info(
            f"Attempting to spawn entity: {entity.name}")

        launch_description = launch.LaunchDescription()

        try:
            description = entity.model.get(
                [ModelType.SDF, ModelType.URDF]).description
        except FileNotFoundError as e:
            self.node.get_logger().error(repr(e))
            return True

        launch_description.add_action(
            launch_ros.actions.Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    '-world', 'default',
                    '-string', description,
                    '-name', entity.name,
                    '-allow_renaming', 'false',
                    '-x', str(entity.position.x),
                    '-y', str(entity.position.y),
                    '-X', str(entity.position.orientation),
                ],
            )
        )
        
        gz_topic = '/world/default/model/' + entity.name
        joint_state_gz_topic = gz_topic + '/joint_state'
        link_pose_gz_topic = gz_topic + '/pose'

        # Bridge to connect Gazebo and ROS2
        launch_description.add_action(
            launch_ros.actions.Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                output='screen',
                arguments=[
                    # Joint states (Gazebo -> ROS2)
                    joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
                    # Link poses (Gazebo -> ROS2)
                    link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    link_pose_gz_topic + \
                        '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    # Velocity and odometry (Gazebo -> ROS2)
                    gz_topic + '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
                    gz_topic + '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                ],
                remappings=[
                    (joint_state_gz_topic, entity.name + '/joint_states'),
                    (link_pose_gz_topic, entity.name + '/tf'),
                    (link_pose_gz_topic + '_static', entity.name+ + '/tf_static'),
                    (gz_topic + '/link/base_link/sensor/imu_sensor/imu', entity.name+'/imu/data'),
                ],
                parameters=[
                    {
                        'qos_overrides./tf_static.publisher.durability': 'transient_local',
                        'use_sim_time': True
                    }
                ],
            )
        )
        self.entities[entity.name] = entity
        self.node.do_launch(launch_description)

        return True

        # Check service availability
        if not self._spawn_entity.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("Spawn service not available!")
            return False

        try:
            # Create spawn request
            request = SpawnEntity.Request()
            request.entity_factory = EntityFactory()
            request.entity_factory.name = entity.name
            request.entity_factory.type = 'sdf'

            # Get model description
            model_description = entity.model.get(self.MODEL_TYPES).description
            request.entity_factory.sdf = model_description

            self.node.get_logger().info(
                f"Model description length for {entity.name}: { len(model_description)}")

            # Set pose
            request.entity_factory.pose = Pose(
                position=Point(x=entity.position.x, y=entity.position.y, z=0),
                orientation=Quaternion(
                    *
                    quaternion_from_euler(
                        0.0,
                        0.0,
                        entity.position.orientation,
                        axes="sxyz"))
            )

            self.node.get_logger().info(
                f"Spawn position for {entity.name}: x={entity.position.x}, y={entity.position.y}")

            # For robots, declare additional parameters
            if isinstance(entity, RobotProps):
                self.node.get_logger().info(
                    f"Entity {entity.name} is a robot, declaring additional parameters")
                self.node.declare_parameter(
                    Namespace(entity.name)("robot_description"),
                    entity.model.get(self.MODEL_TYPES).description
                )
                self.node.declare_parameter(
                    Namespace(entity.name)("tf_prefix"),
                    str(Namespace(entity.name))
                )

            # Send spawn request
            self.node.get_logger().info(
                f"Sending spawn request for {entity.name}")
            future = self._spawn_entity.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error(
                    f"Spawn service call failed for {entity.name}")
                return False

            self.node.get_logger().info(
                f"Spawn result for {entity.name}: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error spawning entity {entity.name}: {str(e)}")
            traceback.print_exc()
            return False

    def delete_entity(self, name: str):
        self.node.get_logger().info(
            f"Attempting to delete entity: {name}")

        if name in self.entities:
            del self.entities[name]

        launch_description = launch.LaunchDescription()

        launch_description.add_action(
            launch_ros.actions.Node(
                package="ros_gz_sim",
                executable="remove",
                output="screen",
                parameters=[
                    {
                        "world": "default",
                        "entity_name": name,
                    }
                ],
            )
        )

        self.node.do_launch(launch_description)
        return True
        self.node.get_logger().info(f"Attempting to delete entity: {name}")
        request = DeleteEntity.Request()
        request.entity = name

        try:
            future = self._delete_entity.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error(
                    f"Delete service call failed for {name}")
                return False

            self.node.get_logger().info(
                f"Delete result for {name}: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error deleting entity {name}: {str(e)}")
            traceback.print_exc()
            return False

    def pause_simulation(self):
        return True
        self.node.get_logger().info("Attempting to pause simulation")
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.pause = True

        try:
            future = self._control_world.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error("Pause service call failed")
                return False

            self.node.get_logger().info(f"Pause result: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(f"Error pausing simulation: {str(e)}")
            traceback.print_exc()
            return False

    def unpause_simulation(self):
        return True
        self.node.get_logger().info("Attempting to unpause simulation")
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.pause = False

        try:
            future = self._control_world.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error("Unpause service call failed")
                return False

            self.node.get_logger().info(f"Unpause result: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error unpausing simulation: {str(e)}")
            traceback.print_exc()
            return False

    def step_simulation(self, steps):
        return True
        self.node.get_logger().info(f"Stepping simulation by {steps} steps")
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.multi_step = steps

        try:
            future = self._control_world.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error("Step service call failed")
                return False

            self.node.get_logger().info(f"Step result: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error stepping simulation: {str(e)}")
            traceback.print_exc()
            return False

    def _publish_goal(self, goal: PositionOrientation):
        self.node.get_logger().info(
            f"Publishing goal: x={goal.x}, y={goal.y}, orientation={goal.orientation}")
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, goal.orientation, axes="sxyz"))

        self._goal_pub.publish(goal_msg)
        self.node.get_logger().info("Goal published")
