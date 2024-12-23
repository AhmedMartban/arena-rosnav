import dataclasses
import functools
import math
import os
import time
from threading import Lock
from typing import Any, Callable, Collection, Dict, List

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import (ComputeAgent, ComputeAgents, GetAgents, MoveAgent,
                            ResetAgents)

from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.hunav_manager import HunavDynamicObstacle
from task_generator.manager.entity_manager.utils import (KnownObstacles,
                                                         ObstacleLayer,
                                                         walls_to_obstacle)
from task_generator.shared import (DynamicObstacle, Model, ModelType,
                                   Namespace, PositionOrientation, Robot)
from task_generator.simulators import BaseSimulator
from task_generator.utils.geometry import quaternion_from_euler
from task_generator.manager.world_manager.utils import WorldMap, WorldWalls
from .import SKIN_TYPES 


import dataclasses
import functools
import math
import os
import time
from threading import Lock
from typing import Any, Callable, Collection, Dict, List

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import (ComputeAgent, ComputeAgents, GetAgents, MoveAgent,
                            ResetAgents)

from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.hunav_manager import HunavDynamicObstacle
from task_generator.manager.entity_manager.utils import (KnownObstacles,
                                                         ObstacleLayer,
                                                         walls_to_obstacle)
from task_generator.shared import (DynamicObstacle, Model,ModelWrapper, ModelType,
                                   Namespace, PositionOrientation, Robot)
from task_generator.simulators import BaseSimulator
from task_generator.utils.geometry import quaternion_from_euler


class HunavManager(EntityManager):
    """HunavManager with debug logging for tracking execution flow"""
    
    # Service Names
    SERVICE_COMPUTE_AGENT = 'compute_agent'
    SERVICE_COMPUTE_AGENTS = 'compute_agents'
    SERVICE_MOVE_AGENT = 'move_agent'
    SERVICE_RESET_AGENTS = 'reset_agents'

    SKIN_TYPES = {
            0: 'elegant_man.dae',
            1: 'casual_man.dae',
            2: 'elegant_woman.dae',
            3: 'regular_man.dae',
            4: 'worker_man.dae',
            5: 'walk.dae'
        }

    def __init__(self, namespace: Namespace, simulator: BaseSimulator):
        """Initialize HunavManager with debug logging"""
        self.node.get_logger().warn("=== HUNAVMANAGER INIT START ===")
        
        super().__init__(namespace=namespace, simulator=simulator)
        self.node.get_logger().warn("Parent class initialized")

        # Initialize state variables
        self.node.get_logger().warn("Initializing state variables...")
        self._is_paused = False
        self._semaphore_reset = False
        self._agents_initialized = False
        self._robot_initialized = False
        self._lock = Lock()
        self._update_rate = 0.1
        self.node.get_logger().warn("State variables initialized")

        # Initialize collections
        self.node.get_logger().warn("Initializing collections...")
        self._known_obstacles = KnownObstacles()
        self._pedestrians = {}
        self.node.get_logger().warn("Collections initialized")

        # Setup services
        self.node.get_logger().warn("Setting up services...")
        setup_success = self.setup_services()
        if not setup_success:
            self.node.get_logger().error("Service setup failed!")
        else:
            self.node.get_logger().warn("Services setup complete")

        # Wait to be sure that all services are ready
        self.node.get_logger().warn("Waiting for services to be ready...")
        time.sleep(2.0)
        self.node.get_logger().warn("Service wait complete")

        # Setup timer for pedestrian updates
        self.node.get_logger().warn("Setting up update timer...")
        self._update_timer = self.node.create_timer(
            self._update_rate,
            self._update_pedestrians
        )
        self.node.get_logger().warn("Update timer setup complete")

        # Initialize JAIL_POS generator
        self.node.get_logger().warn("Initializing JAIL_POS generator...")
        
        def gen_JAIL_POS(steps: int, x: int = 1, y: int = 0):
            steps = max(steps, 1)
            while True:
                x += y == steps
                y %= steps
                yield PositionOrientation(-x, y, 0)
                y += 1
        self.JAIL_POS = gen_JAIL_POS(10)
        self.node.get_logger().warn("JAIL_POS generator initialized")

        self.node.get_logger().warn("=== HUNAVMANAGER INIT COMPLETE ===")

    def test_hunav_services(self):
        """Test all HuNav services with debug logging"""
        self.node.get_logger().warn("=== TEST_HUNAV_SERVICES START ===")

        # Create test agents
        self.node.get_logger().warn("Creating test agents...")
        test_agents = Agents()
        test_agents.header.stamp = self.node.get_clock().now().to_msg()
        test_agents.header.frame_id = "map"

        # Create test agent
        self.node.get_logger().warn("Creating test pedestrian...")
        test_agent = Agent()
        test_agent.id = 1
        test_agent.name = "test_pedestrian"
        test_agent.type = Agent.PERSON
        test_agent.position.position.x = 2.0
        test_agent.position.position.y = 2.0
        test_agent.yaw = 0.0
        test_agent.desired_velocity = 1.0
        test_agent.radius = 0.35

        # Set behavior
        self.node.get_logger().warn("Setting test agent behavior...")
        test_agent.behavior = AgentBehavior()
        test_agent.behavior.type = AgentBehavior.BEH_REGULAR
        test_agent.behavior.configuration = AgentBehavior.BEH_CONF_DEFAULT
        test_agent.behavior.duration = 40.0
        test_agent.behavior.once = True
        test_agent.behavior.goal_force_factor = 2.0
        test_agent.behavior.obstacle_force_factor = 10.0
        test_agent.behavior.social_force_factor = 5.0

        # Add test goal
        self.node.get_logger().warn("Adding test goal...")
        goal = Pose()
        goal.position.x = 5.0
        goal.position.y = 5.0
        test_agent.goals.append(goal)
        test_agent.cyclic_goals = True
        test_agent.goal_radius = 0.3

        test_agents.agents.append(test_agent)

        # Create test robot
        self.node.get_logger().warn("Creating test robot...")
        test_robot = Agent()
        test_robot.id = 0
        test_robot.name = "test_robot"
        test_robot.type = Agent.ROBOT
        test_robot.position.position.x = 0.0
        test_robot.position.position.y = 0.0
        test_robot.yaw = 0.0
        test_robot.radius = 0.3

        # Test compute_agents service
        try:
            self.node.get_logger().warn("Testing compute_agents service...")
            request = ComputeAgents.Request()
            request.robot = test_robot
            request.current_agents = test_agents

            self.node.get_logger().warn(f"Sending request with {len(request.current_agents.agents)} agents")
            future = self._compute_agents_client.call(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result():
                response = future.result()
                self.node.get_logger().warn(f"Received response with {len(response.updated_agents.agents)} agents")
                for agent in response.updated_agents.agents:
                    self.node.get_logger().warn(
                        f"\nAgent {agent.name} (ID: {agent.id}):"
                        f"\n  Position: ({agent.position.position.x:.2f}, {agent.position.position.y:.2f})"
                        f"\n  Behavior Type: {agent.behavior.type}"
                        f"\n  Current State: {agent.behavior.state}"
                        f"\n  Linear Velocity: {agent.linear_vel:.2f}"
                        f"\n  Angular Velocity: {agent.angular_vel:.2f}"
                    )
        except Exception as e:
            self.node.get_logger().error(f"compute_agents service test failed: {str(e)}")

        # Test compute_agent service
        try:
            self.node.get_logger().warn("Testing compute_agent service...")
            request = ComputeAgent.Request()
            request.id = test_agent.id

            future = self._compute_agent_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result():
                response = future.result()
                agent = response.updated_agent
                self.node.get_logger().warn(
                    f"Compute_agent response:"
                    f"\n  Agent: {agent.name} (ID: {agent.id})"
                    f"\n  Position: ({agent.position.position.x:.2f}, {agent.position.position.y:.2f})"
                    f"\n  Behavior: Type={agent.behavior.type}, State={agent.behavior.state}"
                )
        except Exception as e:
            self.node.get_logger().error(f"compute_agent service test failed: {str(e)}")

        # Test move_agent service
        try:
            self.node.get_logger().warn("Testing move_agent service...")
            request = MoveAgent.Request()
            request.agent_id = test_agent.id
            request.robot = test_robot
            request.current_agents = test_agents

            future = self._move_agent_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result():
                response = future.result()
                agent = response.updated_agent
                self.node.get_logger().warn(
                    f"Move_agent response:"
                    f"\n  Agent: {agent.name} (ID: {agent.id})"
                    f"\n  New Position: ({agent.position.position.x:.2f}, {agent.position.position.y:.2f})"
                    f"\n  New Yaw: {agent.yaw:.2f}"
                    f"\n  Behavior State: {agent.behavior.state}"
                )
        except Exception as e:
            self.node.get_logger().error(f"move_agent service test failed: {str(e)}")

        self.node.get_logger().warn("=== TEST_HUNAV_SERVICES COMPLETE ===")

    def setup_services(self):

        """Initialize all required services with debug logging"""
        self.node.get_logger().warn("=== SETUP_SERVICES START ===")
        
        # Debug namespace information
        self.node.get_logger().warn(f"Node namespace: {self.node.get_namespace()}")
        self.node.get_logger().warn(f"Task generator namespace: {self._namespace}")
        
        # Create service names with full namespace path
        service_names = {
            'compute_agent': f'/{self.SERVICE_COMPUTE_AGENT}',
            'compute_agents': f'/{self.SERVICE_COMPUTE_AGENTS}',
            'move_agent': f'/{self.SERVICE_MOVE_AGENT}',
            'reset_agents': f'/{self.SERVICE_RESET_AGENTS}'
        }
        
        # Log service creation attempts
        for service, full_name in service_names.items():
            self.node.get_logger().warn(f"Creating service client for {service} at: {full_name}")

        # Create service clients
        self.node.get_logger().warn("Creating compute_agent client...")
        self._compute_agent_client = self.node.create_client(
            ComputeAgent,
            service_names['compute_agent']
        )
        
        self.node.get_logger().warn("Creating compute_agents client...")
        self._compute_agents_client = self.node.create_client(
            ComputeAgents,
            service_names['compute_agents']
        )
        
        self.node.get_logger().warn("Creating move_agent client...")
        self._move_agent_client = self.node.create_client(
            MoveAgent,
            service_names['move_agent']
        )
        
        self.node.get_logger().warn("Creating reset_agents client...")
        self._reset_agents_client = self.node.create_client(
            ResetAgents,
            service_names['reset_agents']
        )

        # Wait for Services
        required_services = [
            (self._compute_agent_client, 'compute_agent'),
            (self._compute_agents_client, 'compute_agents'),
            (self._move_agent_client, 'move_agent'),
            (self._reset_agents_client, 'reset_agents')
        ]

        max_attempts = 5
        for client, name in required_services:
            attempts = 0
            self.node.get_logger().warn(f"Waiting for service {name}...")
            
            while attempts < max_attempts:
                if client.wait_for_service(timeout_sec=2.0):
                    self.node.get_logger().warn(f'Service {name} is available')
                    break
                attempts += 1
                self.node.get_logger().warn(
                    f'Waiting for service {name} (attempt {attempts}/{max_attempts})\n'
                    f'Looking for service at: {service_names[name]}'
                )

            if attempts >= max_attempts:
                self.node.get_logger().error(
                    f'Service {name} not available after {max_attempts} attempts\n'
                    f'Was looking for service at: {service_names[name]}'
                )
                self.node.get_logger().warn("=== SETUP_SERVICES FAILED ===")
                return False

        self.node.get_logger().warn("=== SETUP_SERVICES COMPLETE ===")
        return True

    def create_pedestrian_sdf(self, agent_config: HunavDynamicObstacle) -> str:
        """Create SDF description for pedestrian (from WorldGenerator)"""
        skin_type = self.SKIN_TYPES.get(
            agent_config.skin,  # Access the attribute directly
            'casual_man.dae')

        # Get path to mesh file
        mesh_path = os.path.join(
            get_package_share_directory('hunav_sim'),
            'hunav_rviz2_panel/meshes',
            skin_type
        )

        # Height adjustment based on skin type (from HuNavPlugin)
        height_adjustments = {
            'elegant_man.dae': 0.96,
            'casual_man.dae': 0.97,
            'elegant_woman.dae': 0.93,
            'regular_man.dae': 0.93,
            'worker_man.dae': 0.97
        }
        z_pos = height_adjustments.get(skin_type, 1.0)

        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{agent_config.name}">
                <static>false</static>
                <pose>0 0 {z_pos} 0 0 0</pose>
                <link name="link">
                    <visual name="visual">
                        <geometry>
                            <mesh>
                                <uri>file://{mesh_path}</uri>
                            </mesh>
                        </geometry>
                    </visual>
                    <collision name="collision">
                        <geometry>
                            <cylinder>
                                <radius>{agent_config.radius}</radius>
                                <length>1.7</length>
                            </cylinder>
                        </geometry>
                    </collision>
                </link>
            </model>
        </sdf>"""
        return sdf



    def spawn_obstacles(self, obstacles):
        """Spawn static obstacles with debug logging"""
        self.node.get_logger().warn("=== SPAWN_OBSTACLES START ===")
        self.node.get_logger().warn(f"Attempting to spawn {len(list(obstacles))} obstacles")
        
        for obstacle in obstacles:
            self.node.get_logger().warn(f"\nProcessing obstacle: {obstacle.name}")
            
            # Known obstacles check
            self.node.get_logger().warn(f"Checking if {obstacle.name} is known")
            known = self._known_obstacles.get(obstacle.name)
            if known is not None:
                self.node.get_logger().warn(f"{obstacle.name} is already known")
                if known.obstacle.name != obstacle.name:
                    error_msg = f"New model name {obstacle.name} does not match model name {known.obstacle.name}"
                    self.node.get_logger().error(error_msg)
                    raise RuntimeError(error_msg)
                known.layer = ObstacleLayer.INUSE
            else:
                self.node.get_logger().warn(f"Creating new known obstacle for {obstacle.name}")
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle,
                    hunav_spawned=False,
                    layer=ObstacleLayer.INUSE,
                )

            # Attempt to spawn
            self.node.get_logger().warn(f"Spawning entity {obstacle.name}")
            success = self._simulator.spawn_entity(obstacle)
            if success:
                self.node.get_logger().warn(f"Successfully spawned {obstacle.name}")
            else:
                self.node.get_logger().error(f"Failed to spawn {obstacle.name}")

        self.node.get_logger().warn("=== SPAWN_OBSTACLES COMPLETE ===")



    def spawn_dynamic_obstacles(self, obstacles: Collection[DynamicObstacle]):
        """Spawn dynamic obstacles (pedestrians) with debug logging"""
        self.node.get_logger().warn("=== SPAWN_DYNAMIC_OBSTACLES START ===")
        self.node.get_logger().warn(f"Received {len(list(obstacles))} dynamic obstacles")
        
        for obstacle in obstacles:
            self.node.get_logger().warn(f"\n--- Processing dynamic obstacle: {obstacle.name} ---")
            
            try:
                # Parse obstacle
                self.node.get_logger().warn("Parsing obstacle as HunavDynamicObstacle")
                obstacle = HunavDynamicObstacle.parse(obstacle.extra, obstacle.model)
                
                # Create agent
                self.node.get_logger().warn("Creating Hunav Agent")
                agent = Agent()

                # Set basic properties
                self.node.get_logger().warn("Setting basic properties")
                agent.id = obstacle.id
                agent.type = obstacle.type
                agent.skin = obstacle.skin
                agent.name = obstacle.name
                agent.group_id = obstacle.group_id

                # Set position
                self.node.get_logger().warn("Setting position data")
                agent.position = obstacle.position
                agent.yaw = obstacle.yaw

                # Set velocity
                self.node.get_logger().warn("Setting velocity data")
                agent.velocity = obstacle.velocity if obstacle.velocity else Twist()
                agent.desired_velocity = obstacle.desired_velocity
                agent.radius = obstacle.radius
                agent.linear_vel = obstacle.linear_vel
                agent.angular_vel = obstacle.angular_vel

                # Set behavior
                self.node.get_logger().warn("Setting behavior data")
                agent.behavior = AgentBehavior()
                agent.behavior.type = obstacle.behavior.type
                agent.behavior.configuration = obstacle.behavior.configuration
                agent.behavior.duration = obstacle.behavior.duration
                agent.behavior.once = obstacle.behavior.once
                agent.behavior.vel = obstacle.behavior.vel
                agent.behavior.dist = obstacle.behavior.dist
                agent.behavior.goal_force_factor = obstacle.behavior.goal_force_factor
                agent.behavior.obstacle_force_factor = obstacle.behavior.obstacle_force_factor
                agent.behavior.social_force_factor = obstacle.behavior.social_force_factor
                agent.behavior.other_force_factor = obstacle.behavior.other_force_factor

                # Set goals
                self.node.get_logger().warn("Setting goals")
                agent.goals = obstacle.goals
                agent.cyclic_goals = obstacle.cyclic_goals
                agent.goal_radius = obstacle.goal_radius
                agent.closest_obs = obstacle.closest_obs

                # Create SDF model
                self.node.get_logger().warn("Creating SDF model")
                sdf = self.create_pedestrian_sdf(obstacle)
                spawn_model = Model(   # Create a new model variable instead of replacing the original
                    description=sdf,
                    type=ModelType.SDF,
                    name=obstacle.name,
                    path='',
                )
                # Create a new obstacle with the spawn model
                spawn_obstacle = dataclasses.replace(
                    obstacle,
                    model=ModelWrapper.from_model(spawn_model) 
                )

                # Spawn in simulator
                self.node.get_logger().warn(f"Attempting to spawn {spawn_obstacle.name} in simulator")
                spawn_success = self._simulator.spawn_entity(spawn_obstacle)  # Use spawn_obstacle
                
                if spawn_success:
                    self.node.get_logger().warn(f"Successfully spawned {obstacle.name}")
                else:
                    self.node.get_logger().error(f"Failed to spawn {obstacle.name}")
                    continue

                # Register with HuNav
                self.node.get_logger().warn("Registering with HuNav")
                request = ComputeAgents.Request() 
                
                # Create new Agents message and setup header
                request.current_agents = Agents()
                request.current_agents.header.stamp = self.node.get_clock().now().to_msg()
                request.current_agents.header.frame_id = "map"

                # Get agents list and add our pedestrian
                agents_list = request.current_agents.agents
                agents_list.append(agent)

                # Add empty robot state (required by service)
                request.robot = Agent()

                # Call service and get future
                future = self._compute_agents_client.call(request)
                
                if future.result():
                    self.node.get_logger().warn(f"Successfully registered {obstacle.name} with HuNav")
                else:
                    self.node.get_logger().error(f"Failed to register {obstacle.name} with HuNav")

                # Store in known obstacles
                self.node.get_logger().warn("Storing in known obstacles")
                self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle,
                    hunav_spawned=True,
                    layer=ObstacleLayer.INUSE,
                )

            except Exception as e:
                self.node.get_logger().error(f"Error processing {obstacle.name}: {str(e)}")
                continue
            
            self.node.get_logger().warn(f"--- Completed processing {obstacle.name} ---")

        self.node.get_logger().warn("=== SPAWN_DYNAMIC_OBSTACLES COMPLETE ===")




    def spawn_walls(self, walls: WorldWalls, heightmap: WorldMap):
        """Spawn walls"""
        self.node.get_logger().debug(f'spawning {len(walls)} walls')
        # Convert walls to obstacle and spawn it
        wall_obstacle = walls_to_obstacle(heightmap)
        self._simulator.spawn_entity(wall_obstacle)
        self._known_obstacles.create_or_get(
            name=wall_obstacle.name,
            obstacle=wall_obstacle,
            hunav_spawned=False,
            layer=ObstacleLayer.INUSE  # Change to INUSE from WORLD
        )


    def unuse_obstacles(self):
        """Just mark obstacles as unused without removing them"""
        self.node.get_logger().debug('unusing obstacles')
        for obstacle_id, obstacle in self._known_obstacles.items():
            if obstacle.layer == ObstacleLayer.INUSE:
                obstacle.layer = ObstacleLayer.UNUSED


    def remove_obstacles(self, purge=ObstacleLayer.UNUSED):  # Change default to UNUSED from WORLD
        """Remove obstacles"""
        self.node.get_logger().debug(f'removing obstacles (level {purge})')

        if not self._is_paused:
            self._is_paused = True
            request = ResetAgents.Request()
            self._reset_agents_client.call(request)

        while self._semaphore_reset:
            time.sleep(0.1)

        self._semaphore_reset = True
        try:
            for obstacle_id, obstacle in list(self._known_obstacles.items()):
                if purge >= obstacle.layer:
                    if obstacle.hunav_spawned:
                        obstacle.hunav_spawned = False
                    self._simulator.delete_entity(name=obstacle_id)
                    self._known_obstacles.forget(name=obstacle_id)
                    if obstacle_id in self._pedestrians:
                        del self._pedestrians[obstacle_id]
        finally:
            self._semaphore_reset = False


    def spawn_robot(self, robot: Robot):

        """Spawn robot with debug logging"""
        self.node.get_logger().warn("=== SPAWN_ROBOT START ===")
        self.node.get_logger().warn(f"Attempting to spawn robot: {robot.name}")
        self.node.get_logger().warn(f"Robot position: {robot.position}")
        
        success = self._simulator.spawn_entity(robot)
        if success:
            self.node.get_logger().warn(f"Successfully spawned robot {robot.name}")
        else:
            self.node.get_logger().error(f"Failed to spawn robot {robot.name}")
            
        self.node.get_logger().warn("=== SPAWN_ROBOT COMPLETE ===")

    def remove_robot(self, name: str):
        """Remove robot with debug logging"""
        self.node.get_logger().warn(f"=== REMOVING ROBOT {name} ===")
        success = self._simulator.delete_entity(name)
        if success:
            self.node.get_logger().warn(f"Successfully removed robot {name}")
        else:
            self.node.get_logger().error(f"Failed to remove robot {name}")
        return success


    def move_robot(self, name: str, position: PositionOrientation):
        """Move robot to new position using HuNavSim's move_agent service"""
        self.node.get_logger().info(f"Moving robot {name}")

        # Basic validation
        if not name or not position:
            self.node.get_logger().warn("Invalid move parameters")
            return

        # Wait for services
        if not self._move_agent_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Move agent service not available")
            return

        try:
            # Create Robot Agent safely
            robot_agent = Agent()
            robot_agent.id = 0  # Robot uses ID 0
            robot_agent.name = name
            robot_agent.type = Agent.ROBOT
            robot_agent.position.position.x = position.x
            robot_agent.position.position.y = position.y
            quat = quaternion_from_euler(0.0, 0.0, position.orientation)
            robot_agent.position.orientation = Quaternion(
                x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            robot_agent.yaw = position.orientation

            # Create empty agents message
            request = MoveAgent.Request()
            request.agent_id = 0
            request.robot = robot_agent
            request.current_agents = Agents()
            request.current_agents.header.stamp = self.node.get_clock().now().to_msg()
            request.current_agents.header.frame_id = "map"

            # Call service with timeout
            future = self._move_agent_client.call_async(request)
            
            timeout_sec = 1.0
            start_time = time.time()
            while not future.done() and time.time() - start_time < timeout_sec:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if not future.done():
                self.node.get_logger().error("Move robot service call timed out")
                return

            if future.result() is not None:
                self.node.get_logger().info(f"Successfully moved robot {name}")
            else:
                self.node.get_logger().error("Move robot service call failed")

        except Exception as e:
            self.node.get_logger().error(f"Error moving robot: {str(e)}")


    def _update_pedestrians(self):
        """Update pedestrians with debug logging"""
        with self._lock:
            self.node.get_logger().warn("=== UPDATE_PEDESTRIANS START ===")
            current_time = time.time()

            # Get updates from Hunav
            self.node.get_logger().warn("Requesting agent updates from HuNav")
            request = ComputeAgents.Request()
            future = self._compute_agents_client.call(request)

            if future.result() is not None:
                agents = future.result().agents
                self.node.get_logger().warn(f"Received updates for {len(agents)} agents")
                
                for agent in agents:
                    if agent.id in self._pedestrians:
                        self.node.get_logger().warn(f"Updating pedestrian {agent.id}")
                        ped_data = self._pedestrians[agent.id]

                        # Update position
                        pos = PositionOrientation(
                            x=agent.position.position.x,
                            y=agent.position.position.y,
                            orientation=agent.yaw
                        )
                        self.node.get_logger().warn(f"Moving entity {agent.id} to new position")
                        self._simulator.move_entity(name=agent.id, position=pos)

                        # Update animation
                        dt = current_time - ped_data['last_update']
                        animation_factor = self._get_animation_factor(agent.behavior)
                        ped_data['animation_time'] += dt * animation_factor

                        # Update animation state if needed
                        if agent.behavior.state != ped_data.get('current_state'):
                            self.node.get_logger().warn(f"Updating animation state for {agent.id}")
                            self._update_agent_animation(agent.id, agent.behavior)
                            ped_data['current_state'] = agent.behavior.state

                        ped_data['last_update'] = current_time
                    else:
                        self.node.get_logger().warn(f"Agent {agent.id} not found in pedestrians")

            self.node.get_logger().warn("=== UPDATE_PEDESTRIANS COMPLETE ===")

    def _get_animation_factor(self, behavior: AgentBehavior) -> float:
        """Get animation factor with debug logging"""
        self.node.get_logger().warn(f"Getting animation factor for behavior type {behavior.type}")
        
        if behavior.state == AgentBehavior.BEH_NO_ACTIVE:
            self.node.get_logger().warn("Behavior inactive, returning default factor 1.0")
            return 1.0

        factor = 1.0
        if behavior.type == AgentBehavior.BEH_REGULAR:
            factor = 1.5
        elif behavior.type == AgentBehavior.BEH_IMPASSIVE:
            factor = 1.5
        elif behavior.type == AgentBehavior.BEH_SURPRISED:
            factor = 1.0
        elif behavior.type == AgentBehavior.BEH_THREATENING:
            factor = 1.0
        elif behavior.type == AgentBehavior.BEH_SCARED:
            factor = 1.5
        elif behavior.type == AgentBehavior.BEH_CURIOUS:
            factor = 1.0

        self.node.get_logger().warn(f"Returning animation factor: {factor}")
        return factor

    def _update_agent_animation(self, agent_id: str, behavior: AgentBehavior):
        """Update agent animation with debug logging"""
        self.node.get_logger().warn(f"=== UPDATE_ANIMATION for agent {agent_id} ===")
        
        animation = 'WALK'
        if behavior.state == AgentBehavior.BEH_NO_ACTIVE:
            animation = 'NORMAL_WAIT'
        else:
            if behavior.type == AgentBehavior.BEH_REGULAR:
                animation = 'WALK'
            elif behavior.type == AgentBehavior.BEH_IMPASSIVE:
                animation = 'WALK_FORWARD'
            elif behavior.type == AgentBehavior.BEH_SURPRISED:
                animation = 'NORMAL_WAIT'
            elif behavior.type == AgentBehavior.BEH_THREATENING:
                animation = 'WALK_ANGRY'
            elif behavior.type == AgentBehavior.BEH_SCARED:
                animation = 'WALK_SCARED'
            elif behavior.type == AgentBehavior.BEH_CURIOUS:
                animation = 'SLOW_WALK'

        self.node.get_logger().warn(f"Selected animation: {animation}")
        if agent_id in self._pedestrians:
            self._pedestrians[agent_id]['current_animation'] = animation
            self.node.get_logger().warn(f"Animation updated for agent {agent_id}")
        else:
            self.node.get_logger().warn(f"Agent {agent_id} not found in pedestrians")


    def _load_agent_config(self, agent_id: str) -> Dict[str, Any]:
        """Load configuration for a specific agent

        Args:
            agent_id (str): ID of the agent

        Returns:
            Dict[str, Any]: Configuration dictionary
        """
        if agent_id not in self.agent_config:
            self.node.get_logger().warn(
                f"No configuration found for agent {agent_id}")
            return {}
        return self.agent_config[agent_id]




    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle with debug logging"""
        normalized = math.fmod(angle, math.pi)
        self.node.get_logger().warn(f"Normalized angle from {angle} to {normalized}")
        return normalized

  
