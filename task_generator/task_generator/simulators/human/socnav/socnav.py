#!/usr/bin/env python3
"""
SocNav Human Simulator for Arena
Uses Pre recorded Trajectory Data 
"""
#TO DO:
#1.load trajectory data from csv based on episode name and load trajectories in desired time frame
#1.2 orientation and velocity calculation
#1.3 implement sdf creation for different skin types
#2  implement service calls (delete_actors)
#4. test in gazebo and isaac sim




import os
from typing import Sequence
from collections.abc import Sequence

# Arena imports
from arena_rclpy_mixins.shared import Namespace
from task_generator.shared import DynamicObstacle, Obstacle
from task_generator.simulators.human.dummy import DummyHumanSimulator
from task_generator.simulators.sim import BaseSim
from task_generator.constants import Constants

from arena_people_msgs.msg import Pedestrian, Pedestrians
from arena_people_msgs.srv import DeleteActors

from .trajectory_loader import SimpleTrajectoryLoader

class _PedestrianHelper:



    _SKIN_TYPES = {
        0: 'elegant_man.dae',
        1: 'casual_man.dae',
        2: 'elegant_woman.dae',
        3: 'regular_man.dae',
        4: 'worker_man.dae',
        5: 'walk.dae'
    }


    @classmethod
    def plugin_entity(cls, namespace: str) -> Obstacle:

        sdf_content = f"""<?xml version="1.0" ?>
            <sdf version="1.9">
                <model name="human_plugin">
                    <static>true</static>
                    <link name="empty">
                        <visual name="visual">
                            <geometry>
                                <box>
                                    <size>0.01 0.01 0.01</size>
                                </box>
                            </geometry>
                        </visual>
                    </link>
                    <plugin name="HumanSystemPlugin" filename="libHumanSystemPlugin.so">
                        <update_rate>1000.0</update_rate>
                        <namespace>{namespace}</namespace>
                        <global_frame_to_publish>map</global_frame_to_publish>
                        <pedestrians_topic>arena_peds</pedestrians_topic>
                    </plugin>
                </model>
            </sdf>"""

        return Obstacle(
            name="human_plugin",
            pose=Pose(Position(x=0.0, y=0.0, z=-1.0)),
            model=ModelWrapper.Constant("human_plugin", {
                ModelType.SDF: Model(
                    type=ModelType.SDF,
                    name="human_plugin",
                    description=sdf_content,
                    path="",
                )
            })
        )



    ###!! anpassen: SocNavDynamicObstacle? und init positions anpassen 
    # @classmethod
    # def create_sdf(cls, agent_config: HunavDynamicObstacle) -> str:
    #     """Create SDF description for pedestrian using gz-sim actor format"""

    #     # Get skin type
    #     skin_type = cls._SKIN_TYPES.get(agent_config.skin, 'casual_man.dae')

    #     # Animation mapping based on behavior
    #     animation_file = '../models/walk.dae'  # temp

    #     # Construct paths
    #     mesh_path = os.path.join(
    #         get_package_share_directory('hunav_rviz2_panel'),
    #         'meshes/models',
    #         skin_type
    #     )

    #     animation_path = os.path.join(
    #         get_package_share_directory('hunav_rviz2_panel'),
    #         'meshes/animations',
    #         animation_file
    #     )

    #     # Create the SDF
    #     sdf = f"""<?xml version="1.0" ?>
    #     <sdf version="1.9">
    #         <actor name="{agent_config.name}">
    #             <pose>{agent_config.init_pose.x} {agent_config.init_pose.y} {cls._HEIGHTS.get(agent_config.skin, 1.0)} 0 0 {agent_config.yaw}</pose>

    #             <skin>
    #                 <filename>{mesh_path}</filename>
    #                 <scale>1.0</scale>
    #             </skin>

    #             <animation name="walking">
    #                 <filename>{animation_path}</filename>
    #                 <scale>1.0</scale>
    #                 <interpolate_x>true</interpolate_x>
    #             </animation>
    #         </actor>
    #     </sdf>"""
    #     return sdf
        

class SocNavHumanSimulator(DummyHumanSimulator):
    """Minimal SocNav Human Simulator - Starting Point"""

    #SERVICE_DELETE_ACTORS = 'delete_actors' will be added soon 

    def __init__(self, namespace: Namespace, simulator: BaseSim):
        super().__init__(namespace, simulator)
        self._current_frame = 1
        self._simulation_running = False
        self._active_pedestrians = {}  # {ped_id: pedestrian_data}
        self._trajectory_loader = SimpleTrajectoryLoader()
        self._logger.info("SocNav Human Simulator initialized (minimal version)")
        self._logger.info("Ready for step-by-step integration")

        self._logger.error("=== LOADING EPISODE FIRST ===")
        # Setup services
        self._logger.debug("Setting up services...")
        # setup_success = self._setup_services()            # will be added soon
        # if not setup_success:
        #     self._logger.error("Service setup failed!")
        # else:
        #     self._logger.error("Services setup complete")

        
        self.WORLD_TO_DATASET = {
            'map_zara': 'zara01',
            'map_eth': 'eth', 
            'map_hotel': 'hotel',
            'map_univ': 'univ',
            'map_zara02': 'zara02'
        }
        
        # ROS2 Parameter declaration & getting
        world_param = self.node.get_parameter('world').value
        dataset_name = self.WORLD_TO_DATASET.get(world_param, 'hotel')
        self._load_episode(dataset_name)
        self._logger.error(f"World parameter: {world_param}")
        self._logger.error(f"Selected dataset: {dataset_name}")



        arena_peds_success = self._setup_arena_peds_publisher()
        if arena_peds_success:
            self._start_trajectory_simulation()      
            self._logger.error("Arena peds publisher setup success!")

        self._logger.debug("Waiting for services to be ready...")
        #time.sleep(2.0)
        self._logger.debug("Service wait complete")

        self._logger.info("=== Socnav INIT COMPLETE ===")
    # =====================================================
    # Required Abstract Methods (from DummyHumanSimulator)
    # =====================================================

    @property
    def _simulator_type(self) -> Constants.SimSimulator:
        """Detect which simulator is being used"""
        return self.node.conf.Arena.SIM.value


    # def _setup_services(self):                                        # will be added soon
    #     """Initialize all required services with debug logging"""
    #     self._logger.error("=== SETUP_SERVICES START ===")

    #     # Debug namespace information
    #     self._logger.error(f"Node namespace: {self.node.get_namespace()}")
    #     self._logger.error(f"Task generator namespace: {self._namespace}")

    #     # Create service names with full namespace path
    #     service_names = {
    #         'delete_actors': self.node.service_namespace(self.SERVICE_DELETE_ACTORS)

    #     }

    #     # Log service creation attempts
    #     for service, full_name in service_names.items():
    #         self._logger.info(f"Creating service client for {service} at: {full_name}")


    #     self._logger.error("Creating delete_actors client...")
    #     self._delete_actors_client = self.node.create_client(
    #         DeleteActors,
    #         service_names['delete_actors'],
    #     )

    #     # Wait for Services
    #     required_services = [
    #         (self._delete_actors_client, 'delete_actors')
    #     ]

    #     max_attempts = float('inf')
    #     for client, name in required_services:
    #         attempts = 0
    #         self._logger.error(f"Waiting for service {name}...")

    #         while attempts < max_attempts:
    #             if client.wait_for_service(timeout_sec=2.0):
    #                 self._logger.debug(f'Service {name} is available')
    #                 break
    #             attempts += 1
    #             self._logger.debug(
    #                 f'Waiting for service {name} (attempt {attempts}/{max_attempts})\n'
    #                 f'Looking for service at: {service_names[name]}'
    #             )

    #         if attempts >= max_attempts:
    #             self._logger.error(
    #                 f'Service {name} not available after {max_attempts} attempts\n'
    #                 f'Was looking for service at: {service_names[name]}'
    #             )
    #             self._logger.error("=== SETUP_SERVICES FAILED ===")
    #             return False

    #     self._logger.error("=== SETUP_SERVICES COMPLETE ===")
    #     return True

    def _setup_arena_peds_publisher(self):
        """Setup arena_peds publisher"""
        try:
            
            self._logger.error("=== SETTING UP ARENA PEDS PUBLISHER ===")
            
            # Create publisher
            self._arena_peds_publisher = self.node.create_publisher(
                Pedestrians,
                self._namespace('arena_peds'),
                10
            )
            
            self._logger.error("Arena peds publisher created successfully")
            return True
            
        except Exception as e:
            self._logger.error(f"Arena peds publisher setup failed: {e}")
            return False




    def _start_trajectory_simulation(self):
        """Start frame-by-frame trajectory simulation"""
        try:
            self._logger.error("=== STARTING TRAJECTORY SIMULATION ===")
            
            # Reset simulation state
            self._current_frame = 1
            self._active_pedestrians = {}
            self._simulation_running = True
            
            # Create timer for 25 fps (0.04 seconds = 25 fps)
            self._simulation_timer = self.node.create_timer(
                0.04,  # 25 fps like original data
                self._simulation_step
            )
            
            self._logger.error("Simulation started at 25 fps")
            
        except Exception as e:
            self._logger.error(f"Failed to start simulation: {e}")


    def _simulation_step(self):
        """Single simulation step - called at 25 fps"""
        if not self._simulation_running:
            return
            
        try:
            # Update frame counter
            self._current_frame += 1
            
            # Get pedestrians for current frame
            current_peds = self._trajectory_loader.get_pedestrians_at_frame(self._current_frame)
            
            # Create and publish arena message
            self._publish_current_frame_pedestrians(current_peds)
            
            # Debug output every 25 frames (1 second)
            if self._current_frame % 25 == 0:
                self._logger.error(f"Frame {self._current_frame}: {len(current_peds)} pedestrians")
                
            # Stop after reasonable time (for testing)
            if self._current_frame > 15000:  # ~20 seconds at 25 fps
                self._stop_simulation()


        except Exception as e:
            self._logger.error(f"Simulation step error: {e}")



    def _publish_current_frame_pedestrians(self, current_peds):
        """Publish pedestrians for current frame"""
        try:
            # Create pedestrians message
            peds_msg = Pedestrians()
            peds_msg.header.frame_id = "map"
            peds_msg.header.stamp = self.node.get_clock().now().to_msg()
            
            # Add each pedestrian
            for ped_id, (x, y) in current_peds.items():
                arena_ped = self._create_arena_pedestrian(ped_id, x, y)
                peds_msg.pedestrians.append(arena_ped)
                
            # Publish
            self._arena_peds_publisher.publish(peds_msg)
            
        except Exception as e:
            self._logger.error(f"Failed to publish frame pedestrians: {e}")



    def _load_episode(self, episode_name: str = "eth"):
        """Load trajectory episode from CSV"""
        try:
            self._logger.error(f"Loading SocNav episode: {episode_name}")
            
            success = self._trajectory_loader.load_csv(episode_name)
            if success:
                self._episode_loaded = True
                stats = self._trajectory_loader.get_stats()
                self._logger.error(f"Episode loaded successfully: {stats}")
                return True
            else:
                self._logger.error(f"Failed to load episode: {episode_name}")
                return False
                
        except Exception as e:
            self._logger.error(f"Error loading episode: {e}")
            return False





    def _create_arena_pedestrian(self, ped_id: int, x: float, y: float) -> Pedestrian:
        """Create arena pedestrian )"""
        
        arena_ped = Pedestrian()
        arena_ped.name = f"socnav_ped_{ped_id}"
        arena_ped.id = ped_id
        
        # Raw coordinates - transformation handled by map.yaml
        arena_ped.position.position.x = x
        arena_ped.position.position.y = y
        arena_ped.position.position.z = 0.8
        
        # Orientation
        arena_ped.position.orientation.w = 1.0
        
        # Initial velocity
        arena_ped.twist.linear.x = 0.0
        arena_ped.twist.linear.y = 0.0
        arena_ped.twist.angular.z = 0.0
        
        # Animation state
        arena_ped.animation_state = Pedestrian.WALKING
        
        return arena_ped


    def _stop_simulation(self):
        """Stop the simulation"""
        self._simulation_running = False
        if hasattr(self, '_simulation_timer'):
            self._simulation_timer.destroy()
        self._logger.error("Simulation stopped")


















    def _spawn_obstacles_impl(
        self,
        obstacles: Sequence[Obstacle],
    ) -> Sequence[Obstacle | None]:
        """Spawn static obstacles"""
        self._logger.info(f"SocNav: spawn_obstacles_impl called with {len(obstacles)} obstacles")
        return obstacles

    def _spawn_dynamic_obstacles_impl(
        self,
        obstacles: Sequence[DynamicObstacle],
    ) -> Sequence[DynamicObstacle | None]:
        """Spawn dynamic obstacles (pedestrians)"""
        self._logger.info(f"SocNav: spawn_dynamic_obstacles_impl called with {len(obstacles)} dynamic obstacles")
        return obstacles

    def _remove_obstacles_impl(self) -> bool:
        """Remove obstacles implementation"""
        self._logger.info("SocNav: remove_obstacles_impl called")
        return True

    def _spawn_walls_impl(self, walls) -> bool:
        """Spawn walls implementation"""
        self._logger.info("SocNav: spawn_walls_impl called")
        return True

    def _spawn_robot_impl(self, robot) -> bool:
        """Spawn robot implementation"""
        self._logger.info("SocNav: spawn_robot_impl called")
        return True

    def _remove_robot_impl(self, name) -> bool:
        """Remove robot implementation"""
        self._logger.info("SocNav: remove_robot_impl called")
        return True

    def _move_robot_impl(self, name, pose) -> bool:
        """Move robot implementation"""
        self._logger.info("SocNav: move_robot_impl called")
        return True


    # def _test_arena_peds_publisher(self):
    #     '""Test the arena_peds publisher by publishing real pedestrians from the trajectory loader"""'
    #     try:
    #         self._logger.error("=== TESTING REAL PEDESTRIAN PUBLISHING ===")
            
    #         # Get pedestrians at frame 1
    #         frame_1_peds = self._trajectory_loader.get_pedestrians_at_frame(1)
    #         self._logger.error(f"Frame 1 has {len(frame_1_peds)} pedestrians")
            
    #         # Create pedestrians message
    #         peds_msg = Pedestrians()
    #         peds_msg.header.frame_id = "map"
    #         peds_msg.header.stamp = self.node.get_clock().now().to_msg()
            
    #         # Add each pedestrian
    #         for ped_id, (x, y) in frame_1_peds.items():
    #             arena_ped = self._create_arena_pedestrian(ped_id, x, y)
    #             peds_msg.pedestrians.append(arena_ped)
                
    #         # Publish
    #         self._arena_peds_publisher.publish(peds_msg)
            
    #         self._logger.error(f"Published {len(peds_msg.pedestrians)} real pedestrians!")
            
    #         # Log first pedestrian details
    #         if peds_msg.pedestrians:
    #             first_ped = peds_msg.pedestrians[0]
    #             self._logger.error(f"First ped: {first_ped.name} at ({first_ped.position.position.x}, {first_ped.position.position.y})")
            
    #     except Exception as e:
    #         self._logger.error(f"Real pedestrian test failed: {e}")