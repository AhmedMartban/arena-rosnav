from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.simulators.base_simulator import BaseSimulator

import rospkg
import os

from abc import abstractmethod
from pedsim_msgs.msg import Ped
from geometry_msgs.msg import Point
from task_generator.constants import Pedsim


import rospy
import random
import subprocess
import numpy as np
import math
import re
from scipy.spatial.transform import Rotation


from pedsim_srvs.srv import SpawnInteractiveObstacles, SpawnInteractiveObstaclesRequest,SpawnObstacle, SpawnObstacleRequest, SpawnPeds, SpawnPed
from pedsim_msgs.msg import InteractiveObstacle, AgentStates, Waypoints, LineObstacle, Ped, LineObstacles

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

from std_msgs.msg import Empty
from std_srvs.srv import Empty, SetBool, Trigger

from rospkg import RosPack

from task_generator.simulators.simulator_factory import SimulatorFactory
from tf.transformations import quaternion_from_euler
from task_generator.constants import Constants, Pedsim
from task_generator.utils import Utils
from nav_msgs.srv import GetMap

import xml.etree.ElementTree as ET

import io

T = Constants.WAIT_FOR_SERVICE_TIMEOUT

class PedsimManager():  

    def __init__(self, namespace: str, simulator: BaseSimulator):

        self.simulator = simulator

        self._ns_prefix = lambda *topic: os.path.join(namespace, *topic)
        self._goal_pub = rospy.Publisher(self._ns_prefix("/goal"), PoseStamped, queue_size=1, latch=True)

        self._robot_name = rospy.get_param("robot_model", "")

        rospy.wait_for_service("/pedsim_simulator/spawn_peds", timeout=T)
        rospy.wait_for_service("/pedsim_simulator/reset_all_peds", timeout=T)
        rospy.wait_for_service("/pedsim_simulator/remove_all_peds", timeout=T)
        rospy.wait_for_service("pedsim_simulator/respawn_peds" , timeout=T)
        rospy.wait_for_service("pedsim_simulator/respawn_interactive_obstacles" , timeout=T)
        rospy.wait_for_service("pedsim_simulator/remove_all_interactive_obstacles" , timeout=T)
        rospy.wait_for_service("pedsim_simulator/add_obstacle", timeout=T)
        
        self._spawn_peds_srv = rospy.ServiceProxy(
            "/pedsim_simulator/spawn_peds", SpawnPeds
        )
        self._remove_peds_srv = rospy.ServiceProxy(
            "/pedsim_simulator/remove_all_peds", SetBool
        )
        self._reset_peds_srv = rospy.ServiceProxy(
            "/pedsim_simulator/reset_all_peds", Trigger
        )
        self.__respawn_interactive_obstacles_srv = rospy.ServiceProxy(
        "pedsim_simulator/respawn_interactive_obstacles" ,SpawnInteractiveObstacles, persistent=True)
    
        self.__remove_all_interactive_obstacles_srv = rospy.ServiceProxy(
        "pedsim_simulator/remove_all_interactive_obstacles" ,Trigger)

        self.spawn_interactive_obstacles_srv = rospy.ServiceProxy(
        "pedsim_simulator/spawn_interactive_obstacles" ,SpawnInteractiveObstacles, persistent=True)

        self.__respawn_peds_srv = rospy.ServiceProxy(
            "pedsim_simulator/respawn_peds" , SpawnPeds, persistent=True)

        self._spawn_peds_srv = rospy.ServiceProxy(
            "pedsim_simulator/spawn_peds", SpawnPeds)

        self.__add_obstacle_srv = rospy.ServiceProxy(
            "pedsim_simulator/add_obstacle" ,SpawnObstacle, persistent=True)

        rospy.set_param("respawn_dynamic", True)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)
        rospy.Subscriber("/pedsim_simulator/simulated_waypoints", Waypoints, self.interactive_actor_poses_callback)
        rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.dynamic_actor_poses_callback)

        self.map_manager = None

        # pkg_path = RosPack().get_path('pedsim_gazebo_plugin')
        # default_actor_model_file = os.path.join(pkg_path, "models", "adult.sdf")

        # actor_model_file = rospy.get_param('~actor_model_file', default_actor_model_file)
        
        # with open(actor_model_file) as f:
        #     self.xml_string = f.read()

        #override
        self.xml_string = """
            <?xml version="1.0" ?>
            <sdf version="1.5">
            <model name="actor_model">
                <pose>0 0 0.75 0 0 0</pose>
                <link name="link">
                    <collision name="collision">
                    <geometry>
                        <box>
                        <size>.5 .5 1.5</size>
                        </box>
                    </geometry>
                    </collision>
                    <visual name="visual">
                    <geometry>
                        <box>
                        <size>.5 .5 1.5</size>
                        </box>
                    </geometry>
                    </visual>
                </link>
            </model>
        """

    def create_obstacle(self, dynamic, i, map_manager, forbidden_zones):
        self.map_manager = map_manager
        safe_distance = 0.5
        [x, y, theta] = self.map_manager.get_random_pos_on_map(safe_distance, forbidden_zones) # check later for the need of free indicies and map papram

        if dynamic == True :
            waypoints = np.array( [x, y, 1]).reshape(1, 3) # the first waypoint
            safe_distance = 0.1 # the other waypoints don't need to avoid robot
            for j in range(10): 
                dist = 0
                while dist < 8:
                    [x2, y2, theta2] = self.map_manager.get_random_pos_on_map( safe_distance, forbidden_zones)
                    dist = np.linalg.norm([waypoints[-1,0] - x2,waypoints[-1,1] - y2])
                waypoints = np.vstack([waypoints, [x2, y2, 1]])
            ped=np.array([i+1, [x, y, 0.0], waypoints],dtype=object)
        else:
            ped=np.array([i+1, [x, y, 0.0]],dtype=object)
        
        return ped

    def spawn_obstacles(self, obstacles, type="shelf", yaml="shelf.yaml", interaction_radius=0.0):
        srv = SpawnInteractiveObstacles()
        srv.InteractiveObstacles = []
        i = 0
        self.agent_topic_str=''   
        while i < len(obstacles) : 
            msg = InteractiveObstacle()
            obstacle = obstacles[i]
            msg.pose = Pose()
            msg.pose.position.x = obstacle[1][0]
            msg.pose.position.y = obstacle[1][1]
            msg.pose.position.z = obstacle[1][2]

            if interaction_radius == 0.0:
                self.agent_topic_str+=f',{self._ns_prefix()}pedsim_static_obstacle_{obstacle[0]}/0' 
            else:
                self.agent_topic_str+=f',{self._ns_prefix()}pedsim_interactive_obstacle_{obstacle[0]}/0'

            msg.type = type
            msg.interaction_radius = interaction_radius
            msg.yaml_path = os.path.join(
                rospkg.RosPack().get_path("arena-simulation-setup"),
                "obstacles", yaml
            )
            srv.InteractiveObstacles.append(msg)
            i = i+1

        max_num_try = 1
        i_curr_try = 0
        print("trying to call service with interactive obstacles: ")    

        while i_curr_try < max_num_try:
        # try to call service
            response=self.spawn_interactive_obstacles_srv.call(srv.InteractiveObstacles)

            if not response.success:  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn static obstacle failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                i_curr_try += 1
            else:
                break
        rospy.set_param(f'{self._ns_prefix()}agent_topic_string', self.agent_topic_str)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)
        return
    
    def spawn_dynamic_obstacles(self, peds, type="adult", yaml="person_two_legged.model.yaml"):
        srv = SpawnPeds()
        srv.peds = []
        i = 0
        self.agent_topic_str=''   
        while i < len(peds) : 
            msg = Ped()
            ped = peds[i]
            msg.id = ped[0]+20

            msg.pos = Point()
            msg.pos.x = ped[1][0]
            msg.pos.y = ped[1][1]
            msg.pos.z = ped[1][2]

            self.agent_topic_str+=f',pedsim_agent_{ped[0]}/0' 
            msg.type = type
            msg.yaml_file = os.path.join(
                rospkg.RosPack().get_path("arena-simulation-setup"),
                "dynamic_obstacles",
                yaml
            )
            msg.number_of_peds = 1
            msg.vmax = Pedsim.VMAX
            msg.start_up_mode = Pedsim.START_UP_MODE
            msg.wait_time = Pedsim.WAIT_TIME
            msg.trigger_zone_radius = Pedsim.TRIGGER_ZONE_RADIUS
            msg.chatting_probability = Pedsim.CHATTING_PROBABILITY
            msg.tell_story_probability = Pedsim.TELL_STORY_PROBABILITY
            msg.group_talking_probability = Pedsim.GROUP_TALKING_PROBABILITY
            msg.talking_and_walking_probability = Pedsim.TALKING_AND_WALKING_PROBABILITY
            msg.requesting_service_probability = Pedsim.REQUESTING_SERVICE_PROBABILITY
            msg.requesting_guide_probability = Pedsim.REQUESTING_GUIDE_PROBABILITY
            msg.requesting_follower_probability = Pedsim.REQUESTING_FOLLOWER_PROBABILITY
            msg.max_talking_distance = Pedsim.MAX_TALKING_DISTANCE
            msg.max_servicing_radius = Pedsim.MAX_SERVICING_RADIUS
            msg.talking_base_time = Pedsim.TALKING_BASE_TIME
            msg.tell_story_base_time = Pedsim.TELL_STORY_BASE_TIME
            msg.group_talking_base_time = Pedsim.GROUP_TALKING_BASE_TIME
            msg.talking_and_walking_base_time = Pedsim.TALKING_AND_WALKING_BASE_TIME
            msg.receiving_service_base_time = Pedsim.RECEIVING_SERVICE_BASE_TIME
            msg.requesting_service_base_time = Pedsim.REQUESTING_SERVICE_BASE_TIME
            msg.force_factor_desired = Pedsim.FORCE_FACTOR_DESIRED
            msg.force_factor_obstacle = Pedsim.FORCE_FACTOR_OBSTACLE
            msg.force_factor_social = Pedsim.FORCE_FACTOR_SOCIAL
            msg.force_factor_robot = Pedsim.FORCE_FACTOR_ROBOT
            msg.waypoint_mode = Pedsim.WAYPOINT_MODE 

            msg.waypoints = []

            for pos in ped[2]:
                p = Point()
                p.x = pos[0]
                p.y = pos[1]
                p.z = pos[2]
                msg.waypoints.append(p)
            srv.peds.append(msg)
            i = i+1

        max_num_try = 1
        i_curr_try = 0
        while i_curr_try < max_num_try:
        # try to call service
            response=self.__respawn_peds_srv.call(srv.peds)

            if not response.success:  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                i_curr_try += 1
            else:
                break
        rospy.set_param(f'{self._ns_prefix()}agent_topic_string', self.agent_topic_str)
        rospy.set_param("respawn_dynamic", True)

    def spawn_map_obstacles(self):
        map = rospy.get_param("map_file")
        map_path = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"), 
            "worlds", 
            map,
            "ped_scenarios",
            f"{map}.xml"
        )
        tree = ET.parse(map_path)
        root = tree.getroot()

        forbidden_zones = []

        add_pedsim_srv=SpawnObstacleRequest()
        for child in root:
            lineObstacle=LineObstacle()
            lineObstacle.start.x,lineObstacle.start.y=float(child.attrib['x1']),float(child.attrib['y1'])
            lineObstacle.end.x,lineObstacle.end.y=float(child.attrib['x2']),float(child.attrib['y2'])
            add_pedsim_srv.staticObstacles.obstacles.append(lineObstacle)
            forbidden_zones.append([lineObstacle.start.x, lineObstacle.start.y, 1])
            forbidden_zones.append([lineObstacle.end.x, lineObstacle.end.y, 1])

        self.__add_obstacle_srv.call(add_pedsim_srv)
        return forbidden_zones

    # SCENARIO INTEGRATION
    def spawn_dynamic_scenario_obstacles(self, peds):
        srv = SpawnPeds()
        srv.peds = []
        i = 0
        self.agent_topic_str=''   
        while i < len(peds) : 
            ped = peds[i]
            msg = Ped()
            msg.id = i 

            msg.pos = Point()
            msg.pos.x = ped["pos"][0]
            msg.pos.y = ped["pos"][1]
            msg.pos.z = 0

            msg.waypoints = []
            for pos in ped["waypoints"]:
                p = Point()
            p.x = pos[0]
            p.y = pos[1]
            p.z = 0
            msg.waypoints.append(p)
            msg.yaml_file = os.path.join(
                rospkg.RosPack().get_path("arena-simulation-setup"),
                "dynamic_obstacles",
                "person_two_legged.model.yaml")

            self.agent_topic_str+=f',pedsim_agent_{i}/0' 
            msg.type = "adult"
            msg.number_of_peds = 1
            msg.vmax = ped["vmax"]
            msg.start_up_mode = ped["start_up_mode"]
            msg.wait_time = ped["wait_time"]
            msg.trigger_zone_radius = ped["trigger_zone_radius"]
            msg.chatting_probability = ped["chatting_probability"]
            msg.tell_story_probability = ped["tell_story_probability"]
            msg.group_talking_probability = ped["group_talking_probability"]
            msg.talking_and_walking_probability = ped["talking_and_walking_probability"]
            msg.requesting_service_probability = ped["requesting_service_probability"]
            msg.requesting_guide_probability = ped["requesting_guide_probability"]
            msg.requesting_follower_probability = ped["requesting_follower_probability"]
            msg.max_talking_distance = ped["max_talking_distance"]
            msg.max_servicing_radius = ped["max_servicing_radius"]
            msg.talking_base_time = ped["talking_base_time"]
            msg.tell_story_base_time = ped["tell_story_base_time"]
            msg.group_talking_base_time = ped["group_talking_base_time"]
            msg.talking_and_walking_base_time = ped["talking_and_walking_base_time"]
            msg.receiving_service_base_time = ped["receiving_service_base_time"]
            msg.requesting_service_base_time = ped["requesting_service_base_time"]
            msg.force_factor_desired = ped["force_factor_desired"]
            msg.force_factor_obstacle = ped["force_factor_obstacle"]
            msg.force_factor_social = ped["force_factor_social"]
            msg.force_factor_robot = ped["force_factor_robot"]
            msg.waypoint_mode = ped["waypoint_mode"] # or 1 check later

            srv.peds.append(msg)
            i = i+1

        max_num_try = 1
        i_curr_try = 0
        while i_curr_try < max_num_try:
        # try to call service
            response=self.__respawn_peds_srv.call(srv.peds)

            if not response.success:  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                # rospy.logwarn(response.message)
                i_curr_try += 1
            else:
                break
        self._peds = peds
        rospy.set_param(f'{self._ns_prefix()}agent_topic_string', self.agent_topic_str)
        rospy.set_param("respawn_dynamic", True)
        return

    def spawn_scenario_obstacles(self, obstacles, interaction_radius=0.0):
        srv = SpawnInteractiveObstacles()
        srv.InteractiveObstacles = []
        i = 0
        self.agent_topic_str=''   
        while i < len(obstacles) : 
            msg = InteractiveObstacle()
            obstacle = obstacles[i]
            # msg.id = obstacle[0]

            msg.pose = Pose()
            msg.pose.position.x = obstacle["pos"][0]
            msg.pose.position.y = obstacle["pos"][1]
            msg.pose.position.z = 0

            self.agent_topic_str+=f',{self._ns_prefix()}pedsim_static_obstacle_{i}/0' 
            msg.type = "shelf"
            msg.interaction_radius = interaction_radius
            msg.yaml_path = os.path.join(
                rospkg.RosPack().get_path("arena-simulation-setup"),
                "obstacles", "shelf.yaml"
            )
            srv.InteractiveObstacles.append(msg)
            i = i+1

        max_num_try = 1
        i_curr_try = 0
        print("trying to call service with static obstacles: ")    

        while i_curr_try < max_num_try:
        # try to call service
            response=self.spawn_interactive_obstacles_srv.call(srv.InteractiveObstacles)

            if not response.success:  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                i_curr_try += 1
            else:
                break
        # self._peds.append(peds)
        rospy.set_param(f'{self._ns_prefix()}agent_topic_string', self.agent_topic_str)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)
        return

    def remove_interactive_obstacles(self):
        self.__remove_all_interactive_obstacles_srv.call()
    
    def interactive_actor_poses_callback(self, actors):
        if rospy.get_param("respawn_interactive"):
            for actor in actors.waypoints:
                if "interactive" in actor.name:
                    actor_name = str(actor.name)
                    orientation = float( re.findall(r'\(.*?\)', str(actor.name))[0].replace("(","").replace(")","").replace(",","."))
                    direction_x = float( actor.name[actor.name.index("{")+1: actor.name.index("}")].replace(",","."))
                    direction_y = float( actor.name[actor.name.index("[")+1: actor.name.index("]")].replace(",","."))
                    ob_type =            actor.name[actor.name.index("&")+1: actor.name.index("!")]
                    rot = Rotation.from_euler('xyz', [0, 0, orientation], degrees=False)
                    rot_quat = rot.as_quat()

                    rospy.loginfo("Spawning interactive: actor_id = %s", actor_name)

                    rospack1 = RosPack()
                    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
                    
                    z = os.path.join(pkg_path, "models", f"{ob_type}.sdf")

                    with open(z) as file_xml:
                        x = file_xml.read()

                    model_pose =  Pose(Point(x= actor.position.x-direction_x,
                                        y= actor.position.y-direction_y,
                                        z= actor.position.z)
                                        ,
                                    Quaternion(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]) )

                    self.simulator.spawn_model(actor_name, x, "", model_pose, "world")
                    self.simulator.spawned_obstacles.append(actor_name)
                    rospy.set_param("respawn_interactive", False)
                
        if rospy.get_param("respawn_static"):
            for actor in actors.waypoints:
                if "static" in actor.name:

                    actor_name = str(actor.name)
                    orientation = float( re.findall(r'\(.*?\)', str(actor.name))[0].replace("(","").replace(")","").replace(",","."))
                    direction_x = float( actor.name[actor.name.index("{")+1: actor.name.index("}")].replace(",","."))
                    direction_y = float( actor.name[actor.name.index("[")+1: actor.name.index("]")].replace(",","."))
                    ob_type = actor.name[actor.name.index("&")+1: actor.name.index("!")]
                    
                    rot = Rotation.from_euler('xyz', [0, 0, orientation], degrees=False)
                    rot_quat = rot.as_quat()
                    
                    rospy.loginfo("Spawning static: actor_id = %s", actor_name)

                    rospack1 = RosPack()
                    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
                    z = pkg_path +  "/models/table.sdf"
                    file_xml = open(z)
                    x = file_xml.read()

                    model_pose =  Pose(Point(x= actor.position.x-direction_x,
                                            y= actor.position.y-direction_y,
                                            z= actor.position.z)
                                            ,
                                    Quaternion(rot_quat[0],
                                                rot_quat[1],
                                                rot_quat[2],
                                                rot_quat[3]) )

                    self.simulator.spawn_model(actor_name, x, "", model_pose, "world")
                    self.simulator.spawned_obstacles.append(actor_name)
                    rospy.set_param("respawn_static", False)


    def dynamic_actor_poses_callback(self, actors):
        if rospy.get_param("respawn_dynamic"):
            for actor in actors.agent_states:
                actor_id = str(actor.id)
                actor_pose = actor.pose
                rospy.loginfo("Spawning dynamic obstacle: actor_id = %s", actor_id)

                model_pose = Pose(Point(x= actor_pose.position.x,
                                    y= actor_pose.position.y,
                                    z= actor_pose.position.z),
                                Quaternion(actor_pose.orientation.x,
                                            actor_pose.orientation.y,
                                            actor_pose.orientation.z,
                                            actor_pose.orientation.w) )
                self.simulator.spawn_model(actor_id, self.xml_string, "", model_pose, "world")
                self.simulator.spawned_obstacles.append(actor_id)
                rospy.set_param("respawn_dynamic", False)