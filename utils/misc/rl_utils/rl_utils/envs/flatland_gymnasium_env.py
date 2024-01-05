#! /usr/bin/env python3
import re
import time
import random

from typing import Tuple

import gymnasium
import numpy as np
import rospy
from flatland_msgs.msg import StepWorld
from geometry_msgs.msg import Twist
from rl_utils.utils.observation_collector.constants import DONE_REASONS
from rl_utils.utils.observation_collector.observation_manager import ObservationManager
from rl_utils.utils.rewards.reward_function import RewardFunction
from rosnav.model.base_agent import BaseAgent
from rosnav.rosnav_space_manager.rosnav_space_manager import RosnavSpaceManager
from std_srvs.srv import Empty
from task_generator.shared import Namespace
from task_generator.task_generator_node import TaskGenerator
from task_generator.tasks.base_task import BaseTask
from task_generator.utils import rosparam_get


def get_ns_idx(ns: str):
    try:
        return int(re.search(r"\d+", ns)[0])
    except Exception:
        return random.uniform(0, 3)
        # return 0.5


def delay_node_init(ns):
    try:
        # given every environment enough time to initialize, if we dont put sleep,
        # the training script may crash.

        ns_int = get_ns_idx(ns)
        time.sleep((ns_int + 1.5) * 2.2)
        # time.sleep((random.randint(2, 6) + random.uniform(0, 3) * 2))
    except Exception:
        rospy.logwarn(
            "Can't not determinate the number of the environment, training script may crash!"
        )
        time.sleep(2)


class FlatlandEnv(gymnasium.Env):
    """
    FlatlandEnv is an environment class that represents a Flatland environment for reinforcement learning.

    Args:
        ns (str): The namespace of the environment.
        agent_description (BaseAgent): The agent description.
        reward_fnc (str): The name of the reward function.
        max_steps_per_episode (int): The maximum number of steps per episode.
        *args: Additional positional arguments.
        **kwargs: Additional keyword arguments.

    Attributes:
        metadata (dict): The metadata of the environment.
        ns (Namespace): The namespace of the environment.
        _agent_description (BaseAgent): The agent description.
        _is_train_mode (bool): Flag indicating if the environment is in training mode.
        model_space_encoder (RosnavSpaceManager): The space encoder for the model.
        task (BaseTask): The task manager.
        reward_calculator (RewardFunction): The reward calculator.
        agent_action_pub (rospy.Publisher): The publisher for agent actions.
        _service_name_step (str): The name of the step world service.
        _step_world_srv (rospy.ServiceProxy): The step world service proxy.
        observation_collector (ObservationManager): The observation collector.
        _steps_curr_episode (int): The current number of steps in the episode.
        _episode (int): The current episode number.
        _max_steps_per_episode (int): The maximum number of steps per episode.
        _last_action (np.ndarray): The last action taken by the agent.
    """

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        ns: str,
        agent_description: BaseAgent,
        reward_fnc: str,
        max_steps_per_episode=100,
        *args,
        **kwargs,
    ):
        super(FlatlandEnv, self).__init__()

        self.ns = Namespace(ns)
        self._agent_description = agent_description

        if not rospy.get_param("/debug_mode", False):
            delay_node_init(ns=self.ns.simulation_ns)
            rospy.init_node(f"env_{self.ns}".replace("/", "_"), anonymous=True)

        self._is_train_mode = rosparam_get(bool, "/train_mode", default=True)
        self._step_size = rosparam_get(float, "/step_size")

        self.model_space_encoder = RosnavSpaceManager(
            space_encoder_class=self._agent_description.space_encoder_class,
            observation_spaces=self._agent_description.observation_spaces,
            observation_space_kwargs=self._agent_description.observation_space_kwargs,
        )

        if self._is_train_mode:
            self._setup_env_for_training(reward_fnc, **kwargs)

        # observation collector
        self.observation_collector = ObservationManager(self.ns)

        self._steps_curr_episode = 0
        self._episode = 0
        self._max_steps_per_episode = max_steps_per_episode
        self._last_action = np.array([0, 0, 0])  # linear x, linear y, angular z

    @property
    def action_space(self):
        return self.model_space_encoder.get_action_space()

    @property
    def observation_space(self):
        return self.model_space_encoder.get_observation_space()

    def _setup_env_for_training(self, reward_fnc: str, **kwargs):
        if not rospy.get_param("/debug_mode", False):
            delay_node_init(self.ns.simulation_ns)
        # instantiate task manager
        task_generator = TaskGenerator(self.ns.simulation_ns)
        self.task: BaseTask = task_generator._get_predefined_task(**kwargs)

        # reward calculator
        self.reward_calculator = RewardFunction(
            rew_func_name=reward_fnc,
            holonomic=self.model_space_encoder._is_holonomic,
            robot_radius=self.task.robot_managers[0]._robot_radius,
            safe_dist=self.task.robot_managers[0].safe_distance,
            goal_radius=rosparam_get(float, "goal_radius", 0.3),
        )

        self.agent_action_pub = rospy.Publisher(self.ns("cmd_vel"), Twist, queue_size=1)

        # service clients
        self._service_name_step = self.ns.simulation_ns("step_world")
        self._step_world_publisher = rospy.Publisher(
            self._service_name_step, StepWorld, queue_size=10
        )
        self._step_world_srv = rospy.ServiceProxy(
            self._service_name_step, Empty, persistent=True
        )

    def _pub_action(self, action: np.ndarray) -> Twist:
        assert len(action) == 3

        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]

        self.agent_action_pub.publish(action_msg)

    def _decode_action(self, action: np.ndarray) -> np.ndarray:
        return self.model_space_encoder.decode_action(action)

    def _encode_observation(self, observation, *args, **kwargs):
        return self.model_space_encoder.encode_observation(observation, **kwargs)

    def step(self, action: np.ndarray):
        """
        Take a step in the environment.

        Args:
            action (np.ndarray): The action to take.

        Returns:
            tuple: A tuple containing the encoded observation, reward, done flag, info dictionary, and False flag.

        """

        decoded_action = self._decode_action(action)
        self._pub_action(decoded_action)

        if self._is_train_mode:
            self.call_service_takeSimStep()

        obs_dict = self.observation_collector.get_observations(
            last_action=self._last_action
        )
        self._last_action = decoded_action

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            action=decoded_action,
            **obs_dict,
        )

        self._steps_curr_episode += 1

        # info
        info, done = self._determine_termination(
            reward_info=reward_info,
            curr_steps=self._steps_curr_episode,
            max_steps=self._max_steps_per_episode,
        )

        return (
            self._encode_observation(obs_dict, is_done=done),
            reward,
            done,
            False,
            info,
        )

    def call_service_takeSimStep(self, t: float = None):
        request = StepWorld()
        request.required_time = self._step_size if t is None else t

        # self._step_world_srv()

        self._step_world_publisher.publish(request)

    def reset(self, seed=None, options=None):
        """
        Reset the environment.

        Args:
            seed: The random seed for the environment.
            options: Additional options for resetting the environment.

        Returns:
            tuple: A tuple containing the encoded observation and an empty info dictionary.

        """

        super().reset(seed=seed)
        self._episode += 1
        self.agent_action_pub.publish(Twist())

        first_map = self._episode <= 1 if "sim_1" in self.ns else False
        self.task.reset(
            callback=lambda: False,
            first_map=first_map,
            reset_after_new_map=self._steps_curr_episode == 0,
        )
        self.reward_calculator.reset()
        self._steps_curr_episode = 0
        self._last_action = np.array([0, 0, 0])

        if self._is_train_mode:
            for _ in range(5):
                self.agent_action_pub.publish(Twist())
                self.call_service_takeSimStep(t=0.1)
                time.sleep(0.15)

        obs_dict = self.observation_collector.get_observations()
        info_dict = {}
        return (
            self._encode_observation(obs_dict),
            info_dict,
        )

    def close(self):
        """
        Close the environment.

        """
        pass

    def _determine_termination(
        self,
        reward_info: dict,
        curr_steps: int,
        max_steps: int,
        info: dict = None,
    ) -> Tuple[dict, bool]:
        """
        Determine if the episode should terminate.

        Args:
            reward_info (dict): The reward information.
            curr_steps (int): The current number of steps in the episode.
            max_steps (int): The maximum number of steps per episode.
            info (dict): Additional information.

        Returns:
            tuple: A tuple containing the info dictionary and a boolean flag indicating if the episode should terminate.

        """

        if info is None:
            info = {}

        terminated = reward_info["is_done"]

        if terminated:
            info["done_reason"] = reward_info["done_reason"]
            info["is_success"] = reward_info["is_success"]
            info["episode_length"] = self._steps_curr_episode

        if curr_steps >= max_steps:
            terminated = True
            info["done_reason"] = DONE_REASONS.STEP_LIMIT.name
            info["is_success"] = 0
            info["episode_length"] = self._steps_curr_episode

        return info, terminated
