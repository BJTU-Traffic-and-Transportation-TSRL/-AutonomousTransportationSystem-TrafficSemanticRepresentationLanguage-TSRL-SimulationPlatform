"""
This file contains the implementation of the TrafficManager class, which manages the traffic simulation, including vehicle behavior updates,
decision making, and planning. It uses the provided roadgraph and vehicle information to generate trajectories for each vehicle in the simulation.

中文翻译：
这个文件包含TrafficManager类的实现，它管理交通模拟，包括车辆行为更新、决策和规划。它使用提供的道路图和车辆信息为模拟中的每个车辆生成轨迹。
"""

import copy
import time
import os
from typing import Dict, List, Union
from pynput import keyboard

# 7.26 导入read_stop_info.py中的函数
from read_stop_info import extract_stop_info

from common.observation import Observation
from common.vehicle import Behaviour, control_Vehicle, VehicleType, create_vehicle, create_vehicle_lastseen

from common.vehicle import get_pre_vehicle_status

from trafficManager.decision_maker.mcts_decision_maker import (
    EgoDecisionMaker,
    MultiDecisionMaker,
)
from planner.ego_vehicle_planner import EgoPlanner
from planner.multi_vehicle_planner import MultiVehiclePlanner
from predictor.simple_predictor import UncontrolledPredictor
from simModel.egoTracking.model import Model
from trafficManager.common.vehicle_communication import CommunicationManager,VehicleCommunicator
from trafficManager.decision_maker.abstract_decision_maker import AbstractEgoDecisionMaker, EgoDecision
from trafficManager.planner.abstract_planner import AbstractEgoPlanner, AbstractMultiPlanner
from trafficManager.predictor.abstract_predictor import AbstractPredictor
from utils.load_config import load_config
from utils.obstacles import StaticObstacle
from utils.roadgraph import AbstractLane, JunctionLane, NormalLane, RoadGraph
from utils import data_copy
from utils.trajectory import State, Trajectory

import logger


logging = logger.get_logger(__name__)

global KEY_INPUT
KEY_INPUT = ""

class TrafficManager:
    """
    TrafficManager is a class that manages the traffic simulation, including vehicle behavior updates,
    decision making, and planning. It uses the provided roadgraph and vehicle information to generate
    trajectories for each vehicle in the simulation.

    Attributes:
        sumo_model: The SUMO traffic simulation model.
        T: The current simulation time.
        lastseen_vehicles: A dictionary containing the last seen state of each vehicle.
        config: The configuration dictionary.
        predictor: An instance of the UncontrolledPredictor class.
        ego_decision: An instance of the EgoDecisionMaker class.
        ego_planner: An instance of the EgoPlanner class.
        multi_veh_planner: An instance of the MultiVehiclePlanner class.
    """

    def __init__(self,
                 model: Model,
                 predictor: AbstractPredictor = None,
                 ego_decision: AbstractEgoDecisionMaker = None,
                 ego_planner: AbstractEgoPlanner = None,
                 multi_decision=None,
                 multi_veh_planner: AbstractMultiPlanner = None,
                 config_file_path=None):
        self.sumo_model = model
        self.time_step = 0
        self.lastseen_vehicles = {} # 上一帧的车辆信息
        # 如果未提供配置文件路径，使用相对于当前文件的路径
        if config_file_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_file_path = os.path.join(current_dir, "config.yaml")
        self.config = load_config(config_file_path) # 交通管理配置文件
        self.last_decision_time = -self.config["DECISION_INTERVAL"]
        self.mul_decisions =None
        self._set_up_keyboard_listener()
        # 7.26 需停止车辆字典的初始化
        self.vehicles_with_stops = {}
        # 7.26 提取停车信息
        self.vehicles_with_stops = extract_stop_info(model.rouFile)
        # 8.18 承接使用模型的通信管理器控制参数,并控制是否开启通信功能
        self.if_traffic_communication = model.communication
        if self.if_traffic_communication:
            self.communication_manager = CommunicationManager()

        self.predictor = predictor if predictor is not None else UncontrolledPredictor()
        self.ego_decision = ego_decision if ego_decision is not None else EgoDecisionMaker()
        self.ego_planner = ego_planner if ego_planner is not None else EgoPlanner()
        self.multi_decision = multi_decision if multi_decision is not None else MultiDecisionMaker()
        self.multi_veh_planner = multi_veh_planner if multi_veh_planner is not None else MultiVehiclePlanner()

    def _set_up_keyboard_listener(self):

        def on_press(key):
            """
            This function is used to detect the key press from the keyboard.
            When the left arrow key or 'a' is pressed, the global variable KEY_INPUT is set to 'Left'.
            When the right arrow key or 'd' is pressed, the global variable KEY_INPUT is set to 'Right'.
            """
            global KEY_INPUT
            if key == keyboard.Key.left or key == keyboard.KeyCode.from_char(
                    'a'):
                KEY_INPUT = 'Left'
            elif key == keyboard.Key.right or key == keyboard.KeyCode.from_char(
                    'd'):
                KEY_INPUT = 'Right'

        listener = keyboard.Listener(on_press=on_press)
        listener.start()  # start to listen on a separate thread

    def plan(self, T: float, roadgraph: RoadGraph,
             vehicles_info: dict) -> Dict[int, Trajectory]:
        """
        This function plans the trajectories of vehicles in a given roadgraph. 
        It takes in the total time T, the roadgraph, and the vehicles_info as parameters. 
        It first listens for keyboard input and then extracts the ego car, current vehicles, 
        and uncontrolled vehicles from the vehicles_info. 
        It then updates the behavior of the ego car and current vehicles. 
        It then constructs the observation and predicts the behavior of the uncontrolled vehicles. 
        It then makes a decision for the ego car if the ego planner is enabled. 
        It then plans the trajectories of the vehicles and updates the last seen vehicles. 
        Finally, it returns the output trajectories.
        """
        global KEY_INPUT # 2025.5.20

        start = time.time()

        current_time_step = int(T / self.config["DT"])
        through_timestep = current_time_step - self.time_step

        """
        Perception module
        # 1. 感知模块：感知周围环境
        # 提取当前场景中的车辆信息
        """
        # 提取车辆信息
        # 8.3 修改提取车辆信息方法，添加停车信息添加方法
        # 8.19 新增提取车辆信息方法，添加通信信息提取方法，并将vehicle类更改为control_Vehicle
        vehicles = self.extract_vehicles(vehicles_info, roadgraph, T,
                                         through_timestep, self.sumo_model.sim_mode)

        # 提取历史轨迹信息
        history_tracks = self.extract_history_tracks(current_time_step,
                                                     vehicles)
        # 提取静态障碍物信息
        static_obs_list = self.extract_static_obstacles()
        # 构造观测信息
        observation = Observation(vehicles=list(vehicles.values()),
                                  history_track=history_tracks,
                                  static_obstacles=static_obs_list)

        """
        # Prediction Module
        # 2. 预测模块：预测其他车辆的行为
        未来可以在这里的通信模块中加入——计划通知模块
        """
        prediction = self.predictor.predict(observation, roadgraph,
                                            self.lastseen_vehicles,
                                            through_timestep, self.config)

        # Update Behavior
        for vehicle_id, vehicle in vehicles.items():
            # only vehicles in AoI will be controlled 只对AOI内的车辆进行控制
            if vehicle.vtype == VehicleType.OUT_OF_AOI:
                continue
            """
            更新AOI内和Ego车辆的行为
            8.4 vehicle.update_behaviour(roadgraph, KEY_INPUT)中加入停车行为更新
            8.12 新增vehicle.front_vehicle_status = get_pre_vehicle_status(vehicle, vehicles)
            以获取车辆前车状态
            """
            vehicle.front_vehicle_status = get_pre_vehicle_status(vehicle, vehicles)
            vehicle.update_behaviour(roadgraph, KEY_INPUT) # 更新车辆行为，不会对通信模块造成影响
            KEY_INPUT = ""

        # make sure ego car exists when EGO_PLANNER is used
        if self.config["EGO_PLANNER"]:
            ego_id = vehicles_info.get("egoCar")["id"]
            if ego_id is None:
                raise ValueError("Ego car is not found when EGO_PLANER is used.")

        """
        # 3. 决策模块：根据感知和预测结果，生成决策
            决策模块在self.config["USE_DECISION_MAKER"]=True时起效果
            该模块的self.ego_decision.make_decision默认不作用，除非添加特定的决策算法
        """
        ego_decision: EgoDecision = None
        # 如果使用Ego决策模块且当前时间步长大于上次决策时间步长
        if self.config["USE_DECISION_MAKER"] and T - self.last_decision_time >= self.config["DECISION_INTERVAL"]:
            if self.config["EGO_PLANNER"]:
                # 8.7 查看Ego决策模块
                ego_decision = self.ego_decision.make_decision(
                    observation, roadgraph, prediction)
            self.mul_decisions = self.multi_decision.make_decision(
                T, observation, roadgraph, prediction, self.config)
            self.last_decision_time = T

        """
        # 4. 规划模块：根据决策，生成轨迹
        """
        # 生成AOI内非Ego车的轨迹
        result_paths = self.multi_veh_planner.plan(observation, roadgraph,
                                                   prediction,
                                                   multi_decision=self.mul_decisions,
                                                   T=T, config=self.config)

        # an example of ego planner
        # 生成Ego车的轨迹
        if self.config["EGO_PLANNER"]:
            ego_path = self.ego_planner.plan(vehicles[ego_id], observation,
                                             roadgraph, prediction, T,
                                             self.config, ego_decision)
            result_paths[ego_id] = ego_path

        # Update Last Seen 更新最后看到的车辆信息
        output_trajectories = {}
        self.lastseen_vehicles = dict(
            (vehicle_id, vehicle)
            for vehicle_id, vehicle in vehicles.items()
            if vehicle.vtype != VehicleType.OUT_OF_AOI)
        for vehicle_id, trajectory in result_paths.items():
            self.lastseen_vehicles[vehicle_id].trajectory = trajectory
            output_trajectories[vehicle_id] = data_copy.deepcopy(trajectory)
            del output_trajectories[vehicle_id].states[0]

        # update self.T
        self.time_step = current_time_step

        logging.info(f"Current frame: {current_time_step}. One loop Time: {time.time() - start}")
        logging.info("------------------------------")

        return output_trajectories

    def extract_history_tracks(self, current_time_step: int,
                               vehicles) -> Dict[int, List[State]]:
        history_tracks = {}
        for vehicle_id in vehicles.keys():
            if vehicle_id not in self.lastseen_vehicles:
                continue
            
            # 7.27：添加边界检查
            trajectory_states = self.lastseen_vehicles[vehicle_id].trajectory.states
            start_idx = min(self.time_step, len(trajectory_states))
            end_idx = min(current_time_step, len(trajectory_states))
            
            if start_idx < end_idx:
                history_tracks[vehicle_id] = trajectory_states[start_idx:end_idx]
            else:
                history_tracks[vehicle_id] = []
                
        return history_tracks

    def extract_static_obstacles(self) -> List[StaticObstacle]:
        """extract static obstacles for planning

        Returns:
            List[StaticObstacle]: static obstacles in frame
        """
        static_obs_list = []
        return static_obs_list

    def extract_vehicles(
            self, vehicles_info: Dict, roadgraph: RoadGraph, T: float,
            through_timestep: int, sim_mode: str,
    ) -> Dict[int, control_Vehicle]:
        """
        Extracts vehicles from the provided information and returns them as separate dictionaries.

        Args:
            vehicle_info (dict): Dictionary containing information about the vehicles.
            roadgraph (RoadGraph): Road graph of the simulation.
            lastseen_vehicles (dict): Dictionary containing the last seen vehicles.
            T (float): Current time step.
            through_timestep (int): The number of timesteps the vehicle has been through.
            sumo_model (Any): The SUMO model containing vehicle type information.

        Returns:
            Tuple[Vehicle, Dict[int, Vehicle], Dict[int, Vehicle]]: A tuple containing the ego car, current vehicles, and uncontrolled vehicles.
        """
        vehicles = {}
        # 提取自车信息(未添加停车信息提取模块！！)
        ego_car = self.extract_ego_vehicle(vehicles_info, roadgraph, T,
                                           through_timestep,sim_mode)
        if ego_car is not None:
            vehicles[ego_car.id] = ego_car
        # 提取AOI内的其他车辆的信息
        # 8.3 添加对于停车信息的提取部分
        for vehicle in vehicles_info["carInAoI"]: #循环查看
            if not vehicle["xQ"]:# 如果车辆未出现在场景中，跳过此车，查看下一辆车
                continue
            # 如果车辆已出现在场景中，且之前有轨迹信息
            if vehicle["id"] in self.lastseen_vehicles  and \
                len(self.lastseen_vehicles[vehicle["id"]].trajectory.states)> through_timestep:
                last_state = self.lastseen_vehicles[
                    vehicle["id"]].trajectory.states[through_timestep]
                # 8.3 修改create_vehicle_lastseen方法，使其能够传递停车信息
                vehicles[vehicle["id"]] = create_vehicle_lastseen(
                    vehicle,
                    self.lastseen_vehicles[vehicle["id"]],
                    roadgraph,
                    T,
                    last_state,
                    VehicleType.IN_AOI,
                    sim_mode
                )
            # 如果车辆已出现在场景中，且之前没有轨迹信息
            else: 
                vtype_info = self.sumo_model.allvTypes[vehicle["vTypeID"]] # 查看此车的类型信息
                # 8.3 修改create_vehicle方法，使其能够传递停车信息
                # 8.19 新增：创建车辆通信功能
                vehicle_temp = create_vehicle(
                    vehicle, roadgraph, vtype_info, T, VehicleType.IN_AOI,self.if_traffic_communication,if_ego=False,communication_manager=self.communication_manager)
                # 8.20 新增：将非Ego车辆添加到通信管理器中
                if self.if_traffic_communication:
                    vehicle_temp.init_communication(self.communication_manager,if_egoCar=False)
                vehicles[vehicle["id"]] = vehicle_temp

        # 提取AOI外的车辆信息
        for vehicle in vehicles_info["outOfAoI"]:
            if not vehicle["xQ"]:
                continue
            vtype_info = self.sumo_model.allvTypes[vehicle["vTypeID"]]
            if roadgraph.get_lane_by_id(vehicle["laneIDQ"][-1]) is not None:
                # 8.19 新增：创建车辆通信功能
                vehicle_temp = create_vehicle(
                    vehicle, roadgraph, vtype_info, T, VehicleType.OUT_OF_AOI,self.if_traffic_communication,if_ego=False,communication_manager=self.communication_manager)
                # 8.20 新增：将非Ego车辆添加到通信管理器中
                if self.if_traffic_communication:
                    vehicle_temp.init_communication(self.communication_manager,if_egoCar=False)
                vehicles[vehicle["id"]] = vehicle_temp

        # 计算主车、AOI内车辆、场景内车辆数量
        ego_cnt = 1 if ego_car is not None else 0
        aoi_cnt = len([
            vehicle for vehicle in vehicles.values()
            if vehicle.vtype == VehicleType.IN_AOI
        ])
        sce_cnt = len([
            vehicle for vehicle in vehicles.values()
            if vehicle.vtype == VehicleType.OUT_OF_AOI
        ])
        logging.info(
            f"There's {ego_cnt} ego cars, {aoi_cnt} cars in AOI, and {sce_cnt} cars in scenario"
        )
        return vehicles

    def extract_ego_vehicle(self, vehicles_info, roadgraph, T,
                            through_timestep,sim_mode) -> Union[None, control_Vehicle]:
        if "egoCar" not in vehicles_info:
            return None

        ego_info = vehicles_info["egoCar"]
        if not ego_info["xQ"]:
            return None

        ego_id = ego_info["id"]
        if ego_id in self.lastseen_vehicles and \
            len(self.lastseen_vehicles[ego_id].trajectory.states)> through_timestep:
            last_state = self.lastseen_vehicles[ego_id].trajectory.states[
                through_timestep]
            ego_car = create_vehicle_lastseen(
                ego_info,
                self.lastseen_vehicles[ego_id],
                roadgraph,
                T,
                last_state,
                VehicleType.EGO,
                sim_mode
            )
        else:# 初次出现自车
            vtype_info = self.sumo_model.allvTypes[ego_info["vTypeID"]]
            # 8.19 新增：通信是否开启的控制参数 if_traffic_communication
            ego_car = create_vehicle(ego_info, roadgraph, vtype_info, T,
                                        VehicleType.EGO,self.if_traffic_communication,if_ego=True,communication_manager=self.communication_manager)
            # 8.19 新增：将自车添加到通信管理器中
            if self.if_traffic_communication:
                ego_car.init_communication(self.communication_manager,True)
                ego_car.communicator.send(f"SelfVehicle({ego_car.id});")

        return ego_car
