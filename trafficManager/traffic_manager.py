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
from common.vehicle import Behaviour, control_Vehicle,VehicleType, create_vehicle, create_vehicle_lastseen, get_pre_vehicle_status
from common.facility import control_RSU, create_rsu, create_rsu_lastseen, RSUType

from trafficManager.decision_maker.TSRL_decision_maker import (
    EgoDecisionMaker,
    MultiDecisionMaker,
)
from planner.TSRL_ego_vehicle_planner import EgoPlanner
from planner.TSRL_multi_vehicle_planner import MultiVehiclePlanner
from predictor.simple_predictor import UncontrolledPredictor
from simModel.egoTracking.model import Model
from TSRL_interaction.vehicle_communication import CommunicationManager, Performative
from TSRL_interaction.communicator_category import RSUCommunicator, VehicleCommunicator, EnvCommunicator
from trafficManager.common.environment_adapter import EnvironmentAdapter
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
        self.lastseen_facilities = {} # 上一帧的设施(RSU)信息
        # 9.15 初始化RSU查询记录集合
        self.queried_rsus = set() # 记录已发送询问消息的RSU
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
        if self.sumo_model.sim_mode =='RealTime':
            self.vehicles_with_stops = extract_stop_info(model.rouFile)
        # 8.18 承接使用模型的通信管理器控制参数,并控制是否开启通信功能
        self.if_traffic_communication = model.communication
        if self.if_traffic_communication:
            self.communication_manager = CommunicationManager(self.sumo_model.Scenario_Name)
            # 初始化环境通信器，用于发送交叉口信息
            self.env_adapter = EnvironmentAdapter(self.sumo_model)
            self.env_communicator = EnvCommunicator(
                env_id="traffic_env",
                communication_manager=self.communication_manager
            )
            # 设置环境适配器
            self.env_communicator.set_context(self.env_adapter)

        self.predictor = predictor if predictor is not None else UncontrolledPredictor()
        self.ego_decision = ego_decision if ego_decision is not None else EgoDecisionMaker(self.sumo_model.Scenario_Name)
        self.ego_planner = ego_planner if ego_planner is not None else EgoPlanner()
        self.multi_decision = multi_decision if multi_decision is not None else MultiDecisionMaker(self.sumo_model.Scenario_Name)
        self.multi_veh_planner = multi_veh_planner if multi_veh_planner is not None else MultiVehiclePlanner()
        
        # 9.9 注册GUI输入回调
        if hasattr(self.sumo_model, 'gui') and self.sumo_model.gui:
            self.sumo_model.gui.set_input_callback(self._handle_user_input)
        
        # 9.9 存储用户输入
        self.user_command = ""
        
        # 添加标志：是否已发送交叉口信息
        self.junction_info_sent = False

    # 从键盘中获取用户的输入
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
            elif key == keyboard.Key.up:
                KEY_INPUT = 'LetAccelerate'

        listener = keyboard.Listener(on_press=on_press)
        listener.start()  # start to listen on a separate thread
    
    # 9.9 新增处理用户输入的方法
    def _handle_user_input(self, user_input: str):
        """处理用户输入"""
        # 读取来自tkinter_scenario_selector的键盘输入信号
        try:
            self.user_command = user_input
            os.remove(key_input_file)
            logging.info(f"Read key input from {key_input_file}: {KEY_INPUT}")
        except Exception as e:
            logging.warning(f"Failed to read key input file: {e}")
        self.user_command = user_input
        logging.info(f"Received user command: {user_input}")
    
    def plan(self, T: float, roadgraph: RoadGraph,
             vehicles_info: dict, facilities: dict) -> Dict[int, Trajectory]:
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
        # 9.12 提取道路设备信息
        facilities = self.extract_facilities(facilities, roadgraph)
        # 9.16 处理RSU与Ego车辆的交互
        self._handle_rsu_ego_interaction(vehicles, facilities, roadgraph, current_time_step)
        # 发送交叉口信息（只在开始时发送一次）
        if self.if_traffic_communication and hasattr(self, 'env_communicator') and not self.junction_info_sent:
            self._send_junction_info()
            self.junction_info_sent = True
        # 提取历史轨迹信息
        history_tracks = self.extract_history_tracks(current_time_step,
                                                     vehicles)
        # 提取静态障碍物信息
        static_obs_list = self.extract_static_obstacles()
        # 构造观测信息
        observation = Observation(vehicles=list(vehicles.values()),
                                  history_track=history_tracks,
                                  static_obstacles=static_obs_list
                                  )
        """
        # Prediction Module
        # 2. 预测模块：预测其他车辆的行为
        未来可以在这里的通信模块中加入“计划+通知”模块
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
            # 9.9 使用用户输入的命令更新车辆行为
            if vehicle_id == self.sumo_model.ego.id and self.user_command:
                vehicle.update_behaviour(roadgraph, self.user_command, vehicles)
                self.user_command = ""  # 清除已处理的命令
            else:
                vehicle.update_behaviour(roadgraph, KEY_INPUT, vehicles) # 更新车辆行为，不会对通信模块造成影响
            KEY_INPUT = ""
            
            # Set context for vehicle communicators to enable EmergencyStation processing
            if self.if_traffic_communication and vehicle.communicator and hasattr(vehicle.communicator, 'set_context'):
                vehicle.communicator.set_context(vehicles, roadgraph)

        # make sure ego car exists when EGO_PLANNER is used
        if self.config["EGO_PLANNER"]:
            # 修复：添加对egoCar是否存在的检查
            ego_car_info = vehicles_info.get("egoCar")
            if ego_car_info is None or "id" not in ego_car_info:
                raise ValueError("Ego car information is not found when EGO_PLANNER is used.")
            ego_id = ego_car_info["id"]
            if ego_id is None:
                raise ValueError("Ego car is not found when EGO_PLANNER is used.")
        """
        # 3. 决策模块：根据感知和预测结果，生成AOI内各车辆的决策
            决策模块在self.config["USE_DECISION_MAKER"]=True时起效果
            该模块的self.ego_decision.make_decision默认不作用，除非添加特定的决策算法
            9.18 尝试将Ego_ego_decision.make_decision更改为TSRL决策模块
        """
        ego_decision: EgoDecision = None
        # 如果使用Vehicle决策模块且当前时间步长距离上次决策的时间大于等于设置的决策间隔
        if self.config["USE_DECISION_MAKER"] and T - self.last_decision_time >= self.config["DECISION_INTERVAL"]:
            if self.config["EGO_PLANNER"]:
                # if EGO_PLANNER is determined to be used, then make decision for ego car
                ego_decision = self.ego_decision.make_decision(
                    T , observation, roadgraph, prediction, self.config)
            # if USE_DECISION_MAKER is determined to be used, then make decision for other vehicles
            self.mul_decisions = self.multi_decision.make_decision(
                T, observation, roadgraph, prediction, self.config)
            self.last_decision_time = T
        """
        # 4. 轨迹生成模块：根据决策，生成轨迹
        """
        # 生成AOI内非Ego车的轨迹
        result_paths = self.multi_veh_planner.plan(observation, roadgraph,
                                                   prediction,
                                                   multi_decision=self.mul_decisions,
                                                   T=T, config=self.config)
        # an example of ego planner
        # 生成Ego车的轨迹
        if self.config["EGO_PLANNER"]:
            # 修复：添加对ego_id是否在vehicles中的检查
            if ego_id not in vehicles:
                raise ValueError(f"Ego vehicle with id {ego_id} not found in vehicles.")
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
        # 更新最后看到的设施(RSU)信息
        self.lastseen_facilities = dict(
            (rsu_id, rsu)
            for rsu_id, rsu in facilities.items())
        for vehicle_id, trajectory in result_paths.items():
            self.lastseen_vehicles[vehicle_id].trajectory = trajectory
            # 检查trajectory是否为None，避免AttributeError
            if trajectory is not None:
                output_trajectories[vehicle_id] = data_copy.deepcopy(trajectory)
                # 检查states列表是否为空，避免索引越界
                if hasattr(output_trajectories[vehicle_id], 'states') and len(output_trajectories[vehicle_id].states) > 0:
                    del output_trajectories[vehicle_id].states[0]
                else:
                    logging.warning(f"Vehicle {vehicle_id} has empty trajectory states or no states attribute")
            else:
                logging.warning(f"Vehicle {vehicle_id} has None trajectory, skipping")
                output_trajectories[vehicle_id] = Trajectory()  # 创建空轨迹作为默认值

        # update self.T
        self.time_step = current_time_step
        logging.info(f"Current frame: {current_time_step}. One loop Time: {time.time() - start}")
        logging.info("------------------------------")
        return output_trajectories
    
    def _send_junction_info(self):
        """
        发送交叉口信息到通信系统
        通过EnvCommunicator主动发送交叉口信息
        读取net.xml文件，过滤掉type="dead_end"的junction
        """
        try:
            # 获取网络文件路径
            net_file_path = self.sumo_model.netFile
            
            # 解析net.xml文件
            import xml.etree.ElementTree as ET
            tree = ET.parse(net_file_path)
            root = tree.getroot()
            
            # 查找所有junction元素，过滤掉type="dead_end"的junction
            junctions = root.findall('.//junction')
            valid_junction_ids = []
            
            for junction in junctions:
                junction_id = junction.get('id')
                junction_type = junction.get('type', '')  # 获取type属性，如果不存在则为空字符串
                
                # 只发送非dead_end和internal类型的junction
                if junction_type != 'dead_end' and junction_type != 'internal':
                    valid_junction_ids.append(junction_id)
            
            # 为每个有效的交叉口单独发送消息
            if self.env_communicator:
                for junction_id in valid_junction_ids:
                    message_content = f"IsJunction({junction_id});"
                    self.env_communicator.send(message_content, performative=Performative.Inform)
                
                logging.info(f"Sent junction info: {len(valid_junction_ids)} junctions (filtered out dead_end junctions)")
        except Exception as e:
            logging.error(f"Error sending junction info: {e}")
            # 如果读取net.xml失败，回退到原来的方法
            try:
                junction_ids = self.env_adapter.get_junction_ids()
                if self.env_communicator:
                    for junction_id in junction_ids:
                        message_content = f"IsJunction({junction_id});"
                        self.env_communicator.send(message_content, performative=Performative.Inform)
                    
                    logging.info(f"Sent junction info (fallback): {len(junction_ids)} junctions")
            except Exception as fallback_error:
                logging.error(f"Fallback method also failed: {fallback_error}")
    
    # 9.16新增 处理RSU与Ego车辆的交互
    def _handle_rsu_ego_interaction(self, vehicles: Dict[str, control_Vehicle], 
                                   facilities: Dict[str, control_RSU], 
                                   roadgraph: RoadGraph, 
                                   current_time_step: int):
        """
        处理RSU与Ego车辆的交互
        1. 提取路侧设备信息
        2. 检查Ego车辆是否进入RSU探测范围
        3. 当Ego车辆进入探测范围且时间步为DECISION_INTERVAL的整数倍时，发送询问消息
        4. RSU使用detect_vehicles_in_range方法获取车辆信息并回复给Ego车辆
        """
        # 获取Ego车辆
        ego_vehicle = None
        for vehicle in vehicles.values():
            if vehicle.vtype == VehicleType.EGO:
                ego_vehicle = vehicle
                break
        # 如果没有Ego车辆或没有启用通信功能，直接返回
        if not ego_vehicle or not self.if_traffic_communication:
            return
        # 遍历每个RSU，检查其是否在Ego车辆的AOI范围内
        for rsu_id, rsu in facilities.items():
            # 检查RSU是否已在EGO车辆的AOI范围内
            if rsu.isInAoI(ego_vehicle.lane_id, ego_vehicle.current_state.s, roadgraph):
                # 如果Ego车辆进入RSU探测范围且当前时间步为决策间隔的整数倍
                if current_time_step % self.config["DECISION_INTERVAL"] == 0:
                    # 检查是否已经发送过询问消息给这个RSU
                    if rsu_id not in self.queried_rsus:
                        # 设置RSU的上下文参数
                        if rsu.communicator and hasattr(rsu.communicator, 'set_context'):
                            rsu.communicator.set_context(vehicles, roadgraph)
                        # Ego车辆发送询问消息给RSU
                        query_content = f"InformationRequest2RSU({ego_vehicle.id},{rsu_id});"
                        if ego_vehicle.communicator:
                            ego_vehicle.communicator.send(query_content, rsu_id, RSUCommunicator, performative=Performative.Query)
                        # 将RSU ID添加到已查询集合中
                        self.queried_rsus.add(rsu_id)
            else:
                # 如果Ego车辆不在RSU的AOI范围内，从已查询集合中移除该RSU
                if rsu_id in self.queried_rsus:
                    self.queried_rsus.discard(rsu_id)

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

    # 9.13 新增方法：提取facility信息
    def extract_facilities(
            self, facilities_info: Dict, roadgraph: RoadGraph
    ) -> Dict:
        """
        Extracts facilities (RSUs) from the provided information and returns them as a dictionary.

        Args:
            facilities_info (dict): Dictionary containing information about the facilities (RSUs).
            roadgraph (RoadGraph): Road graph of the simulation.

        Returns:
            Dict[str, control_RSU]: A dictionary containing the RSU instances.
        """
        facilities = {}
        
        # 如果没有设施信息，直接返回空字典
        if not facilities_info:
            return facilities
            
        # 提取AOI内的RSU信息
        if "rsuInAoI" in facilities_info:
            for rsu_info in facilities_info["rsuInAoI"]:
                if not rsu_info:  # 如果RSU信息为空，跳过
                    continue
                    
                rsu_id = rsu_info["id"]
                # 如果RSU之前已经存在且有通信功能
                if rsu_id in self.lastseen_facilities:
                    facilities[rsu_id] = create_rsu_lastseen(
                        rsu_info,
                        self.lastseen_facilities[rsu_id],
                        RSUType.IN_AOI
                    )
                    # 如果启用了通信功能，且RSU之前没有初始化通信器，初始化通信器
                    if self.if_traffic_communication and facilities[rsu_id].communicator is None:
                        facilities[rsu_id].init_communication(self.communication_manager)
                else:
                    # 创建新的RSU实例
                    rsu_temp = create_rsu(rsu_info, RSUType.IN_AOI)
                    # 如果启用了通信功能，初始化通信器
                    if self.if_traffic_communication:
                        rsu_temp.init_communication(self.communication_manager)
                    facilities[rsu_id] = rsu_temp
        
        return facilities

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
            # 添加对vehicle字典中关键字段的边界检查
            if not vehicle.get("xQ") or not vehicle.get("laneIDQ") or not vehicle.get("yQ"):# 如果车辆未出现在场景中，跳过此车，查看下一辆车
                continue
            # 添加对列表长度的检查
            if len(vehicle["xQ"]) == 0 or len(vehicle["laneIDQ"]) == 0 or len(vehicle["yQ"]) == 0:
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
                    vehicle, roadgraph, vtype_info, T, VehicleType.IN_AOI,self.if_traffic_communication,if_ego=False,communication_manager=self.communication_manager,ego_id=ego_car.id if ego_car else None)
                # 8.20 新增：将非Ego车辆添加到通信管理器中
                if self.if_traffic_communication:
                    vehicle_temp.init_communication(self.communication_manager,if_egoCar=False)
                vehicles[vehicle["id"]] = vehicle_temp

        # 提取AOI外的车辆信息
        for vehicle in vehicles_info["outOfAoI"]:
            # 添加对vehicle字典中关键字段的边界检查
            if not vehicle.get("xQ") or not vehicle.get("laneIDQ"):
                continue
            # 添加对列表长度的检查
            if len(vehicle["xQ"]) == 0 or len(vehicle["laneIDQ"]) == 0:
                continue
            vtype_info = self.sumo_model.allvTypes[vehicle["vTypeID"]]
            # 添加对laneIDQ列表的空值检查，避免索引越界
            if vehicle["laneIDQ"] and roadgraph.get_lane_by_id(vehicle["laneIDQ"][-1]) is not None:
                # 8.19 新增：创建车辆通信功能
                vehicle_temp = create_vehicle(
                    vehicle, roadgraph, vtype_info, T, VehicleType.OUT_OF_AOI,self.if_traffic_communication,if_ego=False,communication_manager=self.communication_manager,ego_id=ego_car.id if ego_car else None)
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
        # 添加对ego_info字典中关键字段的边界检查
        if not ego_info.get("xQ") or not ego_info.get("laneIDQ") or not ego_info.get("yQ"):
            return None
        # 添加对列表长度的检查
        if len(ego_info["xQ"]) == 0 or len(ego_info["laneIDQ"]) == 0 or len(ego_info["yQ"]) == 0:
            return None

        ego_id = ego_info["id"]
        if ego_id in self.lastseen_vehicles:
            # 自车已存在
            if len(self.lastseen_vehicles[ego_id].trajectory.states) > through_timestep:
                # 有足够的轨迹状态，使用create_vehicle_lastseen
                last_state = self.lastseen_vehicles[ego_id].trajectory.states[through_timestep]
                ego_car = create_vehicle_lastseen(
                    ego_info,
                    self.lastseen_vehicles[ego_id],
                    roadgraph,
                    T,
                    last_state,
                    VehicleType.EGO,
                    sim_mode
                )
            else:
                # 轨迹状态不足，创建新的自车对象但保留原有通信器
                vtype_info = self.sumo_model.allvTypes[ego_info["vTypeID"]]
                ego_car = create_vehicle(ego_info, roadgraph, vtype_info, T,
                                        VehicleType.EGO, self.if_traffic_communication, if_ego=True, communication_manager=self.communication_manager, ego_id=ego_id)
                
                # 保留原有通信器和消息历史
                original_vehicle = self.lastseen_vehicles[ego_id]
                if self.if_traffic_communication and hasattr(original_vehicle, 'communicator') and original_vehicle.communicator is not None:
                    # 复制原有通信器的消息历史到新的通信器
                    ego_car.init_communication(self.communication_manager, True)
                    ego_car.communicator.message_history = original_vehicle.communicator.message_history
        else: 
            # 初次出现自车，全新创建
            vtype_info = self.sumo_model.allvTypes[ego_info["vTypeID"]]
            ego_car = create_vehicle(ego_info, roadgraph, vtype_info, T,
                                        VehicleType.EGO, self.if_traffic_communication, if_ego=True, communication_manager=self.communication_manager, ego_id=ego_id)
            # 8.19 新增：将自车添加到通信管理器中
            if self.if_traffic_communication:
                ego_car.init_communication(self.communication_manager, True)
                ego_car.communicator.send(f"SelfVehicle({ego_car.id});")
        return ego_car
