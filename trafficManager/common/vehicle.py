"""
This module contains the Vehicle class and related functions for managing vehicles in a traffic simulation.
翻译：
这个模块包含Vehicle类和相关函数，用于智能管理交通模拟中的车辆
注意和simModel.carFactory.Vehicle的区别
control_Vehicle类：
    - 交通管理专用 ：为交通管理系统设计的车辆类
    - 行为控制 ：包含详细的行为状态管理（如变道、停车、加速等）
    - 决策支持 ：提供车辆行为决策的接口
    - Frenet坐标系 ：支持Frenet坐标系下的车辆状态表示
    - 通信功能 ：集成车辆间通信能力（V2V）

Classes:
    Behaviour (IntEnum): Enum class for vehicle behavior.
    Vehicle: Represents a vehicle in the simulation.

Functions:
    create_vehicle(vehicle_info, roadgraph: RoadGraph, T, vtype_info, vtype) -> Vehicle:
        Creates a new Vehicle instance based on the provided information.
    create_vehicle_lastseen(vehicle_info, lastseen_vehicle, roadgraph: RoadGraph, T, through_timestep, vtype) -> Vehicle:
        Creates a Vehicle instance based on the last seen vehicle information.
    extract_vehicles(vehicles_info: dict, roadgraph: RoadGraph, lastseen_vehicles: dict, T: float, through_timestep: int, sumo_model) -> Tuple[Vehicle, Dict[int, Vehicle], Dict[int, Vehicle]]:
        Extracts vehicles from the provided information and returns them as separate dictionaries.
"""
from copy import copy, deepcopy
from enum import IntEnum, Enum
from typing import Any, Dict, Set, Tuple
from collections import deque

from trafficManager.common.coord_conversion import cartesian_to_frenet2D
from utils.roadgraph import AbstractLane, JunctionLane, NormalLane, RoadGraph
from utils.trajectory import State
from TSRL_interaction.vehicle_communication import CommunicationManager, Performative
from TSRL_interaction.communicator_category import VehicleCommunicator

import logger

logging = logger.get_logger(__name__)


class Behaviour(IntEnum):
    KL = 0 # 保持车道
    AC = 1 # 加速
    DC = 2 # 减速
    LCL = 3 # 变道左
    LCR = 4 # 变道右
    STOP = 5 # 停车
    IN_JUNCTION = 6 # 进入 junction
    OVERTAKE = 7 # 超车
    OTHER = 100
    SEND = 101 # 发送消息
    SPECIAL_HANDLING = 102 # 特殊处理


class VehicleType(str, Enum):
    EGO = "Ego_Car"
    IN_AOI = "Car_In_AoI"
    OUT_OF_AOI = "Car_out_of_AoI"


class control_Vehicle():
    def __init__(self,
                 vehicle_id: int,
                 init_state: State = State(),
                 lane_id: int = -1,
                 target_speed: float = 0.0,
                 behaviour: Behaviour = Behaviour.KL,
                 vtype: VehicleType = VehicleType.OUT_OF_AOI,
                 length: float = 5.0,
                 width: float = 2.0,
                 max_accel: float = 3.0,
                 max_decel: float = -3.0,
                 max_speed: float = 50.0,
                 available_lanes: Dict[int, Any] = {},
                 stop_lane: list[str] = None,
                 stop_pos: deque = deque(maxlen=100),
                 stop_until: deque = deque(maxlen=100),
                 if_traffic_communication: bool = False,
                 if_ego: bool = False,
                 communication_manager: CommunicationManager = None,
                 ego_id: str = None) -> None:



        """
        Initialize a Vehicle instance.

        Args:
            vehicle_id (int): Vehicle ID.
            init_state (State, optional): Initial state of the vehicle. Defaults to State().
            lane_id (int, optional): Lane ID of the vehicle. Defaults to -1.
            target_speed (float, optional): Target speed of the vehicle. Defaults to 0.0.
            behaviour (Behaviour, optional): Behaviour of the vehicle. Defaults to Behaviour.KL.
            vtype (str, optional): Vehicle type. Defaults to "outOfAoI".
            length (float, optional): Length of the vehicle. Defaults to 5.0.
            width (float, optional): Width of the vehicle. Defaults to 2.0.
            max_accel (float, optional): Maximum acceleration of the vehicle. Defaults to 3.0.
            max_decel (float, optional): Maximum deceleration of the vehicle. Defaults to -3.0.
            max_speed (float, optional): Maximum speed of the vehicle. Defaults to 50.0.
            available_lanes (Dict[int, Any], optional): Available lanes for the vehicle. Defaults to {}.
            if_traffic_communication (bool, optional): Whether the vehicle has traffic communication enabled. Defaults to False.
        """
        self.id = vehicle_id
        self._current_state = init_state
        self.lane_id = lane_id
        self.behaviour = behaviour
        self.target_speed = target_speed
        self.vtype = vtype
        self.length = length
        self.width = width
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_speed = max_speed
        self.available_lanes = available_lanes
        self.stop_lane = stop_lane
        self.stop_pos = stop_pos
        self.stop_until = stop_until
        # 8.12 添加前车状态
        self.front_vehicle_status = Behaviour.KL
        # 8.19 新增是否开启通信功能的控制参数
        self.if_traffic_communication = if_traffic_communication
        if if_traffic_communication:
            self.communication_manager=communication_manager
            self.communicator = VehicleCommunicator(self.id, self, self.communication_manager, if_ego)
        # 新增：记录是否已经发送过HasNextJunction消息
        self.has_sent_next_junction_msg = False
        # 新增：记录前一个行为状态，用于检测行为变化
        self.previous_behaviour = None
        # 新增：存储主车ID，用于判断是否为Ego车辆
        self.ego_id = ego_id
    @property
    def current_state(self) -> State:
        """
        Get the current state of the vehicle.

        Returns:
            State: The current state of the vehicle.
        """
        return self._current_state

    @current_state.setter
    def current_state(self, state: State) -> None:
        """
        Set the current state of the vehicle.

        Args:
            state (State): The new state of the vehicle.
        """
        self._current_state = state
    
    # 通信层
    # 25.8.16 新增方法，初始化车辆通信器
    def init_communication(self, communication_manager: CommunicationManager, if_egoCar: bool = False):
        """初始化车辆通信器"""
        self.if_egoCar=if_egoCar
        self.communication_manager = communication_manager
        # 根据车辆ID类型初始化对应通信器
        self.communicator = VehicleCommunicator(self.id, self, self.communication_manager, self.if_egoCar) # 初始化RV通信器
        self.communicator.vehicle = self
    
    # 获取车辆在车道上的状态
    def get_state_in_lane(self, lane) -> State:
        course_spline = lane.course_spline # 获得车道曲线

        rs = course_spline.find_nearest_rs(self.current_state.x,
                                           self.current_state.y)

        rx, ry = course_spline.calc_position(rs)
        ryaw = course_spline.calc_yaw(rs)
        rkappa = course_spline.calc_curvature(rs)

        s, s_d, d, d_d = cartesian_to_frenet2D(rs, rx, ry, ryaw, rkappa,
                                               self.current_state)
        return State(s=s, s_d=s_d, d=d, d_d=d_d,
                     x=self.current_state.x,
                     y=self.current_state.y,
                     yaw=self.current_state.yaw,
                     vel=self.current_state.vel,
                     acc=self.current_state.acc)
    
    # 车道变换
    def change_to_lane(self, lane: AbstractLane) -> None:
        """
        Change the vehicle to the next lane.
        中文翻译：
        将车辆变换到下一个车道（lane）。
        """
        self.lane_id = lane.id # 更新车辆车道ID
        self.current_state = self.get_state_in_lane(lane) # 更新车辆状态
        self.behaviour = Behaviour.KL # 更新车辆行为


    # 获取车辆字符串表示
    def __repr__(self) -> str:
        """
        Get the string representation of the vehicle.

        Returns:
            str: The string representation of the vehicle.
        """
        return f"Vehicle(id={self.id}, lane_id={self.lane_id}, "\
               f"current_state=(s={self.current_state.s}, d={self.current_state.d}, "\
               f"s_d={self.current_state.s_d}, s_dd={self.current_state.s_dd}))"

    def update_behavior_with_manual_input(self, manual_input: str,
                                          current_lane: AbstractLane):
        """use keyboard to send lane change command to ego car.
        中文翻译：
        使用键盘发送车道变换命令给Ego车。
        Args:
            manual_input (str): the left turn or right turn command
            current_lane (AbstractLane): the lane that ego car lies on
        """
        if self.vtype != VehicleType.EGO:
            # currently, manual input only works on ego car
            return
        if self.behaviour != Behaviour.KL:
            # lane change behavior works only when ego is in lane keeping state.
            return
        if manual_input == 'Left' and current_lane.left_lane(
        ) in self.available_lanes:
            self.behaviour = Behaviour.LCL
            logging.info(f"Key command Vehicle {self.id} to change Left lane")
        elif manual_input == 'Right' and current_lane.right_lane(
        ) in self.available_lanes:
            self.behaviour = Behaviour.LCR
            logging.warning(
                f"Key command Vehicle {self.id} to change Right lane")
        elif manual_input:
            # 导入Scanner类和相关依赖
            from TSRL_representation.Scanner import Scanner
            from TSRL_representation.Tokentype import Token, TokenType
            from decision_maker.TSRL_decision_maker import action_name_to_behaviour_mapper
            # 创建Scanner实例
            scanner = Scanner(manual_input)
            # 进行词法分析
            tokens = scanner.scan_tokens()
            # 打印词法分析结果
            for token in tokens:
                if token.type == TokenType.IDENTIFIER:
                    action = token.lexeme
                elif token.type == TokenType.NUMBER:
                    action_id=token.lexeme
            if self.id != action_id:
                return
            self.behaviour = action_name_to_behaviour_mapper.get_behaviour(action)
            
    def handle_sender_location(self, sender_id: str, vehicles: Dict[str, 'control_Vehicle'], roadgraph) -> str:
        """
        处理发送主体与自主体位置消息，判断相对位置关系
        
        Args:
            sender_id: 发送位置消息的车辆主体ID
            vehicles: 所有车辆的字典
            roadgraph: 道路图信息
            
        Returns:
            str: 相对位置关系描述字符串，或None表示无法判断
        """
        # 获取当前车辆信息（自身）
        current_vehicle = vehicles.get(self.id)
        if not current_vehicle:
            print(f"Warning: Current vehicle {self.id} not found in vehicles dict")
            return None
        # 检查发送者是否在车辆字典中
        if sender_id not in vehicles:
            print(f"Warning: Sender vehicle {sender_id} not found in vehicles dict")
            return None
        sender_vehicle = vehicles[sender_id]
        # 获取车道和位置信息
        sender_lane_id = sender_vehicle.lane_id
        sender_pos = sender_vehicle.current_state.s
        current_lane_id = current_vehicle.lane_id
        current_pos = current_vehicle.current_state.s
        # 判断相对位置关系
        if current_lane_id == sender_lane_id:
            if current_pos >= sender_pos:
                return f"VehicleInLane({self.id},{sender_id},Front);" # 自车在sender同车道前方
            else:
                return f"VehicleInLane({self.id},{sender_id},Rear);" # 自车在sender同车道后方
        else:
            # 获取发送者车道对象
            sender_lane = roadgraph.get_lane_by_id(sender_lane_id)
            # 判断当前车辆是否在发送者的左车道
            if (sender_lane and hasattr(sender_lane, 'left_lane') and 
                sender_lane.left_lane() == current_lane_id):
                if current_pos >= sender_pos:
                    return f"VehicleLeftLane({self.id},{sender_id},Front)"
                else:
                    return f"VehicleLeftLane({self.id},{sender_id},Rear)"
            # 判断当前车辆是否在发送者的右车道
            elif (sender_lane and hasattr(sender_lane, 'right_lane') and 
                sender_lane.right_lane() == current_lane_id):
                if current_pos >= sender_pos:
                    return f"VehicleRightLane({self.id},{sender_id},Front)"
                else:
                    return f"VehicleRightLane({self.id},{sender_id},Rear)"
            # 如果无法判断相对位置关系，返回None
            return None

    def update_behaviour(self, roadgraph: RoadGraph, manual_input: str = None, vehicles: dict = None) -> None:
        """Update the behaviour of a vehicle.
        中文翻译：
        更新车辆的行为（默认行为更新，非决策生成行为更新）
        Args:
            roadgraph (RoadGraph): The roadgraph containing the lanes the vehicle is traveling on.
        【基本架构】：
        一、 信息获取
            1.1 保存self车辆上一个行为状态，用于检测行为变化
            1.2 添加车辆位置信息日志
            1.3 车辆前方交叉口信息
            1.4 检测行为状态变化
        二、 外部控制：使用键盘输入左右方向or ad键实现变道
        三、 内部控制 
            3.1 行为更新
                3.1.1 主动停车
                3.1.2 车辆变道行为
                3.1.3 车辆进入 junction 行为更新
        """
        # 一、信息获取
        # 1.1 保存self车辆上一个行为状态，用于检测行为变化
        self.previous_behaviour = self.behaviour
        current_lane = roadgraph.get_lane_by_id(self.lane_id)
        logging.debug(
            f"Vehicle {self.id} is in lane {self.lane_id}, "
            f"In available_lanes? {current_lane.id in self.available_lanes}")
        # 1.2 添加车辆位置信息日志
        logging.info(f"Vehicle {self.id} position: x={self.current_state.x}, y={self.current_state.y}, lane_id={self.lane_id}")
        # 1.3 车辆前方交叉口信息
        # 车辆刚进入道路入口时，发送下一个交叉口信息
        if isinstance(current_lane, NormalLane) and not self.has_sent_next_junction_msg: # 如果当前车辆在普通车道上且还未发送过消息
            try:
                # 获取下一个交叉口信息（不限制位置）
                next_lane = roadgraph.get_available_next_lane(current_lane.id, self.available_lanes)
                if next_lane and hasattr(next_lane, 'affJunc') and next_lane.affJunc:
                    junction_id = next_lane.affJunc
                    # 发送HasNextJunction消息（仅主车发送）
                    if self.if_traffic_communication and self.communicator and self.ego_id and self.id == self.ego_id:
                        self.communicator.send(f"HasNextJunction({self.id},{junction_id});", performative=Performative.Inform)
                        logging.info(f"Vehicle {self.id} sent HasNextJunction message EARLY when entering road, next junction: {junction_id}, current position: s={self.current_state.s}")
                        self.has_sent_next_junction_msg = True  # 标记已发送
            except Exception as e:
                logging.warning(f"Vehicle {self.id} failed to get next junction info early: {e}")
        # 1.4 检测行为状态变化
        # 仅当从其他状态变为STOP时才发送LetStop消息
        if (self.previous_behaviour != self.behaviour and 
            self.behaviour == Behaviour.STOP and 
            self.if_traffic_communication and 
            self.communicator):
            self.communicator.send(f"LetStop({self.id});", performative=Performative.Inform)
            logging.info(f"Vehicle {self.id} sent LetStop message when behaviour changed from {self.previous_behaviour} to STOP")
        # 新增：
        self.previous_behaviour = self.behaviour
        # 二、 外部控制
        # 使用输入指令控制ego车辆（键盘输入左右方向or ad键实现变道）
        self.update_behavior_with_manual_input(manual_input, current_lane)
        # 三、 内部控制
        # 3.1 行为更新
        # 3.1.1 主动停车
        # 如果车辆在stop_lane上且未到达停车位置
        if self.stop_lane and self.lane_id in self.stop_lane and self.current_state.s < self.stop_pos:
            self.behaviour = Behaviour.STOP
        # 如果车辆已经到达停车位置但未到达stop_until时间，保持STOP行为
        elif (self.stop_lane and self.lane_id in self.stop_lane and 
              self.current_state.s >= self.stop_pos and 
              self.stop_until is not None and 
              self.current_state.t < self.stop_until):
            self.behaviour = Behaviour.STOP
            # 确保在停车期间保持stop_flag为True
            self.current_state.stop_flag = True
        # 如果stop_until时间已经到达，清除stop_flag并恢复正常行为
        elif (self.stop_lane and self.lane_id in self.stop_lane and 
              self.stop_until is not None and 
              self.current_state.t >= self.stop_until):
            self.current_state.stop_flag = False
            self.behaviour = Behaviour.KL  # 恢复正常行驶行为
            logging.info(f"Vehicle {self.id} stop_until time {self.stop_until} reached, clearing stop_flag")
        # 3.1.2 车辆变道行为
        # Lane change behavior··
        if isinstance(current_lane, NormalLane): # 如果当前车辆在普通车道上
            # 如果车辆行为是变道左
            if self.behaviour == Behaviour.LCL:
                left_lane_id = current_lane.left_lane()
                left_lane = roadgraph.get_lane_by_id(left_lane_id)
                state = self.get_state_in_lane(left_lane)
                if state.d > -left_lane.width / 2:
                    self.change_to_lane(left_lane)
            # 如果车辆行为是变道右
            elif self.behaviour ==Behaviour.LCR:
                right_lane_id = current_lane.right_lane()
                right_lane = roadgraph.get_lane_by_id(right_lane_id)
                state = self.get_state_in_lane(right_lane)
                if state.d < right_lane.width / 2:
                    self.change_to_lane(right_lane)
            # 如果车辆行为是进入 junction
            elif self.behaviour == Behaviour.IN_JUNCTION:
                self.behaviour = Behaviour.KL          
        # 需要变道的条件：
        # 1. 当前车道不在可用车道中
        # 2. 当前车道可用但前方有停止车辆
        need_lane_change = False
        lane_change_reason = ""
        # 条件1：当前车道不在可用车道中
        if current_lane.id not in self.available_lanes:
            need_lane_change = True
            lane_change_reason = f"since {self.lane_id} not in available_lanes {self.available_lanes}"
        # 条件2：当前车道可用但前方有停止车辆
        # elif vehicles is not None:
        #     front_vehicle_status = get_pre_vehicle_status(self, vehicles)
        #     if front_vehicle_status == Behaviour.STOP:
        #         need_lane_change = True
        #         lane_change_reason = "due to stopped vehicle ahead"
        # 如果需要变道   
        if need_lane_change:  
            logging.debug(
                f"Vehicle {self.id} need lane-change, {lane_change_reason}"
                )
            if self.behaviour == Behaviour.KL:
                # find left available lanes
                lane = current_lane
                # 只对NormalLane类型的车道调用left_lane方法
                while isinstance(lane, NormalLane) and lane.left_lane() is not None:
                    lane_id = lane.left_lane()
                    if lane_id in self.available_lanes:
                        # 当前条件可以进行左变道
                        self.communicator.send(f"LeftChangeLaneSafe({self.id});",performative=Performative.Inform)
                        self.behaviour = Behaviour.LCL
                        logging.info(
                            f"Vehicle {self.id} choose to change Left lane")
                        break
                    lane = roadgraph.get_lane_by_id(lane_id)
                # 只对NormalLane类型的车道调用right_lane方法
                while isinstance(lane, NormalLane) and lane.right_lane() is not None:
                    lane_id = lane.right_lane()
                    if lane_id in self.available_lanes:
                        # 当前条件可以进行右变道
                        self.communicator.send(f"RightChangeLaneSafe({self.id});",performative=Performative.Inform)
                        self.behaviour = Behaviour.LCR
                        logging.info(
                            f"Vehicle {self.id} choose to change Right lane"
                        )
                        break
                    lane = roadgraph.get_lane_by_id(lane_id)
                if self.behaviour == Behaviour.KL:
                    # can not reach to available lanes
                    logging.error(
                        f"Vehicle {self.id} cannot change to available lanes, "
                        f"current lane {self.lane_id}, available lanes {self.available_lanes}"
                    )
        
        # 3.1.4 车辆进入 junction 行为更新
        # in junction behaviour
        if self.current_state.s > current_lane.course_spline.s[-1] - 0.2:
            if isinstance(current_lane, NormalLane):
                next_lane = roadgraph.get_available_next_lane(
                    current_lane.id, self.available_lanes)
                self.lane_id = next_lane.id
                self.current_state = self.get_state_in_lane(next_lane)
                current_lane = next_lane
                # 发送HasNextJunction消息：车辆进入交叉口前（仅主车发送）
                if self.if_traffic_communication and self.communicator and self.ego_id and self.id == self.ego_id:
                    # 获取下一个交叉口ID
                    if hasattr(next_lane, 'affJunc') and next_lane.affJunc:
                        junction_id = next_lane.affJunc
                        self.communicator.send(f"HasNextJunction({self.id},{junction_id});", performative=Performative.Inform)
                        logging.info(f"Vehicle {self.id} sent HasNextJunction message before entering junction {junction_id}")          
            elif isinstance(current_lane, JunctionLane):
                next_lane_id = current_lane.next_lane_id
                next_lane = roadgraph.get_lane_by_id(next_lane_id)
                self.lane_id = next_lane.id
                self.current_state = self.get_state_in_lane(next_lane)
                current_lane = next_lane
            else:
                logging.error(
                    f"Vehicle {self.id} Lane {self.lane_id}  is unknown lane type {type(current_lane)}"
                )
            if isinstance(current_lane, JunctionLane):  # in junction
                self.behaviour = Behaviour.IN_JUNCTION
                logging.info(f"Vehicle {self.id} is in {self.behaviour}")  
            else:  # out junction
                self.behaviour = Behaviour.KL

    def set_stop_info(self, stops):
        """设置车辆停车信息"""
        self.stop_info = stops

    def selfcheck(self, decision_output, roadgraph: RoadGraph):
        """自车进行条件检查行为"""
        # 1. 检查变道是否合法
        if decision_output.startswith("CheckChangeLane"):
            lane = roadgraph.get_lane_by_id(self.lane_id)
            # 只对NormalLane类型的车道调用left_lane方法
            while isinstance(lane, NormalLane) and lane.left_lane() is not None:
                lane_id = lane.left_lane()
                if lane_id in self.available_lanes:
                    # 当前条件可以进行左变道
                    self.communicator.send(f"LeftChangeLaneSafe({self.id});",performative=Performative.Inform)
                    self.behaviour = Behaviour.LCL
                    logging.info(
                        f"Vehicle {self.id} choose to change Left lane")
                    break
                lane = roadgraph.get_lane_by_id(lane_id)
            # 只对NormalLane类型的车道调用right_lane方法
            while isinstance(lane, NormalLane) and lane.right_lane() is not None:
                lane_id = lane.right_lane()
                if lane_id in self.available_lanes:
                    # 当前条件可以进行右变道
                    self.communicator.send(f"RightChangeLaneSafe({self.id});",performative=Performative.Inform)
                    self.behaviour = Behaviour.LCR
                    logging.info(
                        f"Vehicle {self.id} choose to change Right lane")
                    break
                lane = roadgraph.get_lane_by_id(lane_id)
           

def create_vehicle(vehicle_info: Dict, roadgraph: RoadGraph, vtype_info: Any,
                   T,
                   vtype: VehicleType,
                   if_traffic_communication: bool = False,
                   if_ego: bool = False,
                   communication_manager: CommunicationManager = None,
                   ego_id: str = None) -> control_Vehicle:


    """
    Creates a new Vehicle instance based on the provided information.

    Args:
        vehicle_info (Dict): Information about the vehicle.
        roadgraph (RoadGraph): Road graph of the simulation.
        T (float): Current time step.
        vtype_info (Any): Vehicle type information.
        vtype (str): Vehicle type.

    Returns:
        Vehicle: A new Vehicle instance.
    """
    # 10.20 添加对各种Q列表的空值检查，避免索引越界
    if not vehicle_info.get("laneIDQ") or not vehicle_info.get("lanePosQ") or not vehicle_info.get("xQ") or not vehicle_info.get("yQ") or not vehicle_info.get("yawQ") or not vehicle_info.get("speedQ"):
        logging.error(f"Vehicle info missing required data: {vehicle_info}")
        return None
    
    # 添加对列表长度的检查，确保列表不为空
    if len(vehicle_info["laneIDQ"]) == 0 or len(vehicle_info["lanePosQ"]) == 0 or len(vehicle_info["xQ"]) == 0 or len(vehicle_info["yQ"]) == 0 or len(vehicle_info["yawQ"]) == 0 or len(vehicle_info["speedQ"]) == 0:
        logging.error(f"Vehicle info lists are empty: {vehicle_info}")
        return None
    
    available_lanes = vehicle_info["availableLanes"]
    lane_id = vehicle_info["laneIDQ"][-1]
    lane_pos = vehicle_info["lanePosQ"][-1]
    pos_x = vehicle_info["xQ"][-1]
    pos_y = vehicle_info["yQ"][-1]
    yaw = vehicle_info["yawQ"][-1]
    speed = vehicle_info["speedQ"][-1]
    # 8.3 添加停车信息传递
    stop_lane = vehicle_info["stop_info"][0]['lane'] if vehicle_info["stop_info"] else None
    stop_pos = vehicle_info["stop_info"][0]['end_pos'] if vehicle_info["stop_info"] else None
    stop_until= vehicle_info["stop_info"][0]['until'] if vehicle_info["stop_info"] else None
    # acc = vehicle_info["accelQ"].pop()
    acc = 0

    lane_id, pos_s, pos_d = find_lane_position(lane_id, roadgraph,
                                               available_lanes, lane_pos, pos_x,
                                               pos_y)

    init_state = State(x=pos_x,
                       y=pos_y,
                       yaw=yaw,
                       s=pos_s,
                       d=pos_d,
                       s_d=speed,
                       s_dd=acc,
                       t=T)
    v_new=control_Vehicle(
        vehicle_id=vehicle_info["id"],
        init_state=init_state,
        lane_id=lane_id,
        target_speed=30.0 / 3.6,
        behaviour=Behaviour.KL,
        vtype=vtype,
        length=vtype_info.length,
        width=vtype_info.width,
        max_accel=vtype_info.maxAccel,
        max_decel=-vtype_info.maxDecel,
        max_speed=vtype_info.maxSpeed,
        available_lanes=available_lanes,
        stop_lane=stop_lane,
        stop_pos=stop_pos,
        stop_until=stop_until,
        if_traffic_communication=if_traffic_communication,
        if_ego=if_ego,
        communication_manager=communication_manager,
        ego_id=ego_id
    )

    # 新增：车辆创建时立即发送HasNextJunction消息（仅主车发送）
    if if_traffic_communication and isinstance(roadgraph.get_lane_by_id(lane_id), NormalLane) and ego_id and vehicle_info["id"] == ego_id:
        try:
            current_lane = roadgraph.get_lane_by_id(lane_id)
            next_lane = roadgraph.get_available_next_lane(lane_id, available_lanes)
            if next_lane and hasattr(next_lane, 'affJunc') and next_lane.affJunc:
                junction_id = next_lane.affJunc
                v_new.communicator.send(f"HasNextJunction({vehicle_info['id']},{junction_id});", performative=Performative.Inform)
                logging.info(f"Vehicle {vehicle_info['id']} sent HasNextJunction message at CREATION, next junction: {junction_id}, initial lane: {lane_id}")
                v_new.has_sent_next_junction_msg = True
        except Exception as e:
            logging.warning(f"Vehicle {vehicle_info['id']} failed to send HasNextJunction message at creation: {e}")

    return v_new


def find_lane_position(lane_id: str, roadgraph: RoadGraph,
                       available_lanes: Set[str],
                       lane_pos: float, pos_x: float,
                       pos_y: float) -> Tuple[str, float, float]:
    """Given the map information and cartesian coordinate, 
       find corresponding frenet coordinates 

    Args:
        lane_id (str): initial guess of lane that cartesian coordinate lies on
        roadgraph (RoadGraph): map information
        available_lanes (Set[str]): possible lanes
        lane_pos (float): initial guess of s coordinate
        pos_x (float): cartesian x coordinate
        pos_y (float): cartesian y coordinate

    Returns:
        Tuple[str, float, float]: A tuple (lane_id, s coordinate, d coordinate)
    """
    lane = roadgraph.get_lane_by_id(lane_id)

    # lan_pos &lane_id is wrong in internal-junction lane
    # https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html#internal_junctions
    if lane is None or (isinstance(lane, JunctionLane) and
                        lane.id not in available_lanes):
        for available_lane_id in available_lanes:
            lane = roadgraph.get_lane_by_id(available_lane_id)
            if not isinstance(lane, JunctionLane):
                continue

            pos_s, pos_d = lane.course_spline.cartesian_to_frenet1D(
                pos_x, pos_y)

            if abs(pos_d) < 2.0:
                return available_lane_id, pos_s, pos_d
    else:
        pos_s, pos_d = lane.course_spline.cartesian_to_frenet1D(pos_x, pos_y)
        return lane_id, pos_s, pos_d
    return None, None, None


def create_vehicle_lastseen(vehicle_info: Dict, lastseen_vehicle: control_Vehicle,
                            roadgraph: RoadGraph, T: float, last_state: State,
                            vtype: VehicleType, sim_mode: str) -> control_Vehicle:

    """
    Creates a Vehicle instance based on the last seen vehicle information.

    Args:
        vehicle_info (Dict): Information about the vehicle.
        lastseen_vehicle (Vehicle): The last seen vehicle instance.
        roadgraph (RoadGraph): Road graph of the simulation.
        T (float): Current time step.
        through_timestep (int): The number of timesteps the vehicle has been through.
        vtype (str): Vehicle type.

    Returns:
        Vehicle: A new Vehicle instance with updated information.
    """
    # 添加对各种Q列表的空值检查，避免索引越界
    if not vehicle_info.get("laneIDQ") or not vehicle_info.get("xQ") or not vehicle_info.get("yQ"):
        logging.error(f"Vehicle info missing required data for lastseen vehicle: {vehicle_info}")
        return None
        
    # 添加对列表长度的检查，确保列表不为空
    if len(vehicle_info["laneIDQ"]) == 0 or len(vehicle_info["xQ"]) == 0 or len(vehicle_info["yQ"]) == 0:
        logging.error(f"Vehicle info lists are empty for lastseen vehicle: {vehicle_info}")
        return None
        
    vehicle = copy(lastseen_vehicle) # 复制lastseen_vehicle
    vehicle.current_state = last_state
    vehicle.current_state.t = T
    vehicle.current_state.x = vehicle_info["xQ"][-1]
    vehicle.current_state.y = vehicle_info["yQ"][-1]
    vehicle.vtype = vtype
    vehicle.available_lanes = vehicle_info["availableLanes"]
    # 8.3 添加停车信息传递
    if vehicle_info["stop_info"]:
        # 添加对stop_info列表的边界检查
        if len(vehicle_info["stop_info"]) > 0:
            vehicle.stop_lane = vehicle_info["stop_info"][0]['lane']
            vehicle.stop_pos = vehicle_info["stop_info"][0]['end_pos']
            vehicle.stop_until= vehicle_info["stop_info"][0]['until']

    lane_id = vehicle_info["laneIDQ"][-1]
    # in some junctions, sumo will not find any lane_id for the vehicle
    while lane_id == "":
        logging.debug("lane_id is empty")
        if not vehicle_info["laneIDQ"]:  # 检查列表是否为空
            break
        lane_id = vehicle_info["laneIDQ"].pop()
        if lane_id != "":
            lane_id = roadgraph.get_available_next_lane(
                lane_id, vehicle_info["availableLanes"]).id

    # don't care about lane_id while in junction
    if sim_mode == 'InterReplay' or all(isinstance(roadgraph.get_lane_by_id(lane), NormalLane)
                                        for lane in (vehicle.lane_id, lane_id)):
        # fixme: really need?
        if lane_id != vehicle.lane_id and \
                vehicle.behaviour in (Behaviour.LCL, Behaviour.LCR):
            logging.warning(f"Vehicle {vehicle.id} have changed"
                            f"lane from {vehicle.lane_id} to {lane_id}")
            vehicle.behaviour = Behaviour.KL

        if not vehicle_info.get("lanePosQ"):  # 检查lanePosQ是否存在且非空
            logging.error(f"Vehicle info missing lanePosQ for lastseen vehicle: {vehicle_info}")
            return vehicle
            
        lanepos = vehicle_info["lanePosQ"][-1]
        vehicle.lane_id = lane_id
        lane = roadgraph.get_lane_by_id(lane_id)
        x = vehicle_info["xQ"][-1]
        y = vehicle_info["yQ"][-1]
        try:
            s, d = lane.course_spline.cartesian_to_frenet1D(x, y)
        except TypeError:
            logging.error("Vehicle line 185:", vehicle_info["lanePosQ"],
                          lanepos, lane.course_spline.s[-1])
            exit(1)
        vehicle.current_state.x = x
        vehicle.current_state.y = y
        vehicle.current_state.s = s
        vehicle.current_state.d = d

    return vehicle


def get_lane_id(vehicle_info, roadgraph):
    if not vehicle_info.get("laneIDQ") or len(vehicle_info["laneIDQ"]) == 0:
        logging.error(f"Vehicle info missing laneIDQ data: {vehicle_info}")
        return None
        
    lane_id = vehicle_info["laneIDQ"].pop()
    # 添加对laneIDQ列表的额外检查，确保列表不为空
    if len(vehicle_info["laneIDQ"]) == 0 and lane_id == "":
        logging.error(f"Vehicle info laneIDQ list is empty and lane_id is empty: {vehicle_info}")
        return None
    # in some junctions, sumo will not find any lane_id for the vehicle
    while lane_id == "":
        logging.debug("lane_id is empty")
        if not vehicle_info["laneIDQ"]:  # 检查列表是否为空
            break
        lane_id = vehicle_info["laneIDQ"].pop()
        if lane_id != "":
            lane_id = roadgraph.get_available_next_lane(
                lane_id, vehicle_info["availableLanes"]).id

    return lane_id

# 8.12：判断车辆前车状态
def get_pre_vehicle_status(vehicle: control_Vehicle, vehicles: Dict[int,control_Vehicle]) -> Behaviour:
    """
    Get the status of the pre vehicle.
    中文翻译：
    获取前车的状态。
    """
    # 检测前方车辆状态
    front_vehicle_status : Behaviour = Behaviour.KL
    # step 1. 找到前车
    # 遍历所有其他车辆，检查同车道前方的车辆
    for other_id, other_vehicle in vehicles.items():
        if other_id == vehicle.id:
            continue
        # 检查是否在同一条车道
        if other_vehicle.lane_id == vehicle.lane_id:
            # 计算前方距离
            distance = other_vehicle.current_state.s - vehicle.current_state.s
            if distance > 0 and distance < 50:  # 前方50米内
                # 检查前方车辆是否为停止状态（速度接近0）
                if abs(other_vehicle.current_state.s_d) <= 0.1:  # 速度小于0.001m/s
                    front_vehicle_status = Behaviour.STOP

                    logging.info(
                        f"Vehicle {vehicle.id} detected stopped vehicle {other_id} "
                        f"ahead at distance {distance:.2f}m, speed: {other_vehicle.current_state.s_d:.2f}m/s"
                    )
                    # 待补充：检测其他状态
    return front_vehicle_status
