"""
This module contains the RSU class and related functions for managing RSUs in a traffic simulation.
翻译：
这个模块包含RSU类和相关函数，用于管理交通模拟中的路侧单元。
Classes:
    RSU: Represents an RSU in the simulation.
    RSU_detector: Represents an RSU detector in the simulation.
"""

from enum import Enum
from typing import Any, Dict, FrozenSet, Set, List
from utils.trajectory import State
import logger
# 添加RSUCommunicator的导入
import sys
import os
from TSRL_interaction.communicator_category import RSUCommunicator
from TSRL_interaction.vehicle_communication import CommunicationManager
from trafficManager.common.vehicle import control_Vehicle
from TSRL_interaction.vehicle_communication import Message, Performative


logging = logger.get_logger(__name__)


class RSUType(str, Enum):
    IN_AOI = "RSU_In_AoI"
    OUT_OF_AOI = "RSU_out_of_AoI"


class RSU_detector:
    """
    功能：创建ATPSIP仿真平台上的【真实】RSU检测器，位于具体的车道上
    参数：
        id：检测器的唯一标识符
        lane：检测器所在的车道ID
        pos：检测器在车道上的位置（单位：米）
        detectlenth：检测器的检测长度（单位：米）
        detectfreq：检测器的检测频率（单位：赫兹）
        output：检测器的输出文件路径
    """
    def __init__(self, id: str, lane: str = '', pos: float = 0.0, detectlenth: float = 0.0, detectfreq: float = 0.0, output: str = '') -> None:
        self.id = id
        self.lane: str = lane
        self.pos: float = pos
        self.detectlenth: float = detectlenth
        self.detectfreq: float = detectfreq
        self.output: str = output


class control_RSU:
    def __init__(self,
                 rsu_id: str,
                 init_state: State = State(),
                 rsu_type: RSUType = RSUType.OUT_OF_AOI,
                 length: float = 5.0,
                 width: float = 1.8,
                 detectors: List[RSU_detector] = [],
                 deArea: float = 50.0) -> None:
        """
        Initialize an RSU instance.

        Args:
            rsu_id (str): RSU ID.
            init_state (State, optional): Initial state of the RSU. Defaults to State().
            rsu_type (RSUType, optional): RSU type. Defaults to RSUType.OUT_OF_AOI.
            length (float, optional): Length of the RSU. Defaults to 5.0.
            width (float, optional): Width of the RSU. Defaults to 1.8.
            detectors (List[RSU_detector], optional): List of detectors for the RSU. Defaults to [].
            deArea (float, optional): Detection area radius. Defaults to 50.0.
        """
        self.id = rsu_id
        self._current_state = init_state
        self.rsu_type = rsu_type
        self.length = length
        self.width = width
        self.detectors = detectors
        # 检测区域半径，如果self.detectors非空则为self.detectors中检测器的最大检测长度，否则为deArea
        self.deArea: float = max([detector.detectlenth for detector in self.detectors]) if self.detectors else deArea
        # 添加RSU通信器属性
        self.communicator = None

    @property
    def current_state(self) -> State:
        """
        Get the current state of the RSU.

        Returns:
            State: The current state of the RSU.
        """
        return self._current_state

    @current_state.setter
    def current_state(self, state: State) -> None:
        """
        Set the current state of the RSU.

        Args:
            state (State): The new state of the RSU.
        """
        self._current_state = state

    def __repr__(self) -> str:
        """
        Get the string representation of the RSU.

        Returns:
            str: The string representation of the RSU.
        """
        return f"RSU(id={self.id}, rsu_type={self.rsu_type}, " \
               f"current_state=(x={self.current_state.x}, y={self.current_state.y}))"

    def addDetector(self, detector: RSU_detector):
        self.detectors.append(detector)
        # 更新检测区域半径
        self.deArea = max(self.deArea, detector.detectlenth)

    def isInAoI(self, ego_lane: str, ego_pos: float, netInfo) -> bool:
        """
        判断RSU是否在ego车辆的AOI范围内
        通过检查Ego车辆所处车道，以及其所处车道上的位置与RSU在同一车道的检测器之间的距离是否小于AOI距离

        :param ego_lane: ego车辆当前所在的车道ID
        :param ego_pos: ego车辆在车道上的位置（距离车道起点的距离）
        :param netInfo: 网络信息对象
        :return: 如果RSU在AOI范围内返回True，否则返回False
        """
        # 遍历RSU的所有检测器
        for detector in self.detectors:
            # 检查检测器是否在与ego车辆相同的车道上
            if detector.lane == ego_lane:
                # 计算ego车辆位置与检测器位置之间的距离
                distance = abs(ego_pos - detector.pos)
                # 如果距离小于等于检测器的检测长度，则认为RSU在AOI范围内
                if distance <= detector.detectlenth:
                    return True
        # 如果没有找到在同一车道上的检测器，或者距离超出检测范围，则不在AOI内
        return False

    def export2Dict(self, netInfo) -> dict:
        """
        导出RSU信息为字典格式
        :param netInfo: 网络信息对象
        :return: 包含RSU信息的字典
        """
        return {
            'id': self.id,
            'x': self.current_state.x,
            'y': self.current_state.y,
            'deArea': self.deArea,
            'detectors-lanes': [{detector.id, detector.lane} for detector in self.detectors],
            'detectfreq': self.detectors[0].detectfreq if self.detectors and len(self.detectors) > 0 else 0.0
        }

    # 添加初始化通信器的方法
    def init_communication(self, communication_manager):
        """初始化RSU通信器"""
        if RSUCommunicator is not None:
            self.communicator = RSUCommunicator(self.id, self, communication_manager)
            self.communicator.rsu = self
        else:
            logging.warning("RSUCommunicator not available, communication not initialized")
    #9.16 检测在RSU探测器范围内的[车辆]，并将信息打包为Message类
    def detect_vehicles_in_range(self, vehicles: Dict[str, control_Vehicle], roadgraph, receive_message: Message) -> List[str]:
        """
        检测在RSU探测器范围内的车辆，并将信息打包为Message类
        Args:
            vehicles: 字典，包含所有车辆对象，键为车辆ID，值为control_Vehicle对象
            roadgraph: 路网信息对象，用于获取车道信息
            receive_message: 接收到的消息对象，用于排除发送者车辆
        Returns:
            List[str]: 包含检测到的车辆信息的字符串列表
        """
        detected_messages = []
        # 获取发送者车辆ID
        sender_id = receive_message.sender_id
        # 获取发送者车辆位置
        # 获取发送者车辆所处的车道ID
        sender_lane_id = vehicles[sender_id].lane_id
        sender_pos = vehicles[sender_id].current_state.s
        sender_vel = vehicles[sender_id].current_state.vel
        # 遍历所有检测器
        for detector in self.detectors:
            # 获取检测器所在的车道
            detector_lane_id = detector.lane
            if not detector_lane_id:
                continue
            # 获取检测器位置和检测范围
            detector_pos = detector.pos
            detect_length = detector.detectlenth
            # 遍历所有车辆，查找在该检测器所处车道上的车辆
            for vehicle_id, vehicle in vehicles.items():
                # 排除发送者车辆
                if vehicle_id == sender_id:
                    continue
                # 检查车辆是否在检测器所在的车道上
                if vehicle.lane_id == detector_lane_id:
                    # 获取车辆在车道上的位置
                    vehicle_pos = vehicle.current_state.s
                    # 计算车辆与检测器之间的距离
                    distance2detector = abs(vehicle_pos - detector_pos)
                    # 如果车辆在检测范围内
                    if distance2detector <= detect_length:
                        # 创建承载交通信息的互操作语言
                        vehicle_info = f"GetVehicleID({vehicle.id});\n"
                        # 1. 相对位置关系
                        if vehicle.lane_id == sender_lane_id:
                            if vehicle_pos >= sender_pos:
                                # if the vehicle is in front of sender
                                vehicle_info += f"VehicleInLane({vehicle_id},{sender_id},Front);\n"
                            else:
                                vehicle_info += f"VehicleInLane({vehicle_id},{sender_id},Rear);\n"
                        else:
                            # 获取发送者车道和当前车辆车道的对象
                            sender_lane = roadgraph.get_lane_by_id(sender_lane_id)
                            # 判断车辆是否在发送者的左车道
                            if (sender_lane and hasattr(sender_lane, 'left_lane') and 
                                sender_lane.left_lane() == vehicle.lane_id):
                                if vehicle_pos >= sender_pos:
                                    vehicle_info += f"VehicleLeftLane({vehicle_id},{sender_id},Front);\n"
                                else:
                                    vehicle_info += f"VehicleLeftLane({vehicle_id},{sender_id},Rear);\n"
                            # 判断车辆是否在发送者的右车道
                            elif (sender_lane and hasattr(sender_lane, 'right_lane') and 
                                  sender_lane.right_lane() == vehicle.lane_id):
                                if vehicle_pos >= sender_pos:
                                    vehicle_info += f"VehicleRightLane({vehicle_id},{sender_id},Front);\n"
                                else:
                                    vehicle_info += f"VehicleRightLane({vehicle_id},{sender_id},Rear);\n"
                        # 2. 相对速度关系
                        if vehicle.current_state.vel > sender_vel:
                            # if the speed of vehicle is greater than sender
                            vehicle_info += f"GreaterSpeed({vehicle_id},{sender_id});\n"
                        elif vehicle.current_state.vel < sender_vel:
                            # if the speed of vehicle is slower than sender
                            vehicle_info += f"SlowerSpeed({vehicle_id},{sender_id});\n"
                        else:
                            vehicle_info += f"EqualSpeed({vehicle_id},{sender_id});\n"
                        detected_messages.append(vehicle_info)
        return detected_messages

def create_rsu(rsu_info: Dict, rsu_type: RSUType) -> control_RSU:
    """
    Creates a new RSU instance based on the provided information.

    Args:
        rsu_info (Dict): Information about the RSU.
        rsu_type (RSUType): RSU type.

    Returns:
        control_RSU: A new RSU instance.
    """
    rsu_id = rsu_info["id"]
    pos_x = rsu_info["x"]
    pos_y = rsu_info["y"]
    deArea = rsu_info["deArea"]
    
    # Create detectors if they exist in the info
    detectors = []
    if "detectors" in rsu_info:
        for detector_info in rsu_info["detectors"]:
            detector = RSU_detector(
                id=detector_info["id"],
                lane=detector_info["lane"],
                pos=detector_info["pos"],
                detectlenth=detector_info['detectlenth'],
                detectfreq=detector_info['detectfreq']
            )
            detectors.append(detector)

    init_state = State(x=pos_x, y=pos_y)

    rsu_new = control_RSU(
        rsu_id=rsu_info["id"],
        init_state=init_state,
        rsu_type=rsu_type,
        length=5.0,  # Default value
        width=1.8,   # Default value
        detectors=detectors,
        deArea=deArea
    )

    return rsu_new


def create_rsu_lastseen(rsu_info: Dict, lastseen_rsu: control_RSU, rsu_type: RSUType) -> control_RSU:
    """
    Creates an RSU instance based on the last seen RSU information.

    Args:
        rsu_info (Dict): Information about the RSU.
        lastseen_rsu (control_RSU): The last seen RSU instance.
        rsu_type (RSUType): RSU type.

    Returns:
        control_RSU: A new RSU instance with updated information.
    """
    rsu = lastseen_rsu
    rsu.rsu_type = rsu_type
    rsu.current_state.x = rsu_info["x"]
    rsu.current_state.y = rsu_info["y"]
    
    if "deArea" in rsu_info:
        rsu.deArea = rsu_info["deArea"]

    return rsu