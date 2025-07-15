"""
功能：车辆通信模块
作者：Wu Hao
创建日期：2025-07-15
"""

from __future__ import annotations
import time
from typing import Dict, List, Optional
from logger.logger import Logger

class Message:
    """消息类，封装车辆间通信内容"""
    def __init__(self, sender_id: str, receiver_id: str, content: str, timestamp: float = None):
        self.sender_id = sender_id
        self.receiver_id = receiver_id
        self.content = content
        self.timestamp = timestamp or time.time()

class CommunicationManager:
    """通信管理器，负责消息路由和分发"""
    def __init__(self):
        self.subscribers: Dict[str, VehicleCommunicator] = {} # 订阅者列表
        self.logger = Logger.get_instance() # 日志记录器
        self.message_history: List[Message] = [] # 消息历史记录列表

    def register_vehicle(self, vehicle: VehicleCommunicator):
        """注册车辆通信器"""
        self.subscribers[vehicle.vehicle_id] = vehicle

    def send_message(self, message: Message):
        """发送消息并路由到接收者"""
        self.message_history.append(message)
        
        # 记录消息到日志
        self.logger.info(f"Message sent: {message.sender_id} -> {message.receiver_id}: {message.content}")
        
        # 直接发送给目标车辆
        if message.receiver_id in self.subscribers:
            self.subscribers[message.receiver_id].receive_message(message)
        else:
            # 如果接收者不存在，广播给所有车辆
            for vehicle_id, vehicle in self.subscribers.items():
                if vehicle_id != message.sender_id:
                    vehicle.receive_message(message)

class VehicleCommunicator:
    """车辆通信器基类"""
    def __init__(self, vehicle_id: str, communication_manager: CommunicationManager):
        self.vehicle_id = vehicle_id
        self.communication_manager = communication_manager
        self.is_self_vehicle = False  # 是否为自车
        self.logger = Logger.get_instance()
        
        # 注册到通信管理器
        communication_manager.register_vehicle(self)

    def send主动消息(self, content: str, target_id: str = None):
        """主动发送消息"""
        prefix = "Send by HV:" if self.is_self_vehicle else "Send by RV:"
        full_content = f"{prefix}{content}"
        
        # 发送消息
        message = Message(
            sender_id=self.vehicle_id,
            receiver_id=target_id or "broadcast",
            content=full_content
        )
        self.communication_manager.send_message(message)
        
        # 在终端输出
        print(full_content)

    def receive_message(self, message: Message):
        """接收消息并处理"""
        if message.sender_id == self.vehicle_id:
            return  # 忽略自己发送的消息

        # 生成回复内容
        if self.is_self_vehicle:
            reply_prefix = "Received by HV:"
        else:
            reply_prefix = "Received by RV:"

        # 提取原始内容（去除发送者前缀）
        content = message.content.split(":", 1)[1].strip()
        reply_content = f"{reply_prefix}{content}"
        
        # 在终端输出接收信息
        print(reply_content)
        
        # 根据接收到的内容执行相应操作
        self.process_received_content(content)

    def process_received_content(self, content: str):
        """处理接收到的消息内容，子类可重写"""
        # 可以在这里添加根据消息内容触发车辆行为的逻辑
        pass

class HvCommunicator(VehicleCommunicator):
    """自车(HV)通信器"""
    def __init__(self, vehicle_id: str, communication_manager: CommunicationManager):
        super().__init__(vehicle_id, communication_manager)
        self.is_self_vehicle = True

    def process_received_content(self, content: str):
        """处理接收到的消息内容"""
        # 示例：如果接收到紧急停车消息，可以触发相应处理
        if content.startswith("EmergencyStation"):
            # 这里可以添加自车收到紧急停车消息后的逻辑
            self.logger.warning(f"HV {self.vehicle_id} received emergency message: {content}")
            # 例如：触发自车的紧急响应
            # self.vehicle.emergency_response()

class RvCommunicator(VehicleCommunicator):
    """他车(RV)通信器"""
    def __init__(self, vehicle_id: str, communication_manager: CommunicationManager):
        super().__init__(vehicle_id, communication_manager)
        self.is_self_vehicle = False

    def send_emergency_stop_message(self):
        """发送紧急停车消息"""
        self.send主动消息(f"EmergencyStation({self.vehicle_id})")

# 全局通信管理器实例
_global_communication_manager = None

def get_communication_manager() -> CommunicationManager:
    """获取全局通信管理器实例"""
    global _global_communication_manager
    if not _global_communication_manager:
        _global_communication_manager = CommunicationManager()
    return _global_communication_manager