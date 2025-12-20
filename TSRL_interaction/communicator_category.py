"""
功能：通信器类别模块 - 包含所有通信器相关类
作者：Wu Hao
创建日期：2025-07-15
转移日期：2025-11-08
更新日期：2025-11-11 - 迁移核心类回vehicle_communication.py
"""
# 目标：将特定的通信器类保留在此文件
from __future__ import annotations
import glob
import logging
import os
import time
import uuid
from typing import Dict, List, Optional
import logger
from logger import Logger
from enum import Enum
from add.display import NonBlockingInferenceWindow, NonBlockingVehicleDisplayWindow

# 从vehicle_communication导入核心通信类
from TSRL_interaction.vehicle_communication import Communicator, CommunicationManager, Message, MessageList, Performative
class VehicleCommunicator(Communicator):
    """车辆通信器，负责车辆间通信"""
    def __init__(self, vehicle_id, vehicle: 'control_Vehicle', communication_manager: CommunicationManager, if_egoCar: bool = False):
        super().__init__(vehicle_id, communication_manager)
        self.if_egoCar = if_egoCar
        self.vehicle = vehicle
        self.id = vehicle_id
        # 添加vehicles和roadgraph参数的存储
        self.vehicles = None
        self.roadgraph = None
        # 注册到通信管理器
        communication_manager.register(self)
    
    # 添加设置vehicles和roadgraph的方法
    def set_context(self, vehicles: Dict[str, 'control_Vehicle'], roadgraph):
        """设置上下文参数"""
        self.vehicles = vehicles
        self.roadgraph = roadgraph
    
    # 定义方法：主动发送消息
    def send(self, content: str, target_id: str = None, target_category: Communicator = None, performative: Performative = Performative.Other):
        """主动发送消息"""
        # 检查内容是否为空或仅包含空白字符
        if not content or content.strip() == "":
            return  # 不发送空消息，避免产生空消息记录
        
        # 文本前缀，取决于是否为egoCar
        prefix = f"Send by HV {self.id}:" if self.if_egoCar else f"Send by RV {self.id}:"
        # 完整文本
        full_content = f"{prefix}{content}"
        # 发送消息
        message = Message(
            sender_id=self.id,
            sender_category=self,
            Receiver_id=target_id or "broadcast",
            Receiver_category=target_category,
            content=content,
            performative=performative
        )
        # 在终端输出
        print(full_content)
        # 保存显示文本
        self._save_display_text(full_content)
        # 存储消息到本地消息历史列表
        self.message_history.append_message(message)
        # 将消息真正发送出去
        self.communication_manager.send_message(message)
        # 保存消息历史
        self._save_message_history()

    def receive_message(self, message: Message):
        """接收消息并处理"""
        if message.sender_id == self.id:
            return  # 忽略自己发送的消息
        
        # 检查接收到的消息内容是否为空
        if not message.content or message.content.strip() == "":
            return  # 不处理空消息，避免产生空消息记录
        
        # 生成显示内容
        if self.if_egoCar:
            display_prefix = f"Received by HV {self.id}:"
        else:
            display_prefix = f"Received by RV {self.id}:"
        # 提取原始内容
        content = message.content
        display_content = f"{display_prefix}{content}"
        
        # 存储接收到的原始消息到本地列表
        # 创建一个新消息对象，记录接收到的消息
        received_message = Message(
            sender_id=message.sender_id,
            sender_category=message.sender_category,
            Receiver_id=self.id,
            Receiver_category=self,
            content=content,
            performative=message.performative
        )
        
        # 添加消息到本地历史
        self.message_history.append_message(received_message)
        
        # 保存显示文本
        self._save_display_text(display_content)
        
        # 保存消息历史到文件
        self._save_message_history()
        
        # 在终端输出接收信息
        print(display_content)
        
        # 根据接收到的内容执行相应操作
        self.process_received_content(message)

    def process_received_content(self, messages):
        """处理接收到的消息内容"""
        # 判断messages是否为列表，分别处理
        if isinstance(messages, list):
            for message in messages:
                self.Message_process(message)
        else:
            self.Message_process(messages)

    def Message_process(self, message: Message):
        """处理消息真实内容"""
        content = message.content
        # EmergencyStation相对位置关系判断
        if "EmergencyStation" in content:
            import re
            match = re.search(r'EmergencyStation\(([^)]+)\)', content)
            if match:
                params = match.group(1).split(',')
                # 获取EmergencyStation的发送者车辆ID（第1个参数）
                sender_id = params[0].strip()
                # 检查是否有vehicles和roadgraph上下文属性（需要从外部获取）
                if hasattr(self, 'vehicles') and hasattr(self, 'roadgraph'):
                    # 检查上下文是否已设置
                    if not self.vehicles or not self.roadgraph:
                        print(f"Warning: EmergencyStation context not set for vehicle {self.id}")
                        return None
                    # 调用control_Vehicle类的方法处理EmergencyStation消息
                    current_vehicle = self.vehicles.get(self.id)
                    if current_vehicle and hasattr(current_vehicle, 'handle_sender_location'):
                        reply_content = current_vehicle.handle_sender_location(sender_id, self.vehicles, self.roadgraph)
                        if reply_content:
                            self.send(reply_content, target_id=sender_id, performative=Performative.Inform)
                    else:
                        print(f"Warning: Vehicle {self.id} does not have handle_sender_location method")

class RSUCommunicator(Communicator):
    """路侧单元通信器，负责路侧单元的通信"""
    def __init__(self, rsu_id: str, rsu: 'control_RSU', communication_manager: CommunicationManager):
        super().__init__(rsu_id, communication_manager)
        self.id=rsu_id
        self.rsu = rsu
        self.category = self
        # 添加vehicles和roadgraph参数的存储
        self.vehicles = None
        self.roadgraph = None
        # 注册到通信管理器
        communication_manager.register(self)
    
    # 添加设置vehicles和roadgraph的方法
    def set_context(self, vehicles: Dict[str, 'control_Vehicle'], roadgraph):
        """设置上下文参数"""
        self.vehicles = vehicles
        self.roadgraph = roadgraph
    
    # 定义方法：主动发送消息
    def send(self, content: str, target_id: str = None, performative: Performative = Performative.Other):
        """主动发送消息"""
        # 检查内容是否为空或仅包含空白字符
        if not content or content.strip() == "":
            return  # 不发送空消息，避免产生空消息记录
        
        # 文本前缀
        prefix = f"Send by RSU {self.id}:"
        # 完整文本
        full_content = f"{prefix}{content}"
        # 发送消息
        message = Message(
            sender_id=self.id,
            sender_category=self,
            Receiver_id=target_id or "broadcast",
            Receiver_category=None,  # RSU发送消息时可能没有指定接收者类别
            content=content,  # 发送原始内容，不带前缀
            performative=performative
        )
        # 在终端输出
        print(full_content)
        # 保存显示文本
        self._save_display_text(full_content)
        # 存储消息到本地消息历史列表
        self.message_history.append_message(message)
        # 将消息发送出去
        self.communication_manager.send_message(message)
        # 保存消息历史
        self._save_message_history()

    def receive_message(self, message: Message):
        """接收消息并处理"""
        if message.sender_id == self.id:
            return  # 忽略自己发送的消息
        
        # 检查接收到的消息内容是否为空
        if not message.content or message.content.strip() == "":
            return  # 不处理空消息，避免产生空消息记录
        
        # 生成回复内容
        reply_prefix = f"Received by RSU {self.id}:"
        # 提取原始内容（去除发送者前缀）
        # 添加对消息内容格式的检查，防止索引越界
        if ":" in message.content:
            content = message.content.split(":", 1)[1].strip()
        else:
            # 如果消息内容中没有冒号，使用完整内容
            content = message.content
        reply_content = f"{reply_prefix}{content}"
        # 存储消息到本地列表
        reply_message = Message(
            sender_id=message.Receiver_id,
            sender_category=self,
            Receiver_id=message.sender_id,
            Receiver_category=message.sender_category,
            content=content,
            performative=Performative.Inform
        )
        # 添加消息
        self.message_history.append_message(reply_message)
        # 保存显示文本
        self._save_display_text(reply_content)
        # 保存消息历史
        self._save_message_history()
        # 在终端输出接收信息
        print(reply_content)
        # 根据接收到的内容执行相应操作
        self.process_received_content(message)

    def process_received_content(self, messages):
        """处理接收到的消息内容"""
        # 判断messages是否为列表，分别处理
        if isinstance(messages, list):
            for message in messages:
                self.Message_process(message)
        else:
            self.Message_process(messages)

    def Message_process(self, message: Message):
        """处理消息真实内容"""
        content = message.content
        # EmergencyStation相对位置关系判断
        if "EmergencyStation" in content:
            import re
            match = re.search(r'EmergencyStation\(([^)]+)\)', content)
            if match:
                params = match.group(1).split(',')
                # 获取EmergencyStation的发送者车辆ID（第1个参数）
                sender_id = params[0].strip()
                # 检查是否有vehicles和roadgraph上下文属性（需要从外部获取）
                if hasattr(self, 'vehicles') and hasattr(self, 'roadgraph'):
                    # 检查上下文是否已设置
                    if not self.vehicles or not self.roadgraph:
                        print(f"Warning: EmergencyStation context not set for RSU {self.id}")
                        return None
                    # 调用control_RSU类的方法处理EmergencyStation消息
                    current_rsu = self.rsu
                    if current_rsu and hasattr(current_rsu, 'handle_sender_location'):
                        reply_content = current_rsu.handle_sender_location(sender_id, self.vehicles, self.roadgraph)
                        if reply_content:
                            self.send(reply_content, target_id=sender_id, performative=Performative.Inform)
                    else:
                        print(f"Warning: RSU {self.id} does not have handle_sender_location method")
        elif "InformationRequest2RSU" in content:
            import re
            match = re.search(r'InformationRequest2RSU\(([^)]+)\)', content)
            if match:
                params = match.group(1).split(',')
                # 获取InformationRequest2RSU的发送者车辆ID（第1个参数）
                sender_id = params[0].strip()
                # 检查是否有vehicles和roadgraph上下文属性（需要从外部获取）
                if hasattr(self, 'vehicles') and hasattr(self, 'roadgraph'):
                    # 检查上下文是否已设置
                    if not self.vehicles or not self.roadgraph:
                        print(f"Warning: InformationRequest2RSU context not set for RSU {self.id}")
                        return None
                    # 调用control_RSU类的方法处理InformationRequest2RSU消息
                    current_rsu = self.rsu
                    if current_rsu and hasattr(current_rsu, 'detect_vehicles_in_range'):
                        reply_content = current_rsu.detect_vehicles_in_range(self.vehicles, self.roadgraph, message)
                        if reply_content:
                            for content in reply_content:
                                self.send(content, target_id=sender_id, performative=Performative.Inform)
                    else:
                        print(f"Warning: RSU {self.id} does not have handle_information_request method")
                else:
                    print(f"Warning: InformationRequest2RSU context not set for RSU {self.id}")
                    return None

class EnvCommunicator(Communicator):
    """环境通信器，负责环境信息的通信"""
    def __init__(self, env_id: str, communication_manager: CommunicationManager):
        super().__init__(env_id, communication_manager)
        self.id = env_id
        self.environment_adapter = None
        # 注册到通信管理器
        communication_manager.register(self)
    
    def set_context(self, environment_adapter):
        """设置环境适配器"""
        self.environment_adapter = environment_adapter
    
    def send(self, content: str, target_id: str = None, performative: Performative = Performative.Inform):
        """发送环境信息消息"""
        # 检查内容是否为空或仅包含空白字符
        if not content or content.strip() == "":
            return  # 不发送空消息，避免产生空消息记录
        
        # 文本前缀
        prefix = f"Send by Environment {self.id}:"
        # 完整文本
        full_content = f"{prefix}{content}"
        
        # 发送消息
        message = Message(
            sender_id=self.id,
            sender_category=self,
            Receiver_id=target_id or "broadcast",
            Receiver_category=None,
            content=content,
            performative=performative
        )
        
        # 在终端输出
        print(full_content)
        # 保存显示文本
        self._save_display_text(full_content)
        # 存储消息到本地消息历史列表
        self.message_history.append_message(message)
        # 将消息发送出去
        self.communication_manager.send_message(message)
        # 保存消息历史
        self._save_message_history()

    def receive_message(self, message: Message):
        """接收消息并处理"""
        if message.sender_id == self.id:
            return  # 忽略自己发送的消息
        
        # 检查接收到的消息内容是否为空
        if not message.content or message.content.strip() == "":
            return  # 不处理空消息，避免产生空消息记录
        
        # 生成回复内容
        reply_prefix = f"Received by Environment {self.id}:"
        # 提取原始内容（去除发送者前缀）
        if ":" in message.content:
            content = message.content.split(":", 1)[1].strip()
        else:
            content = message.content
        reply_content = f"{reply_prefix}{content}"
        
        # 存储消息到本地列表
        reply_message = Message(
            sender_id=message.Receiver_id,
            sender_category=self,
            Receiver_id=message.sender_id,
            Receiver_category=message.sender_category,
            content=content,
            performative=Performative.Inform
        )
        
        # 添加消息
        self.message_history.append_message(reply_message)
        # 保存显示文本
        self._save_display_text(reply_content)
        # 保存消息历史
        self._save_message_history()
        # 在终端输出接收信息
        print(reply_content)
        # 根据接收到的内容执行相应操作
        self.process_received_content(message)

    def process_received_content(self, messages):
        """处理接收到的消息内容"""
        # 判断messages是否为列表，分别处理
        if isinstance(messages, list):
            for message in messages:
                self.Message_process(message)
        else:
            self.Message_process(messages)

    def Message_process(self, message: Message):
        """处理消息真实内容"""
        content = message.content
        # 这里可以添加特定的环境消息处理逻辑
        # 例如：处理交叉口信息请求、环境状态查询等
        if "IsJunction" in content:
            # 处理交叉口信息
            if self.environment_adapter:
                junction_info = self.environment_adapter.get_junction_info()
                if junction_info:
                    reply_content = f"JunctionInfo: {junction_info}"
                    self.send(reply_content, target_id=message.sender_id, performative=Performative.Inform)