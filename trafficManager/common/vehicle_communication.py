"""
功能：车辆通信模块
作者：Wu Hao
创建日期：2025-07-15
"""

# 目标：7.17，写入每辆车的存储库，存储车辆的交流消息
from __future__ import annotations
import glob
import os
import time
from typing import Dict, List, Optional
import logger
from logger import Logger

class Message:
    """消息类，封装车辆间通信内容"""
    def __init__(self, sender_id: str, receiver_id: str, content: str, timestamp: float = None):
        self.sender_id = sender_id
        self.receiver_id = receiver_id
        self.content = content
        self.timestamp = timestamp or time.time()

# 定义消息列表
class MessageList:
    def __init__(self):
        self.message_list: List[Message] = []
    
    # 定义方法：将消息添加到列表
    def append_message(self, message: Message):
        """将消息添加到列表"""
        self.message_list.append(message)
    
    # 定义方法：打印消息列表
    def print_message_list(self):
        """打印消息列表"""
        for msg in self.message_list:
            print(f"{msg.sender_id} -> {msg.receiver_id}: {msg.content}\n")
    
    # 定义方法：将当前车辆的消息列表保存到当前文件夹的文本文档中
    def save_message_list(self, vehicle_id: str, loc: str):
        """将消息列表保存到当前文件夹的文本文档中"""
        # 确保路径存在
        os.makedirs(loc, exist_ok=True)
        file_path = os.path.join(loc, f"message_{vehicle_id}_history.txt")
        with open(file_path, "a") as file:
            for msg in self.message_list:
                file.write(f"{msg.sender_id} -> {msg.receiver_id}: {msg.content}\n")

class CommunicationManager:
    """通信管理器，负责消息路由和分发"""
    def __init__(self):
        self.subscribers: Dict[str, VehicleCommunicator] = {} # 订阅者列表
        self.logger = logger.get_logger(__name__)# 日志记录器
        self.message_history: List[Message] = [] # 全局消息历史记录列表

    def register_vehicle(self, vehicle: VehicleCommunicator):
        """将车辆注册在通信管理器"""
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
    
    # 8.19 新增方法：删除所有消息历史文件
    def cleanup_message_files(self):
        """删除所有消息历史文件"""
        try:
            # 获取当前目录下所有message_*.txt文件
            pattern = "message_*_history.txt"
            files_to_remove = glob.glob(pattern)
            
            for file_path in files_to_remove:
                try:
                    os.remove(file_path)
                    self.logger.info(f"已删除消息历史文件: {file_path}")
                except Exception as e:
                    self.logger.error(f"删除文件 {file_path} 时出错: {e}")
                    
            self.logger.info(f"共清理了 {len(files_to_remove)} 个消息历史文件")
        except Exception as e:
            self.logger.error(f"清理消息历史文件时出错: {e}")

    # 8.19 新增方法：清空所有消息历史文件内容，而不删除文件
    def clear_message_files_content(self):
        """清空所有消息历史文件的内容（保留文件）"""
        try:
            # 获取当前目录下所有message_*.txt文件
            
            pattern = "message_*_history.txt"
            files_to_clear = glob.glob(pattern)
            
            for file_path in files_to_clear:
                try:
                    # 以写入模式打开文件，清空内容
                    with open(file_path, 'w') as file:
                        file.truncate(0)  # 清空文件内容
                    self.logger.info(f"已清空消息历史文件内容: {file_path}")
                except Exception as e:
                    self.logger.error(f"清空文件 {file_path} 内容时出错: {e}")
                    
            self.logger.info(f"共清空了 {len(files_to_clear)} 个消息历史文件的内容")
        except Exception as e:
            self.logger.error(f"清空消息历史文件内容时出错: {e}")
    
    # 8.27 删除display_text文件
    def cleanup_display_text(self,loc: str):
        """删除display_text文件"""
        # 获取特定目录下所有display_text文件
        file_path = os.path.join(loc, "display_text.txt")
        try:
            # 删除文件
            os.remove(file_path)
            self.logger.info(f"已删除文件: {file_path}")
        except Exception as e:
            self.logger.error(f"删除文件{file_path}时出错: {e}")


    # 8.27 清除display_text文件里的内容，而不删除文件
    def clear_display_text_content(self,loc:str):
        """清空display_text.txt文件的内容（保留文件）"""
        try:
            # 获取特定目录下display_text.txt文件
            file_path = os.path.join(loc, "display_text.txt")
            try:
                # 以写入模式打开文件，清空内容
                with open(file_path, 'w', encoding='utf-8') as file:
                    file.truncate(0)  # 清空文件内容
                self.logger.info(f"已清空display_text文件内容: {file_path}")
            except Exception as e:
                self.logger.error(f"清空文件 {file_path} 内容时出错: {e}")
                
        except Exception as e:
            self.logger.error(f"清空display_text文件内容时出错: {e}")


class VehicleCommunicator:
    """车辆通信器，负责车辆间通信"""
    def __init__(self, vehicle_id: str, communication_manager: CommunicationManager,if_egoCar: bool = False):
        self.if_egoCar = if_egoCar
        self.vehicle_id = vehicle_id
        self.communication_manager = communication_manager
        self.logger = logger.get_logger(__name__)
        # 初始化存储信息的列表
        self.message_history: MessageList = MessageList()
        # 注册到通信管理器
        communication_manager.register_vehicle(self)
    
    # 定义方法：主动发送消息
    def send(self, content: str, target_id: str = None):
        """主动发送消息"""
        # 文本前缀，取决于是否为egoCar
        prefix = f"Send by HV {self.vehicle_id}:" if self.if_egoCar else f"Send by RV {self.vehicle_id}:"
        # 完整文本
        full_content = f"{prefix}{content}"
        # 发送消息
        message = Message(
            sender_id=self.vehicle_id,
            receiver_id=target_id or "broadcast",
            content=full_content
        )
        # 在终端输出
        print(full_content)

        # 将full_content保存至当前文件夹的display_text.txt文件中，并换行
        with open("message_history/display_text.txt", "a", encoding="utf-8") as file:
            file.write(full_content + "\n")
        # 存储消息到本地消息历史列表
        self.message_history.append_message(message)
        # 将消息发送出去
        self.communication_manager.send_message(message)
        # 将本地列表在当前文件夹中的文本文件中打印出来
        self.message_history.save_message_list(self.vehicle_id,loc='message_history')

    def receive_message(self, message: Message):
        """接收消息并处理"""
        if message.sender_id == self.vehicle_id:
            return  # 忽略自己发送的消息
        # 生成回复内容
        if self.if_egoCar:
            reply_prefix = f"Received by HV {self.vehicle_id}:"
        else:
            reply_prefix = f"Received by RV {self.vehicle_id}:"

        # 提取原始内容（去除发送者前缀）
        content = message.content.split(":", 1)[1].strip()
        reply_content = f"{reply_prefix}{content}"
        # 存储消息到本地列表
        reply_message=Message(
            sender_id=message.receiver_id,
            receiver_id=message.sender_id,
            content=reply_content
        )
        # 添加消息
        self.message_history.append_message(reply_message)
        # 保存至当前文件夹的display_text.txt文件中，并换行
        with open("message_history/display_text.txt", "a", encoding="utf-8") as file:
            file.write(reply_content + "\n")
        # 将本地列表在当前文件夹中的文本文件中打印出来
        self.message_history.save_message_list(self.vehicle_id, loc='message_history')
        # 在终端输出接收信息
        print(reply_content)
        # 根据接收到的内容执行相应操作
        self.process_received_content(content)

    def process_received_content(self, content: str):
        """处理接收到的消息内容"""
        # 示例：如果接收到紧急停车消息，可以触发相应处理
        if content.startswith("EmergencyStation"):
            # 这里可以添加自车收到紧急停车消息后的逻辑
            self.logger.warning(f"HV {self.vehicle_id} received emergency message: {content}")
            # 例如：触发自车的紧急响应
            # self.vehicle.emergency_response()


#  全局通信管理器实例
# _global_communication_manager = None

#  def get_communication_manager() -> CommunicationManager:
#      """获取全局通信管理器实例"""
#      global _global_communication_manager
#      if not _global_communication_manager:
#          _global_communication_manager = CommunicationManager()
#      return _global_communication_manager