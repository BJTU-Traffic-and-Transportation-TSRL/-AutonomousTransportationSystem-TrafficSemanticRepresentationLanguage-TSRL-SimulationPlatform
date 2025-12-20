"""
功能：车辆通信模块
作者：Wu Hao
创建日期：2025-07-15
更新日期：2025-11-08 - 重构：通信器类转移到communicator_category.py
"""

# 目标：7.17，写入每辆车的存储库，存储车辆的交流消息
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

# 迁移回vehicle_communication.py的核心通信类
class Performative(str, Enum):
    Inform = "Inform" # 告知
    Query = "Query" # 询问
    Request = "Request" # 请求
    Request_whenever = "Request-whenever" # 每次……即请求
    Accept = "Accept" # 接收
    Refuse = "Refuse" # 拒绝
    Failure = "Failure" # 失败
    Confuse = "Confuse" # 疑惑
    Other = 'None' # 其他


# 9.16 定义语义信息类
class Message:
    """消息类，封装语义交互信息体内容
    
    根据FIPA ACL消息标准，消息包含以下参数：
    a) Message-Identifier：消息体标识符，用于唯一地表示消息帧中的消息体
    b) Performative：述行词，表示消息交互的行为类型
    c) Sender：发送者，表示消息发送者的身份
    d) Sender-Category：发送者类别，表示发送者的角色或分类
    e) Receiver：接收者，表示消息的预期接收者的身份
    f) Receiver-Category：接收者类别，表示接收者的角色或分类
    g) Reply-To：消息回复对象，指示此对话线程中的后续消息将定向到的主体
    h) Content：消息内容，表示消息的内容，为交互语句
    i) Language：消息语言，表示表达参数Content消息内容所使用的语言
    j) Ontology：本体，表示用于赋予参数Content消息内容中的符号含义的本体
    k) Protocol：通信协议，表示发送代理在消息中采用的底层通信协议
    l) Conversation-Identifier：会话标识符，表示正在进行的消息序列所属哪个会话
    m) Reply-With：响应标识符，表示响应主体将使用该表达式来识别此消息
    n) In-Reply-To：回复标识符，表示该消息为此前较早消息的回复消息
    o) Reply-By：答复最晚时间，表示发送者希望接收者答复的最晚时间
    """
    
    def __init__(
        self,
        sender_id: str,# 发送者
        sender_category: Communicator,# 发送者类别
        Receiver_id: str,# 接收者
        Receiver_category: Communicator,# 接收者类别
        content: str, # 消息内容
        performative: Performative,# 述行词
        message_id: Optional[str] = None, # 消息体标识符
        reply_to: Optional[str] = None,# 消息回复对象
        language: Optional[str] = None,# 消息语言
        ontology: Optional[str] = None,# 本体，用于赋予参数Content消息内容中的符号含义的本体
        protocol: Optional[str] = None,# 通信协议，表示发送代理在消息中采用的底层通信协议
        conversation_id: Optional[str] = None,# 会话标识符，表示正在进行的消息序列所属哪个会话
        reply_with: Optional[str] = None,# 响应标识符，表示响应主体将使用该表达式来识别此消息
        in_reply_to: Optional[str] = None,# 回复标识符，表示该消息为此前较早消息的回复消息
        reply_by: Optional[float] = None,# 答复最晚时间，表示发送者希望接收者答复的最晚时间
        timestamp: Optional[float] = None# 时间戳
    ):
        # 必需参数
        self.message_id = message_id or str(uuid.uuid4())  # 消息体标识符
        self.sender_id = sender_id  # 发送者
        self.sender_category = sender_category  # 发送者类别
        self.Receiver_id = Receiver_id  # 接收者
        self.Receiver_category = Receiver_category  # 接收者类别
        self.content = content  # 消息内容
        self.performative = performative  # 述行词
        self.timestamp = timestamp or time.time()  # 时间戳
        
        # 可选参数
        self.reply_to = reply_to  # 消息回复对象
        self.language = language  # 消息语言
        self.ontology = ontology  # 本体
        self.protocol = protocol  # 通信协议
        self.conversation_id = conversation_id or str(uuid.uuid4())  # 会话标识符
        self.reply_with = reply_with  # 响应标识符
        self.in_reply_to = in_reply_to  # 回复标识符
        self.reply_by = reply_by  # 答复最晚时间

    def __str__(self) -> str:
        """返回消息的字符串表示"""
        return f"Message(id={self.message_id}, sender={self.sender_id}, Receiver={self.Receiver_id}, performative={self.performative}, content={self.content})"

    def __repr__(self) -> str:
        """返回消息的详细表示"""
        return self.__str__()
        
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
            print(f"{msg.sender_id} -> {msg.Receiver_id}: {msg.content}\n")
    
    # 定义方法：将当前车辆的消息列表保存到当前文件夹的文本文档中
    def save_message_list(self, vehicle_id: str, loc: str):
        """将消息列表保存到当前文件夹的文本文档中"""
        # 确保路径存在
        os.makedirs(loc, exist_ok=True)
        file_path = os.path.join(loc, f"message_{vehicle_id}_history.txt")
        with open(file_path, "a") as file:
            # 9.18 先将当前文件中的消息列表清空
            file.truncate(0)
            for msg in self.message_list:
                # file.write(f"{msg.sender_id} -> {msg.Receiver_id}: {msg.content}\n")
                file.write(f"{msg.content}\n")

class Communicator:
    """基础通信器，作为其他通信器的基类"""
    def __init__(self, id: str, communication_manager: CommunicationManager):
        self.id = id  # 交通主体ID
        self.communication_manager = communication_manager  # 通信管理器
        self.message_history: MessageList = MessageList()  # 消息历史列表
        self.logger = logger.get_logger(__name__)  # 日志记录器
        self.Scenario_Name = self.communication_manager.Scenario_Name
        communication_manager.register(self)

    def _save_display_text(self, content: str):
        """保存显示文本到文件"""
        # 确保目录存在
        os.makedirs(f"message_history/{self.Scenario_Name}", exist_ok=True)
        # 将内容保存至当前文件夹的display_text.txt文件中，并换行
        with open(f"message_history/{self.Scenario_Name}/display_text.txt", "a", encoding="utf-8") as file:
            file.write(content + "\n")

    def _save_message_history(self):
        """保存消息历史到文件"""
        # 将消息列表在message_history文件夹中的文本文件中打印出来
        self.message_history.save_message_list(self.id, loc=f"message_history/{self.Scenario_Name}")
    
class CommunicationManager:
    """通信管理器，负责消息路由和分发"""
    def __init__(self, Scenario_Name: str):
        self.subscribers: Dict[str, Communicator] = {} # 订阅者列表
        self.logger = logger.get_logger(__name__)# 日志记录器
        self.message_history: List[Message] = [] # 全局消息历史记录列表
        self.Scenario_Name = Scenario_Name

    def register(self, communicator: Communicator):
        """将通信器注册在通信管理器"""
        self.subscribers[communicator.id] = communicator
    
    # def register_vehicle(self, vehicle: VehicleCommunicator):
    #     """将车辆注册在通信管理器"""
    #     self.subscribers[vehicle.vehicle_id] = vehicle
    
    # def register_rsu(self, rsu: RSUCommunicator):
    #     """将RSU注册在通信管理器"""
    #     self.subscribers[rsu.rsu_id] = rsu

    def send_message(self, message: Message):
        """发送消息并路由到接收者"""
        # 记录消息到日志
        self.logger.info(f"Message sent: {message.sender_category}{message.sender_id} -> {message.Receiver_category}{message.Receiver_id}: {message.content}")
        # 直接发送给目标接收者
        target_found = False
        for subscriber_id, subscriber in self.subscribers.items():
            # 检查接收者ID是否匹配
            if subscriber_id == message.Receiver_id:
                # 发送消息给匹配的接收者，不检查类别
                subscriber.receive_message(message)
                target_found = True
                break
        # 如果没有找到特定接收者，广播给所有通信器（除了发送者本身）
        if not target_found:
            for communicator_id, communicator in self.subscribers.items():
                if communicator_id != message.sender_id:
                    communicator.receive_message(message)
    
    # 8.19 新增方法：删除所有消息历史文件
    def cleanup_message_files(self):
        """删除所有消息历史文件"""
        try:
            # 获取当前目录下所有message_*.txt文件
            pattern = "message_*_history.txt"
            files_to_remove = glob.glob(pattern)
            
            for file_path in files_to_remove:  # 修复：将原来的"path"改为"file_path"
                try:
                    os.remove(file_path)
                    self.logger.info(f"Message_history file: {file_path} deleted")
                except Exception as e:
                    self.logger.error(f"Deleting {file_path} Error: {e}")
                    
            self.logger.info(f"{len(files_to_remove)} message_history files deleted")
        except Exception as e:
            self.logger.error(f"Error deleting message_history files: {e}")

    # 8.19 新增方法：清空所有消息历史文件内容，而不删除文件
    def clear_message_files_content(self):
        """清空所有消息历史文件的内容（保留文件）"""
        try:
            # 获取当前目录下message_history文件夹下所有message_*.txt文件
            loc = "message_history"
            pattern = os.path.join(loc, "message_*_history.txt")
            files_to_clear = glob.glob(pattern)
            
            for file_path in files_to_clear:
                try:
                    # 以写入模式打开文件，清空内容
                    with open(file_path, 'w') as file:
                        file.truncate(0)  # 清空文件内容
                    self.logger.info(f"Message_history file content cleared: {file_path}")
                except Exception as e:
                    self.logger.error(f"Error clearing content of {file_path}: {e}")
                    
            self.logger.info(f"{len(files_to_clear)} message_history files content cleared")
        except Exception as e:
            self.logger.error(f"Error clearing message_history files content: {e}")
    
    # 8.27 删除display_text文件
    def cleanup_display_text(self,loc: str):
        """删除display_text文件"""
        try:
            # 获取特定目录下所有display_text文件
            file_path = os.path.join(loc, "display_text.txt")
            # 检查文件是否存在再删除
            if os.path.exists(file_path):
                # 删除文件
                os.remove(file_path)
                self.logger.info(f"已删除文件: {file_path}")
            else:
                self.logger.info(f"文件不存在，无需删除: {file_path}")
        except Exception as e:
            self.logger.error(f"删除文件时出错: {e}")

# 展示 display_text.txt 文件内容
    def show_display_text(self, Scenario_Name: str):
        try:
            display_filepath = os.path.join('message_history', Scenario_Name , 'display_text.txt')
            # 检查文件是否存在
            if os.path.exists(display_filepath):
                with open(display_filepath, 'r', encoding='utf-8') as file:
                    display_content = file.read()
                self._create_display_window("交通场景互操作语言交互展示", display_content)
            else:
                # 如果文件不存在，创建一个空文件
                os.makedirs(os.path.dirname(display_filepath), exist_ok=True)
                with open(display_filepath, 'w', encoding='utf-8') as file:
                    pass  # 创建空文件
                self._create_display_window("交通场景互操作语言交互展示", "暂无内容")
        except Exception as e:
            self.logger.error(f"Error showing display text: {e}")
    
    def _create_display_window(self, title: str, content: str):
        """创建弹窗展示 display_text.txt 文件内容（非阻塞）"""
        try:
            # 获取非阻塞弹窗实例
            window = NonBlockingVehicleDisplayWindow.get_instance()
            # 显示窗口（如果尚未显示）
            window.show_window(title)
            # 等待窗口初始化完成
            import time
            start_time = time.time()
            while not window.is_window_running() and (time.time() - start_time) < 5:
                time.sleep(0.1)
            # 更新窗口内容
            window.update_content(content)
        except Exception as e:
            logging.error(f"Error creating vehicle display window: {e}")

    def _get_current_time(self):
        """获取当前时间戳"""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    # 8.27 清除display_text文件里的内容，而不删除文件
    def clear_display_text_content(self,loc:str):
        """清空display_text.txt文件的内容（保留文件）"""
        try:
            # 获取特定目录下display_text.txt文件
            file_path = os.path.join(loc, "display_text.txt")
            # 检查文件是否存在
            if os.path.exists(file_path):
                try:
                    # 以写入模式打开文件，清空内容
                    with open(file_path, 'w', encoding='utf-8') as file:
                        file.truncate(0)  # 清空文件内容
                    self.logger.info(f"已清空display_text文件内容: {file_path}")
                except Exception as e:
                    self.logger.error(f"清空文件 {file_path} 内容时出错: {e}")
            else:
                self.logger.info(f"文件不存在，无需清空: {file_path}")
        except Exception as e:
            self.logger.error(f"清空display_text文件内容时出错: {e}")

