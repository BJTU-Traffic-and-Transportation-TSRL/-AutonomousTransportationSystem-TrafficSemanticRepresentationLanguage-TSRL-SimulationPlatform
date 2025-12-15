"""
Author: Wu Hao
Date: 2025-9-18
Description: TSRL Decision Maker for vehicle behavior decision based on temporal spatial logic rules

Copyright (c) 2023 by PJLab, All Rights Reserved. 
"""
from __future__ import annotations

import os
import subprocess
import re
import sys
import tkinter as tk
from tkinter import scrolledtext, messagebox
from typing import Dict, List, Optional
from abc import ABC, abstractmethod

from decision_maker.abstract_decision_maker import (
    AbstractEgoDecisionMaker,
    AbstractMultiDecisionMaker,
    EgoDecision,
    MultiDecision,
    SingleStepDecision,
)
from common.observation import Observation
from common.vehicle import Behaviour, control_Vehicle
from predictor.abstract_predictor import Prediction
from utils.roadgraph import RoadGraph
from utils.trajectory import State
from add.display import NonBlockingInferenceWindow
from TSRL_representation.TSRL import TSRL


import logger

logging = logger.get_logger(__name__)

# 新增的映射类，将动作名称映射到Behaviour枚举值
class action_name_to_behaviour_mapper:
    """
    将动作名称映射到Behaviour枚举值的映射类
    """
    # 定义动作名称到Behaviour的映射
    ACTION_TO_BEHAVIOUR = {
        "KeepLane": Behaviour.KL,
        "Accelerate": Behaviour.AC,
        "Decelerate": Behaviour.DC,
        "LeftChangeLane": Behaviour.LCL,  
        "RightChangeLane": Behaviour.LCR,
        "LetStop": Behaviour.STOP,
        "EnterJunction": Behaviour.IN_JUNCTION,
        "Overtake": Behaviour.OVERTAKE,
        "KL": Behaviour.KL,
        "AC": Behaviour.AC,
        "DC": Behaviour.DC,
        "IN_JUNCTION": Behaviour.IN_JUNCTION
    }
    # 定义：以下的推理结果都映射到Behaviour.SEND
    RESULT_TO_SEND = [
        "Congestion"
    ]
    # 定义：以下的推理结果需要对应的特殊处理方法
    RESULT_TO_SPECIAL_HANDLING = [
        "LetStopBeforeJunction"
    ]
    
    @classmethod
    def get_behaviour(cls, action_name: str) -> Behaviour:
        """
        根据动作名称获取对应的Behaviour枚举值
        Args:
            action_name (str): 动作名称
        Returns:
            Behaviour: 对应的Behaviour枚举值，如果未找到则返回Behaviour.OTHER
        """
        # 1. 直接查找动作名称是否在映射中
        # 1.1 如果动作名称直接在映射中，直接返回对应的Behaviour枚举值
        if action_name in cls.ACTION_TO_BEHAVIOUR:
            return cls.ACTION_TO_BEHAVIOUR[action_name]
        # 1.2 如果动作名称不在映射中，忽略大小写查找
        for key, value in cls.ACTION_TO_BEHAVIOUR.items():
            if key.lower() == action_name.lower():
                return value
        # 2. 如果是Behaviour枚举的字符串形式，直接返回对应的Behaviour枚举值
        try:
            return Behaviour[action_name.upper()]
        except KeyError:
            pass
        # 3. 如果是RESULT_TO_SEND中的结果，返回Behaviour.SEND
        if action_name in cls.RESULT_TO_SEND:
            return Behaviour.SEND
        # 4. 如果是RESULT_TO_SPECIAL_HANDLING中的结果，使用特殊处理方法，进行对应处理
        if action_name in cls.RESULT_TO_SPECIAL_HANDLING:
            return Behaviour.SPECIAL_HANDLING
        # 如果都没找到，返回OTHER
        return Behaviour.OTHER

class EgoDecisionMaker(AbstractEgoDecisionMaker):
    def __init__(self, Scenario_Name: str = None):
        # 获取项目根目录
        self.project_root = os.path.join(os.path.dirname(__file__), '..', '..')
        self.Scenario_Name = Scenario_Name
        self.message_history_dir = os.path.join(self.project_root, f'message_history\\{self.Scenario_Name}') # 消息历史文件目录
        self.rules_file = os.path.join(self.project_root, 'TSRL_inference', 'Rules', 'Roadsys_rule.txt') # 规则文件路径
        self.inference_input_dir = os.path.join(self.project_root, 'TSRL_inference', 'Inference_Input') # 推理输入文件目录
        self.inference_output_dir = os.path.join(self.project_root, 'TSRL_inference', 'Inference_Output') # 推理输出文件目录
        self.tsrl_script = os.path.join(self.project_root, 'TSRL_representation', 'TSRL.py') # TSRL脚本路径
        
        # 确保目录存在
        os.makedirs(self.inference_input_dir, exist_ok=True)
        os.makedirs(self.inference_output_dir, exist_ok=True)

    def _read_message_history(self, vehicle_id: str, max_messages: Optional[int] = None) -> List[str]:
        """读取指定车辆的消息历史"""
        message_file = os.path.join(self.message_history_dir, f'message_{vehicle_id}_history.txt')
        if not os.path.exists(message_file):
            logging.warning(f"Message history file for vehicle {vehicle_id} not found")
            return []
        
        try:
            with open(message_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                # 过滤空行
                messages = [line.strip() for line in lines if line.strip()]
                # 如果指定了最大消息数量，则只返回最新的max_messages条消息
                if max_messages is not None and max_messages > 0:
                    return messages[-max_messages:] if len(messages) > max_messages else messages
                return messages
        except Exception as e:
            logging.error(f"Error reading message history for vehicle {vehicle_id}: {e}")
            return []

    def _read_rules(self) -> List[str]:
        """读取所有规则"""
        if not os.path.exists(self.rules_file):
            logging.error(f"Rules file not found: {self.rules_file}")
            return []
        
        try:
            with open(self.rules_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                # 过滤空行和注释行
                return [line.strip() for line in lines if line.strip() and not line.startswith('#')]
        except Exception as e:
            logging.error(f"Error reading rules file: {e}")
            return []

    def _parse_rule(self, rule: str) -> tuple:
        """解析规则，返回左侧和右侧部分"""
        if ":-" not in rule:
            return None, None
        head, body = rule.split(":-", 1)
        head = head.strip()
        body = body.strip().rstrip(';')  # 移除结尾的分号
        conditions = [cond.strip() for cond in body.split('),')] # 进行条件切割
        return head, conditions

    def _check_conditions(self, conditions: List[str], message_history: List[str]) -> bool:
        """检查所有条件是否都在消息历史中"""
        for condition in conditions:
            # 检查条件是否在消息历史中
            found = False
            for msg in message_history:
                # 去除消息末尾的分号
                clean_msg = msg.rstrip(';')
                # 检查如果condition和msg都包含"("，则比较括号前的部分
                if "(" in condition and "(" in clean_msg:
                    condition_prefix = condition.split("(")[0]
                    msg_prefix = clean_msg.split("(")[0]
                    if condition_prefix == msg_prefix:
                        found = True
                        break
                else:
                    # 如果没有括号，则直接比较
                    if condition == clean_msg:
                        found = True
                        break
            
            if not found:
                return False
        return True

    def _generate_inference_input(self, vehicle_id: str, message_history: List[str], rule: str, head: str) -> str:
        """生成推理输入文件"""
        input_filename = f'Inference_{vehicle_id}_input.txt'
        input_filepath = os.path.join(self.inference_input_dir, input_filename)
        
        try:
            with open(input_filepath, 'w', encoding='utf-8') as f:
                # 写入消息历史
                for msg in message_history:
                    f.write(f"{msg}\n")
                f.write("\n")
                # 写入规则
                f.write(f"{rule}\n")
                f.write("\n")
                # 写入ASK语句
                f.write(f"ASK {head};\n")
            
            return input_filepath
        except Exception as e:
            logging.error(f"Error generating inference input for vehicle {vehicle_id}: {e}")
            return ""

    def _run_tsrl_inference(self, input_filepath: str, vehicle_id: str) -> str:
        """运行TSRL推理引擎"""
        output_filename = f'Inference_{vehicle_id}_output.txt'
        output_filepath = os.path.join(self.inference_output_dir, output_filename)
        try:
            TSRL.main(input_filepath, output_filepath)
            return output_filepath
        except Exception as e:
            logging.error(f"Error running TSRL inference for vehicle {vehicle_id}: {e}")
            return ""

    def _parse_inference_output(self, output_filepath: str, head: str) -> Optional[str]:
        """解析推理输出，提取变量替换后的决策"""
        if not os.path.exists(output_filepath):
            return None
        try:
            with open(output_filepath, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # 查找包含head的行
            for line in lines:
                line = line.strip()
                # 检查line是否包含冒号格式的映射关系
                if ':' in line and line.startswith('{') and line.endswith('}'):
                    # 解析line中的映射关系，例如 "{'y': '1'}"
                    try:
                        # 移除大括号
                        content = line[1:-1]  # 移除开头和结尾的大括号
                        # 解析键值对，注意处理逗号在引号内的情况
                        mapping = {}
                        # 简单处理，按逗号分割（假设格式简单）
                        pairs = content.split(',')
                        # 解析每个键值对
                        for pair in pairs:
                            # 分割键和值
                            if ':' in pair:
                                kv = pair.split(':', 1)  # 只分割第一个冒号
                                if len(kv) == 2:
                                    # 移除键值两端的空格和引号
                                    key = kv[0].strip().strip("'\"")
                                    value = kv[1].strip().strip("'\"")
                                    mapping[key] = value
                        
                        # 获取head中的变量名（括号内的内容）
                        if '(' in head and ')' in head:
                            head_prefix = head.split('(')[0]
                            head_var = head[head.find('(')+1:head.find(')')]
                            
                            # 检查head_var是否在映射中
                            if head_var in mapping:
                                # 替换head中的变量为映射值
                                replaced_head = f"{head_prefix}({mapping[head_var]})"
                                return replaced_head
                    except Exception as e:
                        logging.warning(f"Failed to parse mapping line: {line}, error: {e}")
                        continue
                elif line.startswith(head.split('(')[0]):  # 匹配谓词名称
                    return line
            
            return None
        except Exception as e:
            logging.error(f"Error parsing inference output: {e}")
            return None

    def _extract_action_from_head(self, head: str) -> str:
        """从规则头部提取行为名称"""
        if '(' in head:
            return head.split('(')[0]
        return head

    def _create_inference_display_window(self, title: str, content: str):
        """创建弹窗展示推理信息（非阻塞）"""
        try:
            # 获取非阻塞弹窗实例
            window = NonBlockingInferenceWindow.get_instance()
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
            logging.error(f"Error creating inference display window: {e}")

    def _get_current_time(self):
        """获取当前时间戳"""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _generate_detailed_inference_display_file(self, vehicle_id: str, message_history: List[str], rule: str,
                                                 input_filepath: str, output_filepath: str, decision_result: str):
        """生成详细的推理展示文件，包含输入、输出和决策结果，并在弹窗中展示"""
        try:
            # 创建推理展示目录
            inference_display_dir = os.path.join(self.project_root, 'TSRL_inference', 'Inference_display')
            os.makedirs(inference_display_dir, exist_ok=True)
            
            # 生成展示文件路径
            display_filename = f'Detailed_Inference_display_{self.Scenario_Name}_{vehicle_id}.txt'
            display_filepath = os.path.join(inference_display_dir, display_filename)
            
            # 准备展示内容
            content = f"=== TSRL详细推理展示 ===\n\n"
            content += f"车辆ID: {vehicle_id}\n"
            content += f"时间: {self._get_current_time()}\n\n"
            
            content += f"=== 匹配的规则 ===\n{rule}\n\n"
            
            content += f"=== 消息历史 ===\n"
            for i, msg in enumerate(message_history, 1):
                content += f"{i}. {msg}\n"
            content += "\n"
            
            # 读取推理输入文件内容
            content += f"=== 推理输入文件内容 ({os.path.basename(input_filepath)}) ===\n"
            if os.path.exists(input_filepath):
                with open(input_filepath, 'r', encoding='utf-8') as f:
                    content += f.read()
            else:
                content += "输入文件不存在\n"
            content += "\n"
            
            # 读取TSRL推理输出文件内容
            content += f"=== TSRL推理输出内容 ({os.path.basename(output_filepath)}) ===\n"
            if os.path.exists(output_filepath):
                with open(output_filepath, 'r', encoding='utf-8') as f:
                    content += f.read()
            else:
                content += "输出文件不存在\n"
            content += "\n"
            
            # 添加解析后的决策结果
            content += f"=== 解析后的决策结果 ===\n"
            if decision_result:
                content += f"{decision_result}\n"
            else:
                content += "无决策结果\n"
            
            # 写入展示文件
            with open(display_filepath, 'w', encoding='utf-8') as f:
                f.write(content)
            
            # 创建弹窗展示内容
            self._create_inference_display_window("TSRL详细推理展示", content)
            
            return display_filepath
        except Exception as e:
            logging.error(f"Error generating detailed inference display file: {e}")
            return None

    def make_decision(
        self,
        T: float,
        observation: Observation,
        road_graph: RoadGraph,
        prediction: Prediction = None,
        config: dict = None
    ) -> EgoDecision:
        """
        基于TSRL的自车决策器实现
        步骤：
        1. 读取自车消息历史文件
        2. 遍历规则文件中的每条规则
        3. 检查规则条件是否满足
        4. 生成推理输入文件
        5. 运行TSRL推理引擎
        6. 解析输出并生成决策
        """
        # 获取自车信息
        ego_vehicle = None
        for vehicle in observation.vehicles:
            if hasattr(vehicle, 'if_egoCar') and vehicle.if_egoCar:
                ego_vehicle = vehicle
                break
        if not ego_vehicle:
            logging.warning("No ego vehicle found in observation")
            return EgoDecision(ego_veh=None, result=[])
        # 初始化决策结果
        decision_result = []
        # 获取自车ID
        vehicle_id = str(ego_vehicle.id)
        # 读取该车辆的消息历史
        message_history = self._read_message_history(vehicle_id, max_messages=config["NUM_READMESSAGES"])
        if not message_history:
            logging.warning(f"No message history for ego vehicle {vehicle_id}")
            return EgoDecision(ego_veh=ego_vehicle, result=decision_result)
        # 读取规则
        rules = self._read_rules()
        if not rules:
            logging.warning("No rules found, skipping TSRL decision making")
            return EgoDecision(ego_veh=ego_vehicle, result=decision_result)
        # 遍历所有规则
        for rule in rules:
            if ":-" not in rule:
                continue
            head, conditions = self._parse_rule(rule)
            if not head or not conditions:
                continue
            # 检查规则条件是否满足
            if self._check_conditions(conditions, message_history):
                logging.debug(f"Rule conditions satisfied for ego vehicle {vehicle_id}: {rule}")
                # 生成推理输入文件
                input_filepath = self._generate_inference_input(vehicle_id, message_history, rule, head)
                if not input_filepath:
                    continue
                # 运行TSRL推理
                output_filepath = self._run_tsrl_inference(input_filepath, vehicle_id)
                if not output_filepath:
                    continue
                
                # 解析推理输出
                decision_output = self._parse_inference_output(output_filepath, head)
                if decision_output:
                    # 生成详细的推理展示文件并弹窗展示
                    self._generate_detailed_inference_display_file(vehicle_id, message_history, rule, input_filepath, output_filepath, decision_output)
                    # 创建决策
                    decision_at_t = SingleStepDecision()
                    decision_at_t.action = decision_output
                    decision_at_t.expected_time = T
                    # 根据规则头部确定行为类型
                    action_name = self._extract_action_from_head(head)
                    # 使用action_name_to_behaviour_mapper映射action_name到Behaviour
                    decision_at_t.behaviour = action_name_to_behaviour_mapper.get_behaviour(action_name)
                    if decision_at_t.behaviour is Behaviour.OTHER:
                        logging.info(f"Unknown behaviour for action {action_name}")
                    decision_result.append(decision_at_t)
                    logging.info(f"Decision made for ego vehicle {vehicle_id}: {decision_output}")
                    break  # 找到一个适用规则就停止
            else: 
                # 消息列表和当前这条推理规则不对应，所以进入下一个循环
                continue
        
        return EgoDecision(ego_veh=ego_vehicle, result=decision_result)


class MultiDecisionMaker(AbstractMultiDecisionMaker):
    def __init__(self, Scenario_Name: str = None):
        # 获取项目根目录
        self.project_root = os.path.join(os.path.dirname(__file__), '..', '..')
        self.Scenario_Name = Scenario_Name
        self.message_history_dir = os.path.join(self.project_root, f'message_history\\{self.Scenario_Name}') # 消息历史文件目录
        self.rules_file = os.path.join(self.project_root, 'TSRL_inference', 'Rules', 'Roadsys_rule.txt') # 规则文件路径
        self.inference_input_dir = os.path.join(self.project_root, 'TSRL_inference', 'Inference_Input') # 推理输入文件目录
        self.inference_output_dir = os.path.join(self.project_root, 'TSRL_inference', 'Inference_Output') # 推理输出文件目录
        self.tsrl_script = os.path.join(self.project_root, 'TSRL_representation', 'TSRL.py') # TSRL脚本路径
        
        # 确保目录存在
        os.makedirs(self.inference_input_dir, exist_ok=True)
        os.makedirs(self.inference_output_dir, exist_ok=True)
    
    def stop_vehicle(self, vehicle: control_Vehicle, complete_decisions: MultiDecision, T: float, config: dict):
        """为rou文件中有停车需要的车辆生成主动停车决策"""
        vehicle.current_state.stop_flag = True
        # 车辆即将到达停车位置，触发停车逻辑
        stop_decision = SingleStepDecision()
        stop_decision.action = f"Emergencystation({vehicle.id});"
        stop_decision.expected_time = T 
        stop_decision.behaviour = Behaviour.STOP
        complete_decisions.results[vehicle] = [stop_decision]
        return vehicle,complete_decisions
        
        
    def _read_message_history(self, vehicle_id: str, max_messages: Optional[int] = None) -> List[str]:
        """读取指定车辆的消息历史"""
        message_file = os.path.join(self.message_history_dir, f'message_{vehicle_id}_history.txt')
        if not os.path.exists(message_file):
            logging.warning(f"Message history file for vehicle {vehicle_id} not found")
            return []
        
        try:
            with open(message_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                # 过滤空行
                messages = [line.strip() for line in lines if line.strip()]
                # 如果指定了最大消息数量，则只返回最新的max_messages条消息
                if max_messages is not None and max_messages > 0:
                    return messages[-max_messages:] if len(messages) > max_messages else messages
                return messages
        except Exception as e:
            logging.error(f"Error reading message history for vehicle {vehicle_id}: {e}")
            return []

    def _read_rules(self) -> List[str]:
        """读取所有规则"""
        if not os.path.exists(self.rules_file):
            logging.error(f"Rules file not found: {self.rules_file}")
            return []
        
        try:
            with open(self.rules_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                # 过滤空行和注释行
                return [line.strip() for line in lines if line.strip() and not line.startswith('#')]
        except Exception as e:
            logging.error(f"Error reading rules file: {e}")
            return []

    def _parse_rule(self, rule: str) -> tuple:
        """解析规则，返回左侧和右侧部分"""
        if ":-" not in rule:
            return None, None
        head, body = rule.split(":-", 1)
        head = head.strip()
        body = body.strip().rstrip(';')  # 移除结尾的分号
        conditions = [cond.strip() for cond in body.split(',')]
        return head, conditions

    def _check_conditions(self, conditions: List[str], message_history: List[str]) -> bool:
        """检查所有条件是否都在消息历史中"""
        for condition in conditions:
            # 简单检查条件是否在消息历史中
            found = False  # 初始化found变量
            for msg in message_history:
                # 去除消息末尾的分号
                clean_msg = msg.rstrip(';')
                # 检查如果condition和msg都包含"("，则比较括号前的部分
                if "(" in condition and "(" in clean_msg:
                    condition_prefix = condition.split("(")[0]
                    msg_prefix = clean_msg.split("(")[0]
                    if condition_prefix == msg_prefix:
                        found = True
                        break
                else:
                    # 如果没有括号，则直接比较
                    if condition == clean_msg:
                        found = True
                        break
            
            if not found:
                return False
        return True

    def _generate_inference_input(self, vehicle_id: str, message_history: List[str], rule: str, head: str) -> str:
        """生成推理输入文件"""
        input_filename = f'Inference_{vehicle_id}_input.txt'
        input_filepath = os.path.join(self.inference_input_dir, input_filename)
        
        try:
            with open(input_filepath, 'w', encoding='utf-8') as f:
                # 写入消息历史
                for msg in message_history:
                    f.write(f"{msg}\n")
                f.write("\n")
                # 写入规则
                f.write(f"{rule}\n")
                f.write("\n")
                # 写入ASK语句
                f.write(f"ASK {head}\n")
            
            return input_filepath
        except Exception as e:
            logging.error(f"Error generating inference input for vehicle {vehicle_id}: {e}")
            return ""

    def _run_tsrl_inference(self, input_filepath: str, vehicle_id: str) -> str:
        """运行TSRL推理引擎"""
        output_filename = f'Inference_{vehicle_id}_output.txt'
        output_filepath = os.path.join(self.inference_output_dir, output_filename)
        
        try:
            # 构建命令
            cmd = f'python "{self.tsrl_script}" "{input_filepath}"'
            
            # 运行推理
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, cwd=os.path.dirname(self.tsrl_script))
            
            if result.returncode != 0:
                logging.error(f"TSRL inference failed for vehicle {vehicle_id}: {result.stderr}")
                return ""
            
            # 将输出保存到文件
            with open(output_filepath, 'w', encoding='utf-8') as f:
                f.write(result.stdout)
            
            return output_filepath
        except Exception as e:
            logging.error(f"Error running TSRL inference for vehicle {vehicle_id}: {e}")
            return ""

    def _parse_inference_output(self, output_filepath: str, head: str) -> Optional[str]:
        """解析推理输出，提取变量替换后的决策"""
        if not os.path.exists(output_filepath):
            return None
            
        try:
            with open(output_filepath, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # 查找包含head的行
            for line in lines:
                line = line.strip()
                # 检查line是否包含冒号格式的映射关系
                if ':' in line and line.startswith('{') and line.endswith('}'):
                    # 解析line中的映射关系，例如 "{'y': '1'}"
                    try:
                        # 移除大括号
                        content = line[1:-1]  # 移除开头和结尾的大括号
                        # 解析键值对，注意处理逗号在引号内的情况
                        mapping = {}
                        # 简单处理，按逗号分割（假设格式简单）
                        pairs = content.split(',')
                        # 解析每个键值对
                        for pair in pairs:
                            # 分割键和值
                            if ':' in pair:
                                kv = pair.split(':', 1)  # 只分割第一个冒号
                                if len(kv) == 2:
                                    # 移除键值两端的空格和引号
                                    key = kv[0].strip().strip("'\"")
                                    value = kv[1].strip().strip("'\"")
                                    mapping[key] = value
                        
                        # 获取head中的变量名（括号内的内容）
                        if '(' in head and ')' in head:
                            head_prefix = head.split('(')[0]
                            head_var = head[head.find('(')+1:head.find(')')]
                            
                            # 检查head_var是否在映射中
                            if head_var in mapping:
                                # 替换head中的变量为映射值
                                replaced_head = f"{head_prefix}({mapping[head_var]})"
                                return replaced_head
                    except Exception as e:
                        logging.warning(f"Failed to parse mapping line: {line}, error: {e}")
                        continue
                elif line.startswith(head.split('(')[0]):  # 匹配谓词名称
                    return line
            
            return None
        except Exception as e:
            logging.error(f"Error parsing inference output: {e}")
            return None

    def _extract_action_from_head(self, head: str) -> str:
        """从规则头部提取行为名称"""
        if '(' in head:
            return head.split('(')[0]
        return head

    def _get_current_time(self):
        """获取当前时间戳"""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _generate_detailed_inference_display_file(self, vehicle_id: str, message_history: List[str], rule: str,
                                                 input_filepath: str, output_filepath: str, decision_result: str):
        """生成详细的推理展示文件，包含输入、输出和决策结果，并在弹窗中展示"""
        try:
            # 创建推理展示目录
            inference_display_dir = os.path.join(self.project_root, 'TSRL_inference', 'Inference_display')
            os.makedirs(inference_display_dir, exist_ok=True)
            
            # 生成展示文件路径
            display_filename = f'Detailed_Inference_display_{self.Scenario_Name}_{vehicle_id}.txt'
            display_filepath = os.path.join(inference_display_dir, display_filename)
            
            # 准备展示内容
            content = f"=== TSRL详细推理展示 ===\n\n"
            content += f"车辆ID: {vehicle_id}\n"
            content += f"时间: {self._get_current_time()}\n\n"
            
            content += f"=== 匹配的规则 ===\n{rule}\n\n"
            
            content += f"=== 消息历史 ===\n"
            for i, msg in enumerate(message_history, 1):
                content += f"{i}. {msg}\n"
            content += "\n"
            
            # 读取推理输入文件内容
            content += f"=== 推理输入文件内容 ({os.path.basename(input_filepath)}) ===\n"
            if os.path.exists(input_filepath):
                with open(input_filepath, 'r', encoding='utf-8') as f:
                    content += f.read()
            else:
                content += "输入文件不存在\n"
            content += "\n"
            
            # 读取TSRL推理输出文件内容
            content += f"=== TSRL推理输出内容 ({os.path.basename(output_filepath)}) ===\n"
            if os.path.exists(output_filepath):
                with open(output_filepath, 'r', encoding='utf-8') as f:
                    content += f.read()
            else:
                content += "输出文件不存在\n"
            content += "\n"
            
            # 添加解析后的决策结果
            content += f"=== 解析后的决策结果 ===\n"
            if decision_result:
                content += f"{decision_result}\n"
            else:
                content += "无决策结果\n"
            
            # 写入展示文件
            with open(display_filepath, 'w', encoding='utf-8') as f:
                f.write(content)
            
            # 创建弹窗展示内容
            self._create_inference_display_window("TSRL详细推理展示", content)
            
            return display_filepath
        except Exception as e:
            logging.error(f"Error generating detailed inference display file: {e}")
            return None

    def _create_inference_display_window(self, title: str, content: str):
        """创建弹窗展示推理信息（非阻塞）"""
        try:
            # 获取非阻塞弹窗实例
            window = NonBlockingInferenceWindow.get_instance()
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
            logging.error(f"Error creating inference display window: {e}")

    def make_decision(
        self,
        T: float,
        observation: Observation,
        road_graph: RoadGraph,
        prediction: Prediction = None,
        config: dict = None
    ) -> MultiDecision:
        """
        基于TSRL的多车决策器实现
        步骤：
        1. 读取消息历史文件
        2. 遍历规则文件中的每条规则
        3. 检查规则条件是否满足
        4. 生成推理输入文件
        5. 运行TSRL推理引擎
        6. 解析输出并生成决策
        """
        complete_decisions = MultiDecision()
        # 获取所有需要决策的车辆,跳过AOI区域外的车和Ego车
        decision_vehicles = [veh for veh in observation.vehicles if veh.vtype != 'OUT_OF_AOI' and veh.vtype != "Ego_Car"]
        # 读取规则
        rules = self._read_rules()
        if not rules:
            logging.warning("No rules found, skipping TSRL decision making")
            return complete_decisions
        
        # 为每辆车做决策
        for vehicle in decision_vehicles:
            # 11.4 对stop_lane!=None的Vehicle进行主动停车
            if vehicle.stop_lane is not None and vehicle.lane_id in vehicle.stop_lane:
                vehicle,complete_decisions =self.stop_vehicle(vehicle, complete_decisions,T,config)
                continue # 主动停车决策优先级大于TSRL决策
            # TSRL决策
            vehicle_id = str(vehicle.id)
            # 读取该车辆的消息历史，默认读取最新的num_readmessages条消息
            message_history = self._read_message_history(vehicle_id, max_messages=config["NUM_READMESSAGES"])
            if not message_history:
                logging.warning(f"No message history for vehicle {vehicle_id}")
                continue
            
            # 遍历所有规则
            for rule in rules:
                if ":-" not in rule:
                    continue
                    
                head, conditions = self._parse_rule(rule)
                if not head or not conditions:
                    continue
                
                decision_result = None
                # 检查规则条件是否满足
                if self._check_conditions(conditions, message_history):
                    logging.debug(f"Rule conditions satisfied for vehicle {vehicle_id}: {rule}")
                    # 生成推理输入文件
                    input_filepath = self._generate_inference_input(vehicle_id, message_history, rule, head)
                    if not input_filepath:
                        continue
                    
                    # 运行TSRL推理
                    output_filepath = self._run_tsrl_inference(input_filepath, vehicle_id)  
                    if not output_filepath:
                        continue
                    # 解析推理输出
                    decision_result = self._parse_inference_output(output_filepath, head)
                    # 如果规则条件满足，生成推理展示文件并弹窗展示
                    if decision_result:
                        # 生成详细的推理展示文件并弹窗展示
                        self._generate_detailed_inference_display_file(vehicle_id, message_history, rule, input_filepath, output_filepath, decision_result)
                    if decision_result:
                        # 创建决策
                        decision_at_t = SingleStepDecision()
                        decision_at_t.action = decision_result
                        decision_at_t.expected_time = T  
                        # 根据规则头部确定行为类型
                        action_name = self._extract_action_from_head(head)
                        # 使用action_name_to_behaviour_mapper映射action_name到Behaviour
                        decision_at_t.behaviour = action_name_to_behaviour_mapper.get_behaviour(action_name)
                        if decision_at_t.behaviour is None:
                            logging.warning(f"Unknown behaviour for action {action_name}")
                            continue
                        complete_decisions.results[vehicle] = [decision_at_t]
                        logging.info(f"Decision made for vehicle {vehicle_id}: {decision_result}")
                        break  # 找到一个适用规则就停止
                else:
                    # 消息列表和推理规则没有对应，所以返回空决策
                    decision_at_t = SingleStepDecision()
                    decision_at_t.action = decision_result
                    decision_at_t.expected_time = T 
                    complete_decisions.results[vehicle] = [decision_at_t]

        return complete_decisions