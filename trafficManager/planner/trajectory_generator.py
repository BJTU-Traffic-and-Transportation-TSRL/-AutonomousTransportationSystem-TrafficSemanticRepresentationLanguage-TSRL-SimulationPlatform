"""
Author: Licheng Wen
Date: 2022-06-15 10:19:19
Description: 
Planner for a single vehicle

Copyright (c) 2022 by PJLab, All Rights Reserved. 
"""

import math
from typing import List
import numpy as np

from common.vehicle import control_Vehicle, Behaviour
from common import cost
from TSRL_interaction.vehicle_communication import Performative

from utils.roadgraph import AbstractLane, JunctionLane, RoadGraph,NormalLane
from utils.obstacles import ObsType, Obstacle
from utils.trajectory import State, Trajectory

from trafficManager.planner.frenet_optimal_planner import frenet_optimal_planner
from trafficManager.decision_maker.abstract_decision_maker import SingleStepDecision

import logger

logging = logger.get_logger(__name__)

# 全局停车栈，用于记录停车状态变化的车辆
stopped_vehicles_stack = set()  # 使用集合来存储当前停车的车辆ID

# 全局StopAt消息跟踪器，防止重复发送相同的StopAt消息
# 格式: {(stop_target_id, stop_location_id): sender_vehicle_id}
sent_stopat_messages = {}

# 检查路径
def check_path(vehicle, path):
    for state in path.states:
        if state.vel > vehicle.max_speed:  # Max speed check
            # print("Max speed violation", vehicle.id, state.vel)
            return False
        elif (state.s_dd > vehicle.max_accel or
              state.s_dd < vehicle.max_decel):  # Max acceleration check
            # print("Max accel violation ", vehicle.id, state.s_dd)
            return False
        # elif abs(state.cur) > config["MAX_CURVATURE"]:  # Max curvature check
        #     print("Max curvature exceeded")
        #     return False

    if path.cost == math.inf:  # Collision check
        return False
    else:
        return True

# 车道变换轨迹生成器
def lanechange_trajectory_generator(
    vehicle: control_Vehicle,
    target_lane: AbstractLane,
    obs_list,
    config,
    T,
) -> Trajectory:
    # 检查目标车道的course_spline是否存在
    if target_lane.course_spline is None:
        logging.warning(f"Target lane {target_lane.id} has no course_spline")
        return Trajectory()  # 返回空轨迹
    """
    生成车辆车道变换的轨迹路径
    
    参数:
    - vehicle: 需要变道的车辆对象
    - target_lane: 目标车道
    - obs_list: 障碍物列表
    - config: 配置参数字典
    - T: 时间参数
    
    返回:
    - Trajectory: 最优轨迹路径
    """
    # 获取车辆在目标车道上的状态（位置、速度等信息）
    state_in_target_lane = vehicle.get_state_in_lane(target_lane) # 获取目标车道上的状态
    target_vel = vehicle.target_speed  # 获取车辆的目标速度
    dt = config["DT"]  # 时间步长，用于轨迹离散化
    d_t_sample = config["D_T_S"] / 3.6  # 速度采样间隔，转换为m/s
    n_s_d_sample = config["N_D_S_SAMPLE"]  # 纵向速度采样数量
    s_sample = config["S_SAMPLE"]  # 纵向位置采样间隔（米）
    n_s_sample = config["N_S_SAMPLE"]  # 纵向位置采样数量

    # 设置采样时间范围，使用配置中的最小时间除以1.5作为采样时间
    sample_t = [config["MIN_T"] / 1.5]  # Sample course time
    
    # 设置速度采样范围：在当前速度附近采样，限制不超过目标车道的限速
    sample_vel = np.linspace(
        max(1e-9, state_in_target_lane.vel - d_t_sample * n_s_d_sample),  # 最小速度限制
        min(state_in_target_lane.vel + d_t_sample * n_s_d_sample,  # 最大速度限制
            target_lane.speed_limit), 5) # 采样速度，生成5个速度采样点
    
    # 计算当前速度和目标速度的较小值
    vel = min(state_in_target_lane.vel, target_vel)
    
    # 初始化纵向位置采样数组
    sample_s = np.empty(0)
    
    # 根据采样时间计算纵向位置采样范围
    for t in sample_t:
        sample_s = np.append(
            sample_s,
            np.arange(
                state_in_target_lane.s + t * (max(5.0, vel)),  # 起始位置：当前纵向位置 + 时间 * 最小速度
                state_in_target_lane.s + t *  # 结束位置：当前纵向位置 + 时间 * (目标速度 + 采样范围)
                (target_vel + s_sample * n_s_sample * 1.01),
                s_sample,  # 采样间隔
            ),
        )

    # Step 2: Calculate Paths (步骤2：计算轨迹路径)
    # 初始化最优路径和成本
    best_path = None
    best_cost = math.inf
    
    # 遍历所有采样组合（时间、纵向位置、速度）来寻找最优路径
    for t in sample_t:  # 遍历采样时间
        for s in sample_s:  # 遍历纵向位置采样点
            for s_d in sample_vel:  # 遍历速度采样点
                # 定义目标状态（时间、纵向位置、横向位置为0、纵向速度）
                target_state = State(t=t, s=s, d=0, s_d=s_d)
                
                # 使用Frenet最优规划器计算特定路径
                path = frenet_optimal_planner.calc_spec_path(
                    state_in_target_lane, target_state, target_state.t, dt)
                
                # 如果路径为空，跳过
                if not path.states:
                    continue
                
                # 将路径从Frenet坐标系转换为笛卡尔坐标系
                path.frenet_to_cartesian(target_lane, vehicle.current_state)
                
                # 计算路径的综合成本（多维度评估）
                path.cost = (
                    cost.smoothness(path, target_lane.course_spline,  # 平滑度成本
                                    config["weights"]) * dt +
                    cost.vel_diff(path, target_vel, config["weights"]) * dt +  # 速度差异成本
                    cost.guidance(path, config["weights"]) * dt +  # 引导成本
                    cost.acc(path, config["weights"]) * dt +  # 加速度成本
                    cost.jerk(path, config["weights"]) * dt +  # 加加速度成本
                    cost.obs(vehicle, path, obs_list, config) +  # 障碍物避让成本
                    cost.changelane(config["weights"]))  # 变道成本
                
                # 检查路径是否满足非完整约束（车辆运动学约束）
                if not path.is_nonholonomic():
                    continue
                
                # 如果当前路径成本更低，更新最优路径
                if path.cost < best_cost:
                    best_cost = path.cost
                    best_path = path

    # 如果找到有效路径，返回最优路径
    if best_path is not None:
        logging.debug(f"Vehicle {vehicle.id} found a lane change path with cost: {best_cost}")
        return best_path

    # 如果未找到有效变道路径，生成停车路径作为备选方案
    logging.info(f"vehicle {vehicle.id} found no lane change paths, calculating a stop path instead.")

    # 计算紧急停车路径
    stop_path = frenet_optimal_planner.calc_stop_path(state_in_target_lane,
                                                      vehicle.max_decel,  # 最大减速度
                                                      sample_t[0], dt, config)
    
    # 将停车路径从Frenet坐标系转换为笛卡尔坐标系
    stop_path.frenet_to_cartesian(target_lane, state_in_target_lane)
    
    # 计算停车路径的成本
    stop_path.cost = (cost.smoothness(stop_path, target_lane.course_spline,  # 平滑度成本
                                      config["weights"]) * dt +
                      cost.guidance(stop_path, config["weights"]) * dt +  # 引导成本
                      cost.acc(stop_path, config["weights"]) * dt +  # 加速度成本
                      cost.jerk(stop_path, config["weights"]) * dt +  # 加加速度成本
                      cost.stop(config["weights"]))  # 停车成本
    return stop_path

# 停止轨迹生成器
def stop_trajectory_generator(vehicle: control_Vehicle,
                              lanes: List[AbstractLane],
                              obs_list: List[Obstacle],
                              roadgraph: RoadGraph,
                              config,
                              T,
                              redLight: bool = False,
                              force_stop: bool = False,
                              special_prompt: str = None) -> Trajectory:
    """
    Step 0:展示停车前的相关信息
    0.1 提示信息：某车辆进入停止轨迹生成器
    0.2 打印当前车辆与停车轨迹生成有关的特殊提示信息
    0.3 检查当前车道的course_spline是否存在
    0.4 对于当前车辆，检查其是否为已停止的车辆。
    如果是，则查看其stop_until时间是否到达，如果到达则清除stop_flag并恢复正常行驶
    """
    # 0.1 提示信息：某车辆进入停止轨迹生成器
    logging.info(f"Vehicle {vehicle.id} entering stop_trajectory_generator")
    logging.info(f"Vehicle {vehicle.id} current state: s={vehicle.current_state.s:.2f}, s_d={vehicle.current_state.s_d:.2f}, s_dd={vehicle.current_state.s_dd:.2f}")
    # 0.2 打印当前车辆与停车轨迹生成有关的特殊提示信息
    if special_prompt:
        logging.info(f"Vehicle {vehicle.id} special_prompt: {special_prompt}")
    if force_stop:
        logging.info(f"Vehicle {vehicle.id} force_stop: {force_stop}")
    if redLight:
        logging.info(f"Vehicle {vehicle.id} redLight: {redLight}")
    # 0.3 检查当前车道的course_spline是否存在
    current_lane = lanes[0] # 当前车道
    course_spline = current_lane.course_spline # 当前车道曲线
    if course_spline is None:
        logging.warning(f"Current lane {current_lane.id} has no course_spline")
        return Trajectory()  # 返回空轨迹
    current_state = vehicle.current_state # 当前车辆状态
    course_t = config["MIN_T"]  # 采样时间
    dt = config["DT"] # 时间步长
    max_acc = vehicle.max_accel # 最大加速度
    car_length = vehicle.length # 车辆长度
    
    """
    Step 1: find the right stopping position
    计算车辆停止位置

    current_state.s：车辆当前的纵向位置（即车辆在车道上的当前位置）。
    course_spline.s[-1]：当前车道曲线的终点位置（即车道的总长度）。
    current_state.s_d：车辆当前的纵向速度（即车辆沿车道方向的速度）。
    course_t：采样时间（即规划轨迹的时间范围）。
    car_length：车辆的长度。
    """
    s = np.linspace(
        current_state.s,
        min(
            course_spline.s[-1],
            current_state.s + current_state.s_d * course_t + 3 * car_length,
        ),
        100,
    ) # 一维数组，生成100个等间距的纵向位置
    # 停止位置计算：
    # 情况1：红灯停止
    # 情况2：LetStopBeforeJunction停止（防止进入拥堵交叉口）
    if redLight or (special_prompt == "LetStopBeforeJunction"):
        min_s = s[-1] - 5 # 红灯或防止进入拥堵交叉口时，停在车道终点前5米
    else:
        min_s = s[-1] + 100 # 其他情况，最小停止位置为车道终点后100米
    # 遍历所有障碍物
    for obs in obs_list:
        if obs.type == ObsType.OTHER: # 如果属于其他障碍物
            obs_s, obs_d = course_spline.cartesian_to_frenet1D(
                obs.current_state.x, obs.current_state.y)
            if obs_s == s[0] or obs_s == s[-1]:
                continue
            obs_near_d = max(0, abs(obs_d) - obs.shape.width / 2)
            if obs_near_d < current_lane.width / 2:
                min_s = min(min_s, obs_s - obs.shape.length / 2 - car_length)
        elif obs.type == ObsType.PEDESTRIAN: # 如果属于行人
            obs_s, obs_d = course_spline.cartesian_to_frenet1D(
                obs.current_state.x, obs.current_state.y)
            if obs_s == s[0] or obs_s == s[-1]:
                continue
            obs_near_d = max(0, abs(obs_d) - obs.shape.width / 2)
            if obs_near_d < current_lane.width / 1.5:
                min_s = min(min_s, obs_s - obs.shape.length / 2 - car_length)
        elif obs.type == ObsType.CAR: # 如果属于车辆
            if isinstance(current_lane, JunctionLane):
                # check if in same junction 如果在交叉口
                veh_junction_id = vehicle.lane_id.split("_")[0]
                obs_junction_id = obs.lane_id.split("_")[0]
                nextlane_id = current_lane.next_lane_id
                if veh_junction_id != obs_junction_id and (
                    obs.lane_id != nextlane_id
                    or (
                        obs.lane_id == nextlane_id
                        and obs.current_state.s >= course_spline.s[-1]
                    )
                ):
                    continue
                if (
                    obs.lane_id in (nextlane_id, vehicle.lane_id)
                    and abs(obs.current_state.vel - vehicle.current_state.vel) < 0.5
                ):
                    continue

                for i in range(0, min(len(obs.future_trajectory.states), 20), 3):
                    obs_s, obs_d = course_spline.cartesian_to_frenet1D(
                        obs.future_trajectory.states[i].x,
                        obs.future_trajectory.states[i].y)
                    if obs_s <= s[0] or obs_s >= s[-1]:
                        next_lane = roadgraph.get_lane_by_id(current_lane.next_lane_id)
                        nextlane_spline = next_lane.course_spline
                        obs_s, obs_d = nextlane_spline.cartesian_to_frenet1D(
                            obs.future_trajectory.states[i].x,
                            obs.future_trajectory.states[i].y)
                        obs_s += current_lane.course_spline.s[-1]
                    obs_near_d = max(0, abs(obs_d) - obs.shape.width / 2)
                    if obs_near_d < current_lane.width / 2:
                        min_s = min(min_s, obs_s - obs.shape.length - car_length)
            else:  # in normal lane 如果属于普通车道
                if isinstance(roadgraph.get_lane_by_id(obs.lane_id),NormalLane):
                    edge_1 = current_lane.affiliated_edge
                    edge_2 = roadgraph.get_lane_by_id(obs.lane_id).affiliated_edge
                    if edge_1 != edge_2:
                        continue
                    obs_state_on_current_lane = obs.update_frenet_coord_in_lane(current_lane)
                    obs_s, obs_d = obs_state_on_current_lane.s, obs_state_on_current_lane.d
                    if obs_s <= s[0] or obs_s >= s[-1]:
                        continue
                    obs_near_d = max(0, abs(obs_d) - obs.shape.width / 2)
                    if obs_s > vehicle.current_state.s and obs_near_d < current_lane.width / 2:
                        # 2.0 meter as a constant parking distance
                        min_s = min(
                            min_s, obs_s - obs.shape.length / 2 - car_length / 2 - 2.0
                        )
                # if obs.lane_id == current_lane.id:
                #     obs_s, obs_d = obs.current_state.s, obs.current_state.d
                #     if obs_s <= s[0] or obs_s >= s[-1]:
                #         continue
                #     obs_near_d = max(0, abs(obs_d) - obs.shape.width / 2)
                #     if obs_near_d < current_lane.width / 2:
                #         # 2.0 meter as a constant parking distance
                #         min_s = min(
                #             min_s, obs_s - obs.shape.length / 2 - car_length / 2 - 2.0
                #         )
                else:
                    # next_lane = roadgraph.get_available_next_lane(
                    #     current_lane.id, vehicle.available_lanes
                    # )
                    next_lanes =list(current_lane.next_lanes.values())
                    next_lanes = [l[0] for l in next_lanes]
                    if (
                        next_lanes is not None
                        and obs.lane_id in next_lanes
                        and obs.current_state.s <= obs.shape.length + vehicle.length
                    ):
                        min_s = min(
                            min_s,
                            current_lane.course_spline.s[-1]
                            - obs.shape.length / 2
                            - car_length / 2
                            - 2.0,
                        )
    # 8.4 排除在障碍物的影响下，如果stop_flag=True,说明是主动停车,且到达目标车道
    if vehicle.current_state.stop_flag and vehicle.stop_lane and current_lane.id in vehicle.stop_lane:
        # 最小停车距离计算（基于当前速度）
        # 改进：使用更合理的停车距离计算，考虑安全距离和渐进减速
        current_vel = current_state.s_d  # 当前纵向速度
        stop_pos = vehicle.stop_pos[0] if isinstance(vehicle.stop_pos, (list, tuple)) else vehicle.stop_pos
        # 计算停车所需距离：考虑当前速度、最大减速度和安全距离
        braking_distance = (current_vel ** 2) / (2 * abs(vehicle.max_decel))  # 制动距离
        safety_distance = max(0.5, car_length)  # 安全距离，至少一个车身长度
        # 最小停车距离：停车位置、安全距离、制动距离三者取最大
        min_s = max(stop_pos, safety_distance, braking_distance)
        # 如果距离停车点较近但还未到达min_s，建议提前减速
        remaining_distance = stop_pos - current_state.s
        if remaining_distance > 0 and remaining_distance <= braking_distance * 1.5:
            logging.debug(f"Vehicle {vehicle.id}: Approaching stop position, recommend early deceleration. "
                         f"Remaining: {remaining_distance:.2f}m, Braking distance: {braking_distance:.2f}m")
    """
    Step 2: 
    第二步：根据最小距离和当前速度，判断是否需要紧急停车
    """
    path = Trajectory()
    best_path = None
    # 检查车辆是否处于停车状态（使用纵向速度s_d判断，更准确地反映车辆运动状态）
    is_currently_stopped = (abs(current_state.s_d) <= 0.1 and (min_s - current_state.s) <= car_length)
    # 检查车辆之前是否在停车栈中
    was_previously_stopped = vehicle.id in stopped_vehicles_stack
    # 处理停车状态变化
    # 如果车辆是否处于停车状态，且之前不在停车栈中
    if is_currently_stopped and not was_previously_stopped:
        # 非停车状态->停车状态，进入停车栈
        stopped_vehicles_stack.add(vehicle.id)
        """
        step 2.1: 判断停车位置，发送StopAt消息（仅对刚进栈的车辆）
        """
        # 判断停车位置
        veh_poi_id_raw = vehicle.lane_id
        if "_" in veh_poi_id_raw:
            veh_poi_id = veh_poi_id_raw.split("_")[0]
        else:
            veh_poi_id = veh_poi_id_raw
        ## 移除可能的冒号前缀
        veh_poi_id = veh_poi_id.replace(":", "")
        stop_target_id = vehicle.id # 自己停车，输出语言
        # 自己处于停车的同时，检查前方是否有停车车辆
        for obs in obs_list:
            if obs.type == ObsType.CAR:
                # 检查是否在同一车道且在前方
                obs_s, _ = current_lane.course_spline.cartesian_to_frenet1D(
                    obs.current_state.x, obs.current_state.y)
                if abs(obs_s - current_state.s) < 20 and obs_s > current_state.s:  # 前方20米内
                    stop_target_id = obs._obstacle_id
                    break
        stop_location_id = veh_poi_id
        # 检查是否已经发送过相同的StopAt消息（防止重复发送）
        message_key = (stop_target_id, stop_location_id)
        if message_key not in sent_stopat_messages:
            # 发送停车消息：StopAt(停车对象ID, 停车位置ID) - 仅在首次进入停车状态时发送
            vehicle.communicator.send(f"StopAt({stop_target_id},{stop_location_id});", performative=Performative.Inform)
            # 记录已发送的消息，防止其他车辆重复发送
            sent_stopat_messages[message_key] = vehicle.id
            logging.info(f"Vehicle {vehicle.id} FIRST TIME stopped at junction {stop_location_id} due to target {stop_target_id} - added to stopped stack")
        else:
            # 已经有其他车辆发送了相同的StopAt消息，跳过发送
            original_sender = sent_stopat_messages[message_key]
            logging.info(f"Vehicle {vehicle.id} detected stop event at junction {stop_location_id} for target {stop_target_id}, "
                        f"but StopAt message already sent by vehicle {original_sender} - skipping duplicate")
    # 从停车状态变为其他状态：出栈
    elif not is_currently_stopped and was_previously_stopped:
        stopped_vehicles_stack.discard(vehicle.id)
        # 清理该车辆发送的StopAt消息记录（可选：保持记录一段时间或永久记录）
        # 这里选择保留记录，因为其他车辆可能仍然需要知道该停车事件
        logging.info(f"Vehicle {vehicle.id} resumed from stop state, removed from stopped vehicles stack")
    
    # 处理已经停下的车辆逻辑
    if is_currently_stopped:
        # # 特殊处理：LetStopBeforeJunction状态需要持续维护
        # if special_prompt == "LetStopBeforeJunction" and vehicle.current_state.stop_flag:
        #     logging.info(f"Vehicle {vehicle.id} LetStopBeforeJunction: Maintaining stop state, distance to lane end: {course_spline.s[-1] - current_state.s:.2f}m")
        #     path = Trajectory()
        #     for t in np.arange(0, course_t, dt):
        #         path.states.append(State(t=t, s=current_state.s, d=current_state.d, s_d=0.0))
        #     path.frenet_to_cartesian(lanes, vehicle.current_state)
        #     path.cost = (
        #         cost.smoothness(path, lanes[0].course_spline, config["weights"]) *
        #         dt + cost.guidance(path, config["weights"]) * dt +
        #         cost.acc(path, config["weights"]) * dt +
        #         cost.jerk(path, config["weights"]) * dt)
        #     vehicle.behaviour = Behaviour.STOP
        #     return path
        """
        step 2.2: 检查是否到达stop_until时间，如果未到达则保持静止
        """
        if (vehicle.stop_until is not None and 
            current_state.t < vehicle.stop_until):
            # 如果未到达stop_until时间，保持静止
            logging.debug(f"Vehicle {vehicle.id} Already stopped, keeping still until {vehicle.stop_until}")
            path = Trajectory()
            # 计算e^-10的值
            e_neg_10 = math.exp(-10)
            for t in np.arange(0, course_t, dt):
                # 根据特殊提示决定静止状态的速度和加速度值
                if special_prompt == "LetStopBeforeJunction":
                    # LetStopBeforeJunction时，速度和加速度设为e^-10
                    v = e_neg_10
                    a = e_neg_10
                else:
                    # 其他情况保持为0
                    v = 0.0
                    a = 0.0
                # 生成静止轨迹
                path.states.append(State(
                    t=t, 
                    s=current_state.s, 
                    d=current_state.d, 
                    s_d=v,  # 纵向速度
                    s_dd=a,  # 纵向加速度
                    vel=v,  # 整体速度
                    acc=a,  # 整体加速度
                    stop_flag=True  # 标记为停止状态
                ))
            path.frenet_to_cartesian(lanes, vehicle.current_state)
            path.cost = (
                cost.smoothness(path, lanes[0].course_spline, config["weights"]) *
                dt + cost.guidance(path, config["weights"]) * dt +
                cost.acc(path, config["weights"]) * dt +
                cost.jerk(path, config["weights"]) * dt)
            return path
        else:
            # 其他情况：
            # 情况1：没有stop_until；情况2：时间已经到达stop_until
            logging.debug(f"Vehicle {vehicle.id} Already stopped, stop_until time reached or no stop_until - checking front vehicle status")
            # 检查前方是否有停车车辆
            # 从障碍物列表中提取前方车辆信息
            front_vehicles = {}
            for obs in obs_list:
                if obs.type == ObsType.CAR:
                    # 检查是否在同一车道且在前方
                    obs_s, _ = current_lane.course_spline.cartesian_to_frenet1D(
                        obs.current_state.x, obs.current_state.y)
                    if abs(obs_s - current_state.s) < 50 and obs_s > current_state.s:  # 前方50米内
                        # 创建模拟的control_Vehicle对象用于状态检测
                        front_vehicles[obs._obstacle_id] = type('MockVehicle', (), {
                            'id': obs._obstacle_id,
                            'lane_id': current_lane.id,
                            'current_state': type('MockState', (), {
                                's': obs_s,
                                's_d': obs.current_state.vel  # 使用障碍物状态中的速度
                            })
                        })()
            # 如果有前方车辆，检查其状态
            if front_vehicles:
                from trafficManager.common.vehicle import get_pre_vehicle_status
                front_vehicle_status = get_pre_vehicle_status(vehicle, front_vehicles)
                # 如果前方有停车车辆，保持停止状态，不恢复行驶
                if front_vehicle_status == Behaviour.STOP:
                    logging.info(f"Vehicle {vehicle.id} detected stopped vehicle ahead, keeping stopped despite stop_until time reached")
                    path = Trajectory()
                    # 计算e^-10的值
                    e_neg_10 = math.exp(-10)
                    for t in np.arange(0, course_t, dt):
                        # 根据特殊提示决定静止状态的速度和加速度值
                        if special_prompt == "LetStopBeforeJunction":
                            # LetStopBeforeJunction时，速度和加速度设为e^-10
                            v = e_neg_10
                            a = e_neg_10
                        else:
                            # 其他情况保持为0
                            v = 0.0
                            a = 0.0
                        # 生成静止轨迹
                        path.states.append(State(
                            t=t, 
                            s=current_state.s, 
                            d=current_state.d, 
                            s_d=v,  # 纵向速度
                            s_dd=a,  # 纵向加速度
                            vel=v,  # 整体速度
                            acc=a,  # 整体加速度
                            stop_flag=True  # 标记为停止状态
                        ))
                    path.frenet_to_cartesian(lanes, vehicle.current_state)
                    path.cost = (
                        cost.smoothness(path, lanes[0].course_spline, config["weights"]) *
                        dt + cost.guidance(path, config["weights"]) * dt +
                        cost.acc(path, config["weights"]) * dt +
                        cost.jerk(path, config["weights"]) * dt)
                    vehicle.behaviour = Behaviour.STOP
                    return path
            # stop_until时间到达，且前方无停车车辆，恢复正常行驶，清除stop_flag
            if (vehicle.stop_until is not None and current_state.t >= vehicle.stop_until and vehicle.current_state.stop_flag):
                logging.info(f"Vehicle {vehicle.id} Stop until time {vehicle.stop_until} reached at time {current_state.t}, clearing stop_flag and resuming normal driving")
                vehicle.current_state.stop_flag = False
                vehicle.behaviour = None  # 重置行为状态
                # 生成正常行驶的轨迹，而不是静止轨迹
                target_vel = min(20.0 / 3.6, lanes[0].speed_limit)  # 使用车道限速或默认20km/h
                target_s = current_state.s + target_vel * course_t
                target_state = State(s=target_s, s_d=target_vel, d=0)
                path = frenet_optimal_planner.calc_spec_path(current_state, target_state, course_t, dt)
                path.frenet_to_cartesian(lanes, current_state)
                path.cost = (
                    cost.smoothness(path, lanes[0].course_spline, config["weights"]) *
                    dt + cost.guidance(path, config["weights"]) * dt +
                    cost.acc(path, config["weights"]) * dt +
                    cost.jerk(path, config["weights"]) * dt)
                return path
    
    """
        处理好已经停下的车辆后，根据当前速度和剩余距离动态调整停车阈值
        处理真正可能需要停车的情况，以决定是否进入step 2
    """
    # 特殊处理：当special_prompt为"LetStopBeforeJunction"时，限制停车位置在车道末尾之前
    if special_prompt == "LetStopBeforeJunction":
        distance_to_lane_end = course_spline.s[-1] - current_state.s
        min_s = min(min_s, current_state.s + distance_to_lane_end - 15.0)  # 在车道末尾前15米处停车
        logging.debug(f"Vehicle {vehicle.id} LetStopBeforeJunction: Adjusted min_s to {min_s:.2f}m (distance to lane end: {distance_to_lane_end:.2f}m)")
    
    # 8.19 增加智能停车判断：对主动停车(stop_flag=True)的车辆使用更严格的判断条件
    # 改进：根据当前速度和剩余距离动态调整停车阈值
    remaining_distance = min_s - current_state.s
    current_vel = current_state.s_d
    # 首先定义stop_intent，然后使用它设置基础阈值
    stop_intent = vehicle.current_state.stop_flag and remaining_distance < 50.0
    base_threshold = 0.8 if stop_intent else 1.5
    # 根据速度调整阈值：高速时使用更保守的阈值
    if current_vel > 15.0 / 3.6:  # 速度大于15km/h
        speed_factor = 1.2  # 高速时更保守
    elif current_vel > 8.0 / 3.6:  # 速度8-15km/h
        speed_factor = 1.0  # 正常阈值
    else:  # 低速
        speed_factor = 0.8  # 低速时可以更激进
    stop_threshold = base_threshold * speed_factor
    # 如果剩余距离接近制动距离，提前触发停车逻辑
    braking_distance = (current_vel ** 2) / (2 * abs(vehicle.max_decel))
    if remaining_distance <= braking_distance * 2.0 and remaining_distance > 0:
        stop_threshold *= 0.7  # 需要停车时降低阈值，提前规划停车轨迹
    # 特殊处理：当special_prompt为"LetStopBeforeJunction"时，强制立即停车
    if special_prompt == "LetStopBeforeJunction":
        logging.info(f"Vehicle {vehicle.id} LetStopBeforeJunction enabled - forcing immediate emergency stop")
        
        # 对于LetStopBeforeJunction，无论车辆当前位置如何，都应立即停止
        # 如果车辆已经超过了计算出的停止位置，使用当前位置作为停止位置
        if min_s < current_state.s:
            logging.warning(f"Vehicle {vehicle.id} is already beyond the calculated stop position (current s: {current_state.s:.2f}, min_s: {min_s:.2f}), will stop at current position")
            min_s = current_state.s
        
        # 强制立即停车，使用最大减速度
        path = frenet_optimal_planner.calc_stop_path(current_state,
                                                     vehicle.max_decel * 2.0,  # 使用2倍最大减速度确保立即停止
                                                     course_t, dt, config)
        path.frenet_to_cartesian(lanes, current_state)
        
        # 1. 设置静止时间为5秒
        target_stop_time = 5.0  # 目标静止时间：5秒
        num_states_needed = int(target_stop_time / dt)  # 5秒所需的状态数量
        
        # 2. 设置s_d和s_dd为e^-10
        e_neg_10 = math.exp(-10)  # 计算e^-10的值
        
        # 确保所有轨迹状态都正确停止
        for state in path.states:
            state.stop_flag = True
            state.s_d = e_neg_10  # 设置纵向速度为e^-10
            state.s_dd = e_neg_10  # 设置纵向加速度为e^-10
            # 如果状态的s值超过了min_s，将其限制在min_s，防止车辆继续前进
            if state.s > min_s:
                state.s = min_s
        
        # 3. 确保轨迹保持静止5秒：如果生成的轨迹长度不足5秒，添加额外的静止状态
        if len(path.states) < num_states_needed:
            # 获取最后一个状态作为基础
            last_state = path.states[-1]
            # 添加足够的静止状态，使总长度达到5秒
            for i in range(len(path.states), num_states_needed):
                # 创建新的静止状态，时间递增，位置、速度、加速度保持不变
                new_time = last_state.t + (i - len(path.states) + 1) * dt
                new_state = State(
                    t=new_time,
                    s=last_state.s,
                    d=last_state.d,
                    s_d=e_neg_10,
                    s_dd=e_neg_10,
                    vel=e_neg_10,
                    acc=e_neg_10,
                    stop_flag=True
                )
                path.states.append(new_state)
        
        path.cost = (
            cost.smoothness(path, lanes[0].course_spline, config["weights"]) * dt
            + cost.guidance(path, config["weights"]) * dt
            + cost.acc(path, config["weights"]) * dt
            + cost.jerk(path, config["weights"]) * dt
            + cost.stop(config["weights"])
        )
        
        # 记录强制停车的详细信息
        logging.info(f"Vehicle {vehicle.id} LetStopBeforeJunction: Forced emergency stop completed, "
                    f"Final speed: {path.states[-1].s_d:.2e}m/s, "
                    f"Final acceleration: {path.states[-1].s_dd:.2e}m/s², "
                    f"Stop distance: {min_s - current_state.s:.2f}m, "
                    f"Stop duration: {path.states[-1].t - path.states[0].t:.2f}s")
        return path
    
    # 如果强制停车标志为True，直接跳过"不需要停车"的判断
    if force_stop:
        logging.info(f"Vehicle {vehicle.id} Force stop enabled - bypassing 'no need to stop' logic")
    
    if ((min_s - current_state.s) >
            current_state.s_d * course_t / stop_threshold):  
        """
        no need to stop
        情况2：车辆不需要停车
        """
        logging.debug(f"Vehicle {vehicle.id} No need to stop")
        if (min_s - current_state.s) < 5.0 / 3.6 * course_t:
            target_s = min_s
            target_state = State(s=target_s, s_d=5.0 / 3.6, d=0)
        else:
            # 20 km/h is the speed limit in junction lane
            target_vel = min(20.0 / 3.6, lanes[0].speed_limit)
            target_s = (current_state.s +
                        (current_state.s_d +
                         (target_vel - current_state.s_d) / 1.3) * course_t)
            target_state = State(s=target_s, s_d=target_vel, d=0)
            if target_vel > current_state.s_d:
                current_state.s_dd = max(1e-2,current_state.s_dd)

        path = frenet_optimal_planner.calc_spec_path(current_state,
                                                     target_state, course_t, dt
                                                     )
        # print("no need path", [state.s for state in path.states], [
        #       state.s_d for state in path.states])
        path.frenet_to_cartesian(lanes, current_state)
        path.cost = (
            cost.smoothness(path, lanes[0].course_spline, config["weights"]) *
            dt + cost.guidance(path, config["weights"]) * dt +
            cost.acc(path, config["weights"]) * dt +
            cost.jerk(path, config["weights"]) * dt)
        return path
    elif (min_s - current_state.s) < max(current_state.s_d**2 / (2 * max_acc),
                                         car_length / 4):  
        """
        need emergency stop
        情况3：需要立刻停止
        改进：优化紧急停车判断，考虑渐进减速和更合理的阈值
        """
        # 计算更合理的紧急停车阈值
        current_vel = current_state.s_d
        emergency_threshold = max(
            current_vel**2 / (2 * max_acc),  # 基于当前速度和最大加速度
            car_length / 4,                   # 最小车身相关阈值
            0.5                               # 绝对最小阈值
        )
        # 记录紧急停车原因
        stop_reason = ""
        if redLight:
            stop_reason = "Red Light"
            # 8.18 车辆写入互操作语言：红灯停止
            vehicle.communicator.send(f"Redlight({vehicle.id});",performative=Performative.Inform)
        elif vehicle.current_state.stop_flag: 
            stop_reason = "Active Stop Request"
            # 8.16 车辆写入互操作语言：紧急停止
            logging.debug(f"Vehicle {vehicle.id} Emergency Brake - Reason: {stop_reason}")
            vehicle.communicator.send(f"EmergencyStation({vehicle.id});",performative=Performative.Inform)
        else:
            # 其他情况，记录特殊提示作为原因
            stop_reason = special_prompt
            logging.info(f"Vehicle {vehicle.id} Emergency Stop triggered by - Reason: {stop_reason}")
        
        # 添加调试信息，记录紧急停车的详细情况
        logging.debug(f"Vehicle {vehicle.id} Emergency Stop: "
                     f"Remaining distance: {min_s - current_state.s:.2f}m, "
                     f"Current speed: {current_vel:.2f}m/s, "
                     f"Emergency threshold: {emergency_threshold:.2f}m, "
                     f"Reason: {stop_reason}")
        # 8.4 进入查看
        # 使用最大减速度（1.5倍）确保车辆能够快速停止
        path = frenet_optimal_planner.calc_stop_path(current_state,
                                                     vehicle.max_decel * 1.5,
                                                     course_t, dt, config)
        path.frenet_to_cartesian(lanes, current_state)
        
        # 设置所有轨迹状态的stop_flag为True，确保车辆保持停止状态
        for state in path.states:
            state.stop_flag = True
        
        path.cost = (
            cost.smoothness(path, lanes[0].course_spline, config["weights"]) *
            dt + cost.guidance(path, config["weights"]) * dt +
            cost.acc(path, config["weights"]) * dt +
            cost.jerk(path, config["weights"]) * dt +
            cost.stop(config["weights"]))
        return path
    else:
        """
        normal stop
        情况4：正常停止
        改进：优化正常停车的采样策略，考虑渐进减速和舒适性
        """
        logging.debug(f"Vehicle {vehicle.id} Normal stopping")
        # 计算当前状态信息
        remaining_distance = min_s - current_state.s
        current_vel = current_state.s_d
        # 如果非常接近停车点，保持当前横向位置，否则尝试回到车道中心
        if remaining_distance < car_length:
            sample_d = [current_state.d]  # 保持当前横向位置
        else:
            sample_d = [0]  # 回到车道中心
            # sample_d = np.arange(-road_width / 2, road_width / 2 * 1.01, d_road_w)

        # 改进停车时间采样：基于当前速度和剩余距离计算合理的停车时间范围
        # 考虑舒适性和安全性，避免过快的减速
        if current_vel > 0:
            # 基于舒适减速度计算最小停车时间（避免乘客不适）
            comfort_decel = min(abs(vehicle.max_decel) * 0.5, 2.0)  # 舒适减速度不超过2m/s²
            min_stop_time = max(current_vel / comfort_decel, 0.5)  # 最小停车时间
            
            # 基于当前速度和剩余距离计算最大停车时间
            if remaining_distance > 0:
                max_stop_time = max(remaining_distance / max(current_vel * 0.5, 0.1), min_stop_time * 2)
            else:
                max_stop_time = min_stop_time * 3
            
            # 在合理范围内采样停车时间
            sample_stop_t = np.linspace(min_stop_time, max_stop_time, 4)
        else:
            # 如果当前速度为0，使用默认采样
            sample_stop_t = np.linspace(0.5, 2.0, 4)
        best_path = None
        best_cost = math.inf
        for d in sample_d:
            for stop_t in sample_stop_t:
                target_state = State(s=min_s, s_d=0, d=d)
                path = frenet_optimal_planner.calc_spec_path(
                    current_state, target_state, stop_t, dt)
                t = path.states[-1].t
                s = path.states[-1].s
                d = path.states[-1].d
                while len(path.states) < course_t / dt:
                    t += dt
                    path.states.append(State(t=t, s=s, d=d))

                path.frenet_to_cartesian(lanes, current_state)
                path.cost = (cost.smoothness(path, lanes[0].course_spline,
                                            config["weights"]) * dt +
                            cost.guidance(path, config["weights"]) * dt +
                            cost.jerk(path, config["weights"]) * dt +
                            cost.stop(config["weights"]))
                if path.cost < best_cost:
                    best_cost = path.cost
                    best_path = path
    """
    step 5：如果所有路径都不合适，返回一个默认的紧急停车轨迹
    """
    if best_path is None:
        logging.warning(f"Vehicle {vehicle.id}: No valid stop path found, using emergency stop path")
        stop_path = frenet_optimal_planner.calc_stop_path(current_state,
                                                         vehicle.max_decel * 1.5,
                                                         course_t, dt, config)
        stop_path.frenet_to_cartesian(lanes, current_state)
        # 设置所有轨迹状态的stop_flag为True，确保车辆保持停止状态
        for state in stop_path.states:
            state.stop_flag = True
        stop_path.cost = (
            cost.smoothness(stop_path, lanes[0].course_spline, config["weights"]) * dt
            + cost.guidance(stop_path, config["weights"]) * dt
            + cost.acc(stop_path, config["weights"]) * dt
            + cost.jerk(stop_path, config["weights"]) * dt
            + cost.stop(config["weights"])
        )
        return stop_path
    # 确保best_path不为None，避免UnboundLocalError
    if best_path is None:
        logging.error(f"Vehicle {vehicle.id}: Critical error - best_path is None after all checks")
        # 返回一个安全的默认轨迹
        default_path = Trajectory()
        default_path.states = [current_state]
        return default_path
    
    # 记录最终返回的路径信息
    if best_path and best_path.states:
        final_state = best_path.states[-1]
        logging.info(f"Vehicle {vehicle.id} stop_trajectory_generator completed: "
                    f"Final speed: {final_state.s_d:.2f}m/s, "
                    f"Final position: {final_state.s:.2f}m, "
                    f"Path length: {len(best_path.states)} states")
        # 特殊处理：LetStopBeforeJunction需要持续维护stop_flag
        if special_prompt == "LetStopBeforeJunction":
            # 计算e^-10的值
            e_neg_10 = math.exp(-10)
            # 确保轨迹中所有状态都保持stop_flag=True和s_d、s_dd为e^-10
            for state in best_path.states:
                state.stop_flag = True
                state.s_d = e_neg_10
                state.s_dd = e_neg_10
            logging.info(f"Vehicle {vehicle.id} LetStopBeforeJunction: Enforced stop_flag=True and s_d/s_dd={e_neg_10:.2e} for entire trajectory")
    
    return best_path


def lanekeeping_trajectory_generator(vehicle: control_Vehicle,
                                     lanes: List[AbstractLane], obs_list,
                                     config, T) -> Trajectory:
    # 检查当前车道的course_spline是否存在
    if lanes[0].course_spline is None:
        logging.warning(f"Lane {lanes[0].id} has no course_spline")
        return Trajectory()  # 返回空轨迹
        
    road_width = lanes[0].width # 车道宽度
    current_state = vehicle.current_state # 当前状态
    target_vel = vehicle.target_speed # 目标速度

    # Step 1: Sample target states
    d_road_w = config["D_ROAD_W"]
    d_t_sample = config["D_T_S"] / 3.6
    n_s_d_sample = config["N_D_S_SAMPLE"]
    dt = config["DT"]

    sample_d = np.linspace(-road_width / 3,
                           road_width / 3,
                           num=int(road_width / d_road_w) +
                           1)  # sample target lateral offset
    sample_d = sample_d[sample_d != 0]
    center_d = [0]
    sample_t = [config["MIN_T"]]  # Sample course time
    # sample target longtitude vel(Velocity keeping)
    # decelerate when traveling into next road segement
    if current_state.vel * sample_t[0] > lanes[0].course_spline.s[
            -1] - current_state.s:
        speed_limit = 25 / 3.6
        if len(lanes) > 1:
            speed_limit = min(speed_limit, lanes[1].speed_limit)
        sample_vel = np.linspace(min(current_state.vel, 10 / 3.6), speed_limit, 4)
    else:
        sample_vel = np.linspace(
            max(1e-9, current_state.vel - d_t_sample * n_s_d_sample),
            min(
                max(current_state.vel, target_vel) +
                d_t_sample * n_s_d_sample * 1.01, lanes[0].speed_limit),
            5,
        )

    # Step 2: Generate Center line trajectories
    center_paths = frenet_optimal_planner.calc_frenet_paths(
        current_state, center_d, sample_t, sample_vel, dt, config)
    best_path = None
    best_cost = math.inf
    if center_paths is not None:
        for path in center_paths:
            path.frenet_to_cartesian(lanes, current_state)
            path.cost = (
                cost.smoothness(path, lanes[0].course_spline, config["weights"])
                * dt + cost.vel_diff(path, target_vel, config["weights"]) * dt +
                cost.guidance(path, config["weights"]) * dt +
                cost.acc(path, config["weights"]) * dt +
                cost.jerk(path, config["weights"]) * dt +
                cost.obs(vehicle, path, obs_list, config))
            if check_path(vehicle, path) and path.cost < best_cost:
                best_cost = path.cost
                best_path = path

    if best_path is not None:
        return best_path
    
    # Step 3: If no valid path, Generate nudge trajectories
    paths = frenet_optimal_planner.calc_frenet_paths(current_state, sample_d,
                                                     sample_t, sample_vel, dt,
                                                     config)
    best_cost = math.inf
    if paths is not None:
        for path in paths:
            path.frenet_to_cartesian(lanes, current_state)
            path.cost = (
                cost.smoothness(path, lanes[0].course_spline, config["weights"])
                * dt + cost.vel_diff(path, target_vel, config["weights"]) * dt +
                cost.guidance(path, config["weights"]) * dt +
                cost.acc(path, config["weights"]) * dt +
                cost.jerk(path, config["weights"]) * dt +
                cost.obs(vehicle, path, obs_list, config))
            if check_path(vehicle, path) and path.cost < best_cost:
                best_cost = path.cost
                best_path = path

    if best_path is not None:
        logging.debug(
            f"Vehicle {vehicle.id} finds a lanekeeping NUDGE path with minimum cost: {best_cost}"
        )
        return best_path


def decision_trajectory_generator(
    vehicle: control_Vehicle,
    lanes: List[AbstractLane],
    obs_list,
    config,
    T,
    decision_list: List[SingleStepDecision],
) -> Trajectory:
    d_road_w = config["D_ROAD_W"]
    d_vel = config["D_T_S"] / 3.6
    dt = config["DT"]

    fullpath = Trajectory()
    current_time = T
    current_state = vehicle.current_state
    has_lane_change = False
    for (idx, decision) in enumerate(decision_list):
        if T >= decision.expected_time:
            # decision time is in the past
            continue
        if (
            idx + 1 < len(decision_list)
            and decision_list[idx + 1].action == decision.action
        ):
            # skip for the same action
            continue
        # if lane change
        if decision.action == "LCL" or decision.action == "LCR" or has_lane_change:
            if decision.expected_state.s > lanes[0].spline_length:
                # vehicle is out of the lane
                break
            decision.expected_state.s, decision.expected_state.d = lanes[
                0
            ].course_spline.cartesian_to_frenet1D(
                decision.expected_state.x,
                decision.expected_state.y)
            has_lane_change = True

        seg_time = decision.expected_time - current_time
        sample_d = np.linspace(
            decision.expected_state.d - d_road_w,
            decision.expected_state.d + d_road_w,
            5,
        )
        sample_vel = np.linspace(
            max(1e-9, decision.expected_state.vel - 3* d_vel),
            min(decision.expected_state.vel + d_vel, lanes[0].speed_limit),
            10,
        )

        seg_paths = frenet_optimal_planner.calc_frenet_paths(
            current_state, sample_d, [seg_time], sample_vel, dt, config
        )  
        offset_frame = len(fullpath.states)
        best_path = None
        best_cost = math.inf
        for path in seg_paths:
            path.frenet_to_cartesian(lanes, current_state)
            path.cost = (
                cost.smoothness(path, lanes[0].course_spline, config["weights"]) * dt
                + cost.vel_diff(path, vehicle.target_speed, config["weights"]) * dt
                + cost.guidance(path, config["weights"]) * dt
                + cost.acc(path, config["weights"]) * dt
                + cost.jerk(path, config["weights"]) * dt
                + cost.obs(vehicle, path, obs_list, config, offset_frame)
            )
            if check_path(vehicle, path) and path.cost < best_cost:
                best_cost = path.cost
                best_path = path

        if best_path is not None:
            current_state = best_path.states[-1]
            fullpath.concatenate(best_path)
            current_time = decision.expected_time
        else:
            fullpath = None
            logging.info(
                "cannot generate path for id %s at traj_idx %d, decision %s",
                vehicle.id, idx, decision.action,
            )
            break
        if (
            current_time - T > config["MIN_T"]
            or current_time - T > fullpath.states[-1].t
        ):
            # finish planning
            break

    if fullpath is not None and len(fullpath.states) > 0:
        return fullpath
    else:  # no valid path found
        return None


def cleanup_stopped_vehicle(vehicle_id: str):
    """
    从停车栈中移除指定车辆
    用于清理不再存在的车辆
    """
    global stopped_vehicles_stack
    if vehicle_id in stopped_vehicles_stack:
        stopped_vehicles_stack.discard(vehicle_id)
        logging.info(f"Vehicle {vehicle_id} removed from stopped vehicles stack during cleanup")

def get_stopped_vehicles():
    """
    获取当前在停车栈中的所有车辆ID
    """
    return list(stopped_vehicles_stack)

if __name__ == "__main__":
    pass
