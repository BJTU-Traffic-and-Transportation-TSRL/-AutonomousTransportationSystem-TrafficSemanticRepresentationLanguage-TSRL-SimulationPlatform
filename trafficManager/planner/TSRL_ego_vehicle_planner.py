import time
import traci

from common.observation import Observation
from common.vehicle import Behaviour, control_Vehicle
from decision_maker.abstract_decision_maker import EgoDecision, MultiDecision
from trafficManager.planner.abstract_planner import AbstractEgoPlanner
from predictor.abstract_predictor import Prediction

import logger
import trafficManager.planner.trajectory_generator as traj_generator
from TSRL_interaction.vehicle_communication import Performative
from utils.obstacles import DynamicObstacle, ObsType, Rectangle
from utils.roadgraph import JunctionLane, NormalLane, RoadGraph
from utils.trajectory import State, Trajectory

logging = logger.get_logger(__name__)
"""
Ego自车轨迹规划(TSRL)
"""
class EgoPlanner(AbstractEgoPlanner):
    def is_in_intersection(
            self, current_lane, next_lane, vehicle_state
        ) -> bool:
        """check if vehicle is in intersection

        Args:
            current_lane: the lane vehicle lies on
            next_lane: the next lane
            vehicle_state: current state of the vehicle

        Returns:
            bool: True is vehicle is in intersection.
        """
        # 如果当前车道就是交叉口车道，直接返回True
        if isinstance(current_lane, JunctionLane):
            return True
        # 如果当前车道是正常车道，但下一个车道是交叉口车道，且车辆接近交叉口入口，返回True
        if isinstance(current_lane, NormalLane) and next_lane is not None and isinstance(next_lane, JunctionLane):
            # 检查车辆是否已经进入交叉口区域
            distance_to_junction = current_lane.course_spline.s[-1] - vehicle_state.s
            if distance_to_junction < 10.0:  # 距离交叉口入口小于10米认为已进入
                return True
        return False
    
    def is_approaching_intersection(
            self, current_lane, next_lane, vehicle_state
        ) -> bool:
        """check if vehicle is approaching intersection

        Args:
            current_lane: the lane vehicle lies on
            next_lane: the next lane
            vehicle_state: current state of the vehicle

        Returns:
            bool: True if vehicle is approaching intersection.
        """
        # 如果当前车道是正常车道，下一个车道是交叉口车道，且车辆还在道路上，返回True
        if isinstance(current_lane, NormalLane) and next_lane is not None and isinstance(next_lane, JunctionLane):
            # 检查车辆是否还在道路上接近交叉口
            distance_to_junction = current_lane.course_spline.s[-1] - vehicle_state.s
            if distance_to_junction >= 5.0:  # 距离交叉口入口大于等于5米认为正在接近
                return True
        return False

    def is_waiting_for_green_light(
            self, current_lane, next_lane, vehicle_state
        ) -> bool:
        """check if vehicle is waiting for green light at intersection

        Args:
            current_lane: the lane vehicle lies on
            next_lane: the next lane
            vehicle_state: current state of the vehicle

        Returns:
            bool: True is vehicle is in intersection and waiting for green light.
        """
        if not self.is_in_intersection(current_lane, next_lane, vehicle_state):
            return False
        # 检查next_lane是否为JunctionLane类型，只有JunctionLane才有currTlState属性
        if isinstance(next_lane, JunctionLane):
            return next_lane.currTlState in ("R", "r")
        # 如果next_lane不是JunctionLane，检查current_lane是否为JunctionLane
        elif isinstance(current_lane, JunctionLane):
            return current_lane.currTlState in ("R", "r")
        # 其他情况，默认返回False
        return False

    def plan(self,
             ego_veh: control_Vehicle, # 当前车辆
             observation: Observation, # 观测
             roadgraph: RoadGraph, # 道路图
             prediction: Prediction, # 预测
             T, # 时间
             config, # 配置
             ego_decision: EgoDecision = None) -> Trajectory: # 决策
        """
        规划步骤：
        1. 获取相关参数，处理障碍物
        2. 预测当前车辆
        """
        """
        1. 获取相关参数，处理障碍物
        """
        vehicle_id = ego_veh.id # 车辆ID
        start = time.time() # 开始时间
        current_lane = roadgraph.get_lane_by_id(ego_veh.lane_id) # 当前车道
        obs_list = [] # 障碍物列表
        # Process static obstacle
        for obs in observation.obstacles:
            obs_list.append(obs)
        # Process dynamic_obstacles
        # 处理动态障碍物
        for predict_veh, prediction in prediction.results.items():
            if predict_veh.id == vehicle_id:
                continue
            # 检查prediction列表是否为空，避免索引越界
            if not prediction or len(prediction) == 0:
                logging.warning(f"Vehicle {vehicle_id}: Empty prediction for vehicle {predict_veh.id}, skipping")
                continue
            shape = Rectangle(predict_veh.length, predict_veh.width)
            current_state = State(x=prediction[0].x,
                                  y=prediction[0].y,
                                  s=prediction[0].s,
                                  d=prediction[0].d,
                                  yaw=prediction[0].yaw,
                                  vel=prediction[0].vel)
            dynamic_obs = DynamicObstacle(obstacle_id=predict_veh.id,
                                          shape=shape,
                                          obstacle_type=ObsType.CAR,
                                          current_state=current_state,
                                          lane_id=predict_veh.lane_id)
            for i in range(1, len(prediction)):
                state = State(x=prediction[i].x,
                              y=prediction[i].y,
                              s=prediction[i].s,
                              d=prediction[i].d,
                              yaw=prediction[i].yaw,
                              vel=prediction[i].vel)
                dynamic_obs.future_trajectory.states.append(state)
            obs_list.append(dynamic_obs)
        """
        Predict for current vehicle
        2. 预测当前车辆
        """
        # 获取车辆可用车道（当前车道下一个车道）
        next_lane = roadgraph.get_available_next_lane(
            current_lane.id, ego_veh.available_lanes)
        lanes = [current_lane, next_lane] if next_lane != None else [
            current_lane]
        # 确定车辆行为：优先使用决策中的行为，如果没有则使用车辆当前行为
        vehicle_behaviour = ego_veh.behaviour
        if ego_decision and ego_decision.result:
            decision_list = ego_decision.result
            if decision_list and decision_list[0].behaviour is not None:
                vehicle_behaviour = decision_list[0].behaviour
                logging.info(f"Using behaviour from decision: {vehicle_behaviour}")
        
        # 处理无效的车辆行为值，确保使用有效的Behaviour枚举值
        if vehicle_behaviour is None or not isinstance(vehicle_behaviour, Behaviour):
            # 如果行为无效，使用默认的保持车道行为
            logging.warning(f"Vehicle {ego_veh.id} has invalid behaviour: {vehicle_behaviour}, defaulting to KL")
            vehicle_behaviour = Behaviour.KL
        # 保存前一个行为状态，用于检测行为变化
        if not hasattr(ego_veh, 'previous_behaviour'):
            ego_veh.previous_behaviour = vehicle_behaviour
        previous_behaviour = ego_veh.previous_behaviour
        """
        根据车辆行为，规划轨迹
        """
        # 初始化path变量，确保在所有情况下都有默认值
        path = None
        # 1.车辆行为：保持车道
        if vehicle_behaviour == Behaviour.KL:
            need_stop = False
            # 检查是否在等待红灯
            if self.is_waiting_for_green_light(current_lane, next_lane, ego_veh.current_state):
                need_stop = True
                logging.debug(f"Vehicle {ego_veh.id} is waiting for green light, will stop")
            else:
                # 检查前方是否有停止的车辆（在所有速度下都进行检测）
                has_stopped_vehicle_ahead = False
                for obs in obs_list:
                    if obs.type == ObsType.CAR:
                        # 检查是否在同一车道且在前方
                        if hasattr(obs, 'current_state') and hasattr(obs.current_state, 'vel'):
                            if obs.current_state.vel < 0.1:  # 前方车辆速度小于0.1m/s认为已停止
                                # 检查是否在同一车道或相邻车道
                                obs_s, obs_d = current_lane.course_spline.cartesian_to_frenet1D(
                                    obs.current_state.x,
                                    obs.current_state.y
                                )
                                if (obs_s > ego_veh.current_state.s and 
                                    abs(obs_d) < current_lane.width * 1.5):  # 考虑相邻车道的情况
                                    has_stopped_vehicle_ahead = True
                                    logging.info(f"Vehicle {ego_veh.id} detected stopped vehicle {obs._obstacle_id} ahead at distance {obs_s - ego_veh.current_state.s:.2f}m, will stop")
                                    break
                need_stop = has_stopped_vehicle_ahead
            
            if need_stop:
                # 停止轨迹生成
                path = traj_generator.stop_trajectory_generator(
                    ego_veh, lanes, obs_list, roadgraph, config, T, force_stop=True
                )
            else:
                # 正常行驶轨迹生成
                path = traj_generator.lanekeeping_trajectory_generator(
                    ego_veh, lanes, obs_list, config, T,
                )
        # 2. 车辆行为：停止
        elif vehicle_behaviour == Behaviour.STOP:
            # Stopping
            # 仅当行为从其他状态变为STOP时才发送LetStop消息
            if (previous_behaviour != vehicle_behaviour and 
                ego_veh.if_traffic_communication and 
                ego_veh.communicator):
                ego_veh.communicator.send(f"LetStop({ego_veh.id});", performative=Performative.Inform)
                logging.info(f"Vehicle {ego_veh.id} sent LetStop message when behaviour changed from {previous_behaviour} to STOP")
            path = traj_generator.stop_trajectory_generator(
                ego_veh, lanes, obs_list, roadgraph, config, T,
            )
        # 如果当前车辆行为是变道左
        elif vehicle_behaviour == Behaviour.LCL:
            # 8.27 新增：发送变道互操作语言
            ego_veh.communicator.send(f"LeftChangeLane({ego_veh.id});",performative=Performative.Inform) 
            # Turn Left
            # 只对NormalLane类型的车道调用left_lane方法
            left_lane_id = current_lane.left_lane() if isinstance(current_lane, NormalLane) else None
            left_lane = roadgraph.get_lane_by_id(left_lane_id) if left_lane_id is not None else None
            # 检查left_lane是否为None，防止后续访问course_spline时出错
            if left_lane is None:
                logging.warning(f"Cannot find left lane for vehicle {ego_veh.id}, keeping current lane instead")
                path = traj_generator.lanekeeping_trajectory_generator(
                    ego_veh, lanes, obs_list, config, T,
                )
            else:
                path = traj_generator.lanechange_trajectory_generator(
                    ego_veh,
                    left_lane,
                    obs_list,
                    config,
                    T,
                )
                # 10.20 确认车辆是否已经在目标车道上
                if traci.vehicle.getLaneID(ego_veh.id) == left_lane.id:
                    # 车辆已在目标车道，发送完成变道消息并保持车道
                    ego_veh.communicator.send(f"LeftChangeLaneComplete({ego_veh.id});", performative=Performative.Inform)
                    ego_veh.behaviour = Behaviour.KL
                    # path = traj_generator.lanekeeping_trajectory_generator(
                    #     ego_veh, lanes, obs_list, config, T,
                    # )
        # 如果当前车辆行为是变道右
        elif vehicle_behaviour == Behaviour.LCR:
            # 8.27 新增：发送变道互操作语言
            ego_veh.communicator.send(f"RightChangeLane({ego_veh.id});",performative=Performative.Inform) 
            # Turn Right
            # 只对NormalLane类型的车道调用right_lane方法
            right_lane_id = current_lane.right_lane() if isinstance(current_lane, NormalLane) else None
            right_lane = roadgraph.get_lane_by_id(right_lane_id) if right_lane_id is not None else None
            # 检查right_lane是否为None，防止后续访问course_spline时出错
            if right_lane is None:
                logging.warning(f"Cannot find right lane for vehicle {ego_veh.id}, keeping current lane instead")
                path = traj_generator.lanekeeping_trajectory_generator(
                    ego_veh, lanes, obs_list, config, T,
                )
            else:
                path = traj_generator.lanechange_trajectory_generator(
                    ego_veh,
                    right_lane,
                    obs_list,
                    config,
                    T,
                )
        # 如果当前车辆行为是进入junction
        elif vehicle_behaviour == Behaviour.IN_JUNCTION:
            # in Junction. for now just stop trajectory
            path = traj_generator.stop_trajectory_generator(
                ego_veh, lanes, obs_list, roadgraph, config, T,
            )
        # 如果当前车辆行为是发送
        elif vehicle_behaviour == Behaviour.SEND:
            # 发送互操作语言
            ego_veh.communicator.send(f"{ego_decision.result[-1].action};", performative=Performative.Inform)
            # Default to keep lane behavior if sending message
            path = traj_generator.lanekeeping_trajectory_generator(
            ego_veh, lanes, obs_list, config, T,
            )
        elif vehicle_behaviour == Behaviour.SPECIAL_HANDLING:
            if ego_decision and ego_decision.result and len(ego_decision.result) > 0:
                ego_veh.communicator.send(f"{ego_decision.result[-1].action};", performative=Performative.Inform)
                action = ego_decision.result[-1].action
                if "LetStopBeforeJunction" in action:
                    # 特殊处理行为，根据具体情况实现
                    # 检查车辆当前位置：道路上即将进入交叉口 或 已在交叉口内
                    is_in_junction = self.is_in_intersection(current_lane, next_lane, ego_veh.current_state)
                    is_approaching = self.is_approaching_intersection(current_lane, next_lane, ego_veh.current_state)
                    
                    # 无论车辆位置如何，只要收到LetStopBeforeJunction决策，就执行停车行为
                    logging.info(f"Vehicle {ego_veh.id} received LetStopBeforeJunction decision, will stop regardless of traffic light")
                    
                    if is_in_junction:
                        # 情况1：车辆已在交叉口内，立即停止
                        logging.info(f"Vehicle {ego_veh.id} is in intersection {next_lane.id if next_lane else current_lane.id}, will stop immediately")
                        path = traj_generator.stop_trajectory_generator(
                            ego_veh, lanes, obs_list, roadgraph, config, T, force_stop=True, special_prompt="LetStopBeforeJunction"
                        )
                    elif is_approaching:
                        # 情况2：车辆在道路上，即将进入拥堵交叉口
                        logging.info(f"Vehicle {ego_veh.id} is approaching congested intersection, will stop 5m before lane end")
                        # 调用stop_trajectory_generator，传递special_prompt="LetStopBeforeJunction"，仿照红灯停止逻辑
                        path = traj_generator.stop_trajectory_generator(
                            ego_veh, lanes, obs_list, roadgraph, config, T, special_prompt="LetStopBeforeJunction", redLight=False, force_stop=True
                        )
                    else:
                        # 其他情况，默认停止
                        logging.info(f"Vehicle {ego_veh.id} LetStopBeforeJunction: Default stop behavior")
                        path = traj_generator.stop_trajectory_generator(
                            ego_veh, lanes, obs_list, roadgraph, config, T, force_stop=True, special_prompt="LetStopBeforeJunction"
                        )
                else:
                    # 其他特殊处理行为，默认使用停车轨迹
                    logging.warning(f"Vehicle {ego_veh.id}: Unknown special handling action '{action}', defaulting to stop trajectory")
                    path = traj_generator.stop_trajectory_generator(
                        ego_veh, lanes, obs_list, roadgraph, config, T
                    )
            else:
                # 没有有效的决策结果，默认使用停车轨迹
                logging.warning(f"Vehicle {ego_veh.id}: No valid decision result for special handling, defaulting to stop trajectory")
                path = traj_generator.stop_trajectory_generator(
                    ego_veh, lanes, obs_list, roadgraph, config, T
                )
        else:
            # 未知行为，记录日志
            logging.info(
                "Vehicle {} has unknown behaviour {}".format(
                    ego_veh.id, vehicle_behaviour)
            )
            # 10.24 根据条件自检，确认下一步行为
            if ego_decision.result[-1].action.startswith("Check"):
                ego_veh.communicator.send(f"{ego_decision.result[-1].action};", performative=Performative.Inform)
                ego_veh.selfcheck(ego_decision.result[-1].action,roadgraph)
            # Default to keep lane behavior if unknown
            path = traj_generator.lanekeeping_trajectory_generator(
            ego_veh, lanes, obs_list, config, T,
            )
        logging.debug(
            "Vehicle {} Total planning time: {}".format(
                ego_veh.id, time.time() - start)
        )

        # 更新前一个行为状态，用于下次检测行为变化
        ego_veh.previous_behaviour = vehicle_behaviour
        
        # 最终安全检查，确保path变量已被正确赋值
        if path is None:
            logging.error(f"Vehicle {ego_veh.id}: Path generation failed for behaviour {vehicle_behaviour}, defaulting to emergency stop")
            path = traj_generator.stop_trajectory_generator(
                ego_veh, lanes, obs_list, roadgraph, config, T, force_stop=True
            )
        
        return path