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
Ego自车轨迹规划(MCTS)
"""


class EgoPlanner(AbstractEgoPlanner):
    def plan(self,
             ego_veh: control_Vehicle, # 当前车辆
             observation: Observation, # 观测
             roadgraph: RoadGraph, # 道路图
             prediction: Prediction, # 预测
             T, # 时间
             config, # 配置
             ego_decision: EgoDecision = None) -> Trajectory: # 决策

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
        预测当前车辆
        """
        next_lane = roadgraph.get_available_next_lane(
            current_lane.id, ego_veh.available_lanes)
        lanes = [current_lane, next_lane] if next_lane != None else [
            current_lane]

        # 9.19 确定车辆行为：优先使用决策中的行为，如果没有则使用车辆当前行为
        vehicle_behaviour = ego_veh.behaviour
        if ego_decision and ego_decision.result:
            decision_list = ego_decision.result
            if decision_list and decision_list[0].behaviour is not None:
                vehicle_behaviour = decision_list[0].behaviour
                logging.info(f"Using behaviour from decision: {vehicle_behaviour}")

        # 如果当前车辆行为是保持车道
        if vehicle_behaviour == Behaviour.KL:
            if isinstance(current_lane, NormalLane) and next_lane != None and isinstance(next_lane, JunctionLane) and (next_lane.currTlState == "R" or next_lane.currTlState == "r"):
                # Stop
                path = traj_generator.stop_trajectory_generator(
                    ego_veh, lanes, obs_list, roadgraph, config, T, redLight=True
                )
            else:
                # Keep Lane
                if ego_veh.current_state.s_d >= 10 / 3.6:
                    path = traj_generator.lanekeeping_trajectory_generator(
                        ego_veh, lanes, obs_list, config, T,
                    )
                else:
                    path = traj_generator.stop_trajectory_generator(
                        ego_veh, lanes, obs_list, roadgraph, config, T,
                    )
        # 如果当前车辆行为是停止
        elif vehicle_behaviour == Behaviour.STOP:
            # Stopping
            path = traj_generator.stop_trajectory_generator(
                ego_veh, lanes, obs_list, roadgraph, config, T,
            )
        elif vehicle_behaviour == Behaviour.LCL:
            # 8.27 新增：发送变道互操作语言
            ego_veh.communicator.send(f"LeftChangeLane({ego_veh.id});",performative=Performative.Inform) 
            # Turn Left
            left_lane = roadgraph.get_lane_by_id(current_lane.left_lane())
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
        elif vehicle_behaviour == Behaviour.LCR:
            # 8.27 新增：发送变道互操作语言
            ego_veh.communicator.send(f"RightChangeLane({ego_veh.id});",performative=Performative.Inform) 
            # Turn Right
            right_lane = roadgraph.get_lane_by_id(
                current_lane.right_lane())
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
        elif vehicle_behaviour == Behaviour.IN_JUNCTION:
            # in Junction. for now just stop trajectory
            path = traj_generator.stop_trajectory_generator(
                ego_veh, lanes, obs_list, roadgraph, config, T,
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

        return path