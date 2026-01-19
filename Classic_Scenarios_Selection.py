"""
经典场景选择程序:
合并了四个经典交通场景：前向碰撞警告、人车交互、车与RSU交互、车车交互
支持通过命令行参数选择运行不同的场景
"""
from simModel.egoTracking.model import Model
from trafficManager.traffic_manager import TrafficManager
from trafficManager.common.vehicle import Behaviour
from simModel.common.carFactory import Vehicle  # 导入正确的Vehicle类
from traci import TraCIException
import traci.constants as tc

import traci
import json
import logger
import sys
import os
import time
import argparse

# 定义项目根目录
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

# 导入地址配置文件
from utils.load_config import load_config
loc_config = load_config("loc_config.yaml")

# 将日志文件保存到DEBUG_TSRL目录
log_dir = loc_config["LOC_DEBUG"]
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
log_file_path = os.path.join(log_dir, "app_debug_Classic_Scenarios.log")
log = logger.setup_app_level_logger(file_name=log_file_path)

# 场景名称常量
SCENARIO_FORWARD_COLLISION = "Forward_Collision_Warning"
SCENARIO_HUMAN_VEHICLE = "Human_Vehicle_Interacting"
SCENARIO_VEHICLE_RSU = "Vehicle_RSU_Interacting"
SCENARIO_VEHICLE_VEHICLE = "Vehicle_Vehicle_Interacting"
COLLABORATION_SCENARIO = "Vehicle_Vehicle_Collaboration"

# 各场景对应的主车ID
SCENARIO_EGO_IDS = {
    SCENARIO_FORWARD_COLLISION: "1",  # 前向碰撞警告场景主车ID
    SCENARIO_HUMAN_VEHICLE: "0",   # 人车交互场景主车ID
    SCENARIO_VEHICLE_RSU: "0",     # 车与RSU交互场景主车ID
    SCENARIO_VEHICLE_VEHICLE: "0",    # 车车交互场景主车ID
    COLLABORATION_SCENARIO: "HV"    # 车车合作场景主车ID
}
# 合并五个场景的路网文件路径
file_paths = {
    # 前向碰撞警告场景
    SCENARIO_FORWARD_COLLISION: (
        "networkFiles/Forward_Collision_Warning/Forward_Collision_Warning.net.xml",
        "networkFiles/Forward_Collision_Warning/Forward_Collision_Warning.rou.xml"
    ),
    # 人车交互场景
    SCENARIO_HUMAN_VEHICLE: (
        "networkFiles/Human_Vehicle_Interacting/Human_Vehicle_Interacting.net.xml",
        "networkFiles/Human_Vehicle_Interacting/Human_Vehicle_Interacting.rou.xml"
    ),
    # 车与RSU交互场景
    SCENARIO_VEHICLE_RSU: (
        "networkFiles/Vehicle_RSU_Interacting/Vehicle_RSU_Interacting.net.xml",
        "networkFiles/Vehicle_RSU_Interacting/Vehicle_RSU_Interacting.rou.xml",
        "networkFiles/Vehicle_RSU_Interacting/Vehicle_RSU_Interacting.add.xml"
    ),
    # 车车交互场景
    SCENARIO_VEHICLE_VEHICLE: (
        "networkFiles/Vehicle_Vehicle_Interacting/Vehicle_Vehicle_Interacting.net.xml",
        "networkFiles/Vehicle_Vehicle_Interacting/Vehicle_Vehicle_Interacting.rou.xml"
    ),
    # 车车合作场景
    COLLABORATION_SCENARIO: (
        "networkFiles/Vehicle_Vehicle_Collaboration/Vehicle_Vehicle_Collaboration.net.xml",
        "networkFiles/Vehicle_Vehicle_Collaboration/Vehicle_Vehicle_Collaboration.rou.xml",
        "networkFiles/Vehicle_Vehicle_Collaboration/Vehicle_Vehicle_Collaboration.add.xml"
    )
}

def run_model(
    scenario_name,
    net_file,
    rou_file,
    add_file,
    ego_veh_id,
    data_base=None,
    SUMOGUI=loc_config["SUMO_GUI"],
    sim_note=None,
    carla_cosim=False,
    max_sim_time=300,  # 单位秒
    communication=True,  # 全局通信管理器
    if_clear_message_file=True  # 是否清理消息文件本体
):
    """运行指定场景的模拟"""
    # 设置默认参数
    if data_base is None:
        data_base = f"{scenario_name}.db"
    if sim_note is None:
        sim_note = f"{scenario_name} simulation, ATSISP-v-1.0."
    
    try:
        # 加载配置文件
        from utils.load_config import load_config
        config = load_config(loc_config["LOC_CONFIG"])
        log.info(f"Starting {scenario_name} simulation")
        
        model = Model(
            ego_veh_id,
            net_file,
            rou_file,
            addFile=add_file,
            dataBase=data_base,
            SUMOGUI=SUMOGUI,
            simNote=sim_note,
            carla_cosim=carla_cosim,
            max_steps=int(max_sim_time * 10), # 将max_sim_time转换为步长
            communication=communication, # 全局通信管理器
            Scenario_Name=scenario_name, # 场景名称
            config=config  # 传递配置信息
        )
        model.start() # 初始化
        planner = TrafficManager(model) # 初始化车辆规划模块
        # 清理消息文件or清理消息内容：
        model.clear_message_files(planner, if_clear_message_file)
        # 主循环
        # 当自车未到达终点时，继续模拟
        while not model.tpEnd:
            try:
                # 检查是否需要暂停运行
                pause_signal_file = os.path.join(PROJECT_ROOT, "trafficManager", "planner", "pause_resume_signal", f"pause_{scenario_name}.signal")
                # 检查是否需要恢复运行
                resume_signal_file = os.path.join(PROJECT_ROOT, "trafficManager", "planner", "pause_resume_signal", f"resume_{scenario_name}.signal")
                # 如果检测到暂停信号，进入暂停状态
                while os.path.exists(pause_signal_file):
                    log.info(f"Simulation {scenario_name} paused at step {model.timeStep}")
                    time.sleep(0.1)  # 每秒检查一次恢复信号
                    # 如果检测到恢复信号，退出暂停状态
                    if os.path.exists(resume_signal_file):
                        log.info(f"Simulation {scenario_name} resumed at step {model.timeStep}")
                        # 删除恢复信号文件
                        os.remove(resume_signal_file)
                        # 删除暂停信号文件
                        if os.path.exists(pause_signal_file):
                            os.remove(pause_signal_file)
                        break

                # 检查是否存在人工指令文件
                instruction_file = os.path.join(PROJECT_ROOT, "trafficManager", "planner", "input_signal", f"input_{scenario_name}.signal")
                if os.path.exists(instruction_file):
                    with open(instruction_file, 'r', encoding='utf-8') as f:
                        instruction = f.read().strip()
                    log.info(f"Received manual instruction: {instruction}")
                    planner._handle_user_input(instruction)
                    # 删除指令文件
                    os.remove(instruction_file)
                
                model.moveStep()
                if model.timeStep % 5 == 0:
                    # 展示 display_text.txt 文件内容
                    if hasattr(planner, 'communication_manager') and planner.communication_manager:
                        planner.communication_manager.show_display_text(scenario_name)
                    #导出场景 7.27 打印exportSce()得到的vehicles中的stop_info
                    export_result = model.exportSce()
                    # 确保返回值数量正确
                    if len(export_result) == 3:
                        roadgraph, vehicles, facilities = export_result
                    elif len(export_result) == 2:
                        roadgraph, vehicles = export_result
                        facilities = {}  # 如果没有facilities，创建一个空字典
                    else:
                        # 处理其他情况
                        continue
                    # 如果自车开始行驶且场景存在
                    if model.tpStart and roadgraph:
                        log.info(f"Frame {model.timeStep}: Calling planner.plan with {len(vehicles)} vehicles and {len(facilities)} facilities")
                        if len(export_result) == 3:
                            trajectories = planner.plan(
                            model.timeStep * 0.1, roadgraph, vehicles, facilities
                            )# 规划轨迹
                        else:
                            trajectories = planner.plan(
                            model.timeStep * 0.1, roadgraph, vehicles
                            )# 规划轨迹
                        log.info(f"Frame {model.timeStep}: Completed planner.plan")
                        model.setTrajectories(trajectories) # 设置轨迹
                    else:
                        model.ego.exitControlMode() # 退出控制模式
                model.updateVeh()
            except TraCIException as e:
                log.error(f"TraCI error at step {model.timeStep}: {str(e)}")
                break
            except Exception as e:
                import traceback
                log.error(f"Unexpected error at step {model.timeStep}: {str(e)}")
                log.error(f"Traceback: {traceback.format_exc()}")
                break
    except Exception as e:
        log.error(f"Error during model execution: {str(e)}")
        raise
    finally:
        traci.close()
        log.info(f"{scenario_name} simulation ended")

def main():
    """主函数：处理命令行参数并运行对应场景"""
    parser = argparse.ArgumentParser(description='经典交通场景选择程序')
    parser.add_argument(
        '-s', '--scenario',
        type=str,
        choices=[
            SCENARIO_FORWARD_COLLISION,
            SCENARIO_HUMAN_VEHICLE,
            SCENARIO_VEHICLE_RSU,
            SCENARIO_VEHICLE_VEHICLE,
            COLLABORATION_SCENARIO
        ],
        default=COLLABORATION_SCENARIO,
        help='选择要运行的场景 (默认: 车车合作场景)'
    )
    parser.add_argument(
        '--ego-id',
        type=str,
        default="AV_0",
        help='自车ID (默认: AV_0)'
    )
    parser.add_argument(
        '--max-time',
        type=int,
        default=300,
        help='最大仿真时间（秒） (默认: 300)'
    )
    parser.add_argument(
        '--gui',
        action='store_true',
        help='使用SUMO GUI界面'
    )
    parser.add_argument(
        '--clear-messages',
        action='store_true',
        help='清理消息文件'
    )
    args = parser.parse_args()
    try:
        # 获取场景对应的路网文件
        if args.scenario not in file_paths:
            log.error(f"场景 {args.scenario} 不存在")
            sys.exit(1)
        
        # 解析路网文件路径
        path_info = file_paths[args.scenario]
        if len(path_info) == 3:
            # 包含add.xml文件
            net_file, rou_file, add_file = path_info
        else:
            net_file, rou_file = path_info
            add_file = None
        
        print(f"场景: {args.scenario}")
        print(f"路网文件: {net_file}")
        print(f"路由文件: {rou_file}")
        print(f"附加文件: {add_file}")
        
        # 设置SUMO路径
        sumo_gui = loc_config["LOC_SUMO_GUI"] if args.gui else loc_config["LOC_SUMO"]
        
        # 获取场景对应的主车ID，如果没有指定则使用场景的默认主车ID
        if args.ego_id == "AV_0":  # 如果使用的是默认的AV_0，则根据场景选择对应的主车ID
            ego_veh_id = SCENARIO_EGO_IDS.get(args.scenario, "AV_0")
        else:
            ego_veh_id = args.ego_id
        
        print(f"主车ID: {ego_veh_id}")
        
        # 运行模型
        run_model(
            scenario_name=args.scenario,
            net_file=net_file,
            rou_file=rou_file,
            add_file=add_file,
            ego_veh_id=ego_veh_id,
            SUMOGUI=sumo_gui,
            max_sim_time=args.max_time,
            if_clear_message_file=args.clear_messages
        )
        
    except Exception as e:
        log.error(f"程序执行错误: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()