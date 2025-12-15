"""
车-车交互测试：
应用定义:驾驶车辆与前方拥堵车辆交互,测试自主式交通主体间语义表达与理解方法可支持道路交通环境下车-车交互

测试步骤：
1.	仿真车辆即将抵达无信号交叉口，但无信号交叉口发生交通堵塞，前车速度为0km/h。
2.	车辆间相互发送行驶信息，仿真车辆利用接收信息进行知识推理，得出仿真车辆需要在路口以外排队等候且不得进入路口的指令。
3.	仿真车辆得出指令后，在停车线前停下或与前车保持安全距离并停下，防止车辆穿插或者超车加剧交叉口的拥堵程度。
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

# 将日志文件保存到DEBUG_TSRL目录
log_dir = "DEBUG_TSRL"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
log_file_path = os.path.join(log_dir, "app_debug_VVehicle.log")
log = logger.setup_app_level_logger(file_name=log_file_path)
# 设置场景名称
Scenario_Name = "Vehicle_Vehicle_Interacting"
# 设置路网文件路径
file_paths = {
    "{}".format(Scenario_Name): (
        "networkFiles/Vehicle_Vehicle_Interacting/Vehicle_Vehicle_Interacting.net.xml",
        "networkFiles/Vehicle_Vehicle_Interacting/Vehicle_Vehicle_Interacting.rou.xml"
    )
}

def run_model(
    net_file,
    rou_file,
    add_file,
    ego_veh_id,
    data_base='Vehicle_Vehicle_Interacting.db',
    SUMOGUI="D:\\sumo-win64-1.15.0\\sumo-1.15.0\\bin\\sumo-gui.exe",
    sim_note="Vehicle-Vehicle interaction simulation, ATSISP-v-1.0.",
    carla_cosim=False,
    max_sim_time=200,  # 单位秒
    communication=True,  # 全局通信管理器
    if_clear_message_file=False  # 是否清理消息文件本体
):
    """运行车辆与其他车辆交互模拟"""
    try:
        # 加载配置文件
        from utils.load_config import load_config
        config = load_config("trafficManager/config.yaml")
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
            Scenario_Name=Scenario_Name, # 场景名称
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
                model.moveStep()
                if model.timeStep % 5 == 0:
                    # 展示 display_text.txt 文件内容
                    planner.communication_manager.show_display_text(Scenario_Name)
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
        log.info("SUMO simulation ended")

if __name__ == "__main__":
    try:
        if len(file_paths['{}'.format(Scenario_Name)]) == 3:
            # add.xml exists
            net_file, rou_file,add_file = file_paths['{}'.format(Scenario_Name)]
        else:
            net_file, rou_file = file_paths['{}'.format(Scenario_Name)]
            add_file = None
        print("net_file:\n", net_file, "\nrou_file:\n", rou_file,"\nadd_file:\n", add_file)
        # 运行模型
        run_model(
            net_file, 
            rou_file,
            add_file=add_file,
            ego_veh_id="AV",
            carla_cosim=False,
            max_sim_time=300,
            # SUMOGUI=True
        )
    except Exception as e:
        log.error(f"Main program error: {str(e)}")
        sys.exit(1)