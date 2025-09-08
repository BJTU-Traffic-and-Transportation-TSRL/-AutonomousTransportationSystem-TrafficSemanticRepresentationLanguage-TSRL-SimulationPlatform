"""
前向碰撞预警:
应用定义:前向碰撞预警(FCW:Forward Collision Warning)是指,主车(HV)在车道上行驶,与在正前方同一车道的远车(RV)存在追尾碰撞危险时,FCW应用将对HV驾驶员进行预警。
"""
from simModel.egoTracking.model import Model
from trafficManager.traffic_manager import TrafficManager
from trafficManager.common.vehicle import Behaviour
from simModel.common.carFactory import Vehicle  # 导入正确的Vehicle类
import logger
import traci
from traci import TraCIException
import traci.constants as tc
import sys
import os


log = logger.setup_app_level_logger(file_name="app_debug_FCW.log")

file_paths = {
    "ForwardCollisionWarning": (
        "networkFiles/ForwardCollisionWarning/ForwardCollisionWarning.net.xml",
        "networkFiles/ForwardCollisionWarning/ForwardCollisionWarning.rou.xml"
    )
}

def run_model(
    net_file,
    rou_file,
    ego_veh_id="1",
    data_base=None,
    SUMOGUI="D:\sumo-win64-1.15.0\sumo-1.15.0\bin\sumo-gui.exe",
    sim_note="example simulation, LimSim-v-0.2.0.",
    carla_cosim=False,
    max_sim_time=100, # 新增参数，单位秒
    communication = True,# 新增参数，全局通信管理器
    if_clear_message_file=False # 8.27 新增参数，是否清理消息文件
):
    try:
        model = Model(
            ego_veh_id,
            net_file,
            rou_file,
            dataBase=data_base,
            SUMOGUI=SUMOGUI,
            simNote=sim_note,
            carla_cosim=carla_cosim,
            max_steps=int(max_sim_time * 10), # 将max_sim_time转换为步长
            communication=communication, # 25.8.16 新增参数，全局通信管理器
        )
        model.start() # 初始化
        planner = TrafficManager(model) # 初始化车辆规划模块
        # 8.27 新增：清理消息文件or清理消息内容：
        if if_clear_message_file == True:
            # 8.19 新增：删除所有消息历史文件
            planner.communication_manager.cleanup_message_files()
            # 8.27 新增：删除display_text文件
            planner.communication_manager.cleanup_display_text(loc="message_history")
        else:
            # 8.19 新增：清空所有消息历史文件内容
            planner.communication_manager.clear_message_files_content()
            # 8.27 新增：清空display_text文件里的内容
            planner.communication_manager.clear_display_text_content(loc="message_history")
        
        # 主循环
        # 当自车未到达终点时，继续模拟
        while not model.tpEnd:
            # 修改这行代码
            stopping_car= next((v for v in model.vehicles if v.id == "0"), None)
            if stopping_car:
                print(stopping_car.stop_info)
            try:
                model.moveStep()
                if model.timeStep % 5 == 0:
                    # 导出场景 
                    #  7.27 打印exportSce()得到的vehicles中的stop_info
                    roadgraph, vehicles = model.exportSce() 
                    # 如果自车开始行驶且场景存在
                    if model.tpStart and roadgraph: 
                        trajectories = planner.plan(
                        model.timeStep * 0.1, roadgraph, vehicles
                        )# 规划轨迹
                        model.setTrajectories(trajectories) # 设置轨迹
                    else:
                        model.ego.exitControlMode() # 退出控制模式

                model.updateVeh() # 6.16 更改该函数内部结构
            except TraCIException as e:
                log.error(f"TraCI error at step {model.timeStep}: {str(e)}")
                break
            except Exception as e:
                log.error(f"Unexpected error at step {model.timeStep}: {str(e)}")
                break



    except Exception as e:
        log.error(f"Error during model initialization: {str(e)}")

if __name__ == "__main__":
    try:
        net_file, rou_file = file_paths['ForwardCollisionWarning']
        print("net_file:",net_file,",rou_file:",rou_file)
        
        # 使用相对于当前文件的路径创建input_FCW.txt
        current_dir = os.path.dirname(os.path.abspath(__file__))
        tsil_dir = os.path.join(current_dir, "TSIL")
        input_file_path = os.path.join(tsil_dir, "input_FCW.txt")
        
        # 确保TSIL目录存在
        os.makedirs(tsil_dir, exist_ok=True)
        
        # 在指定路径打开input_FCW文本文档并写入指定内容
        content_to_write = "EmergencyStation(RV);\nSelfVehicle(HV);\nChangeLane(y):-SelfVehicle(y),EmergencyStation(x);\nASK ChangeLane(y);"
        with open(input_file_path, 'w') as file:
            file.write(content_to_write)

        run_model(net_file, 
                rou_file,
                ego_veh_id="1",
                carla_cosim=False,
                # SUMOGUI=True
                )
    except Exception as e:
        log.error(f"Main program error: {str(e)}")
        sys.exit(1)