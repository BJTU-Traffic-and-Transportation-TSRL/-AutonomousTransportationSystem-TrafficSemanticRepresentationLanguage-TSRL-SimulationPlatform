"""
前向碰撞预警:
应用定义:前向碰撞预警(FCW:Forward Collision Warning)是指,主车(HV)在车道上行驶,与在正前方同一车道的远车(RV)存在追尾碰撞危险时,FCW应用将对HV驾驶员进行预警。
"""
from simModel.egoTracking.model import Model
from trafficManager.traffic_manager import TrafficManager
from trafficManager.common.vehicle import Behaviour
import logger
import traci
from traci import TraCIException
import traci.constants as tc
import sys

log = logger.setup_app_level_logger(file_name="app_debug.log")

file_paths = {
    "ForwardCollisionWarning": (
        "networkFiles/ForwardCollisionWarning/ForwardCollisionWarning.net.xml",
        "networkFiles/ForwardCollisionWarning/ForwardCollisionWarning.rou.xml"
    ),
}

def run_model(
    net_file,
    rou_file,
    ego_veh_id="1",
    data_base=None,
    SUMOGUI="D:\sumo-win64-1.15.0\sumo-1.15.0\bin\sumo-gui.exe",
    sim_note="example simulation, LimSim-v-0.2.0.",
    carla_cosim=False,
    max_sim_time=100 # 新增参数，单位秒
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
        )
        model.start() # 初始化，无问题
        planner = TrafficManager(model) # 车辆规划模块，无问题

        # 主循环
        # 当自车未到达终点时，继续模拟
        while not model.tpEnd:
            try:
                model.moveStep()
                if model.timeStep % 5 == 0:
                    roadgraph, vehicles = model.exportSce() # 导出场景
                    # 如果自车开始行驶且场景存在
                    if model.tpStart and roadgraph: 
                        trajectories = planner.plan(
                            model.timeStep * 0.1, roadgraph, vehicles
                        )# 规划轨迹
                        model.setTrajectories(trajectories) # 设置轨迹
                    else:
                        model.ego.exitControlMode() # 退出控制模式

                model.updateVeh() # 6.16 更改内部结构
            except TraCIException as e:
                log.error(f"TraCI error at step {model.timeStep}: {str(e)}")
                break
            except Exception as e:
                log.error(f"Unexpected error at step {model.timeStep}: {str(e)}")
                break

    except Exception as e:
        log.error(f"Error during model initialization: {str(e)}")
    finally:
        try:
            # 确保正确关闭连接
            if 'model' in locals():
                model.destroy()
            try:
                traci.close()
            except Exception as e:
                # 如果连接已经关闭，忽略这个错误
                if "Not connected" not in str(e):
                    log.error(f"Error during traci cleanup: {str(e)}")
        except Exception as e:
            log.error(f"Error during model cleanup: {str(e)}")

if __name__ == "__main__":
    try:
        net_file, rou_file = file_paths['ForwardCollisionWarning']
        print("net_file:",net_file,",rou_file:",rou_file)
        run_model(net_file, 
                rou_file,
                ego_veh_id="1",
                carla_cosim=False,
                SUMOGUI=True
                )
    except Exception as e:
        log.error(f"Main program error: {str(e)}")
        sys.exit(1)