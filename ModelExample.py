from simModel.egoTracking.model import Model
from trafficManager.traffic_manager import TrafficManager
import logger
import os
# config a logger, set use_stdout=True to output log to terminal
# 将日志文件保存到DEBUG_TSRL目录
log_dir = "DEBUG_TSRL"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
log_file_path = os.path.join(log_dir, "app_debug_example.log")
log = logger.setup_app_level_logger(file_name=log_file_path,
                                    level="DEBUG",
                                    use_stdout=False)
file_paths = {
    "corridor": (
        "networkFiles/corridor/corridor.net.xml",
        "networkFiles/corridor/corridor.rou.xml",
    ),
    "CarlaTown01": (
        "networkFiles/CarlaTown01/Town01.net.xml",
        "networkFiles/CarlaTown01/carlavtypes.rou.xml,networkFiles/CarlaTown01/Town01.rou.xml"
    ),
    "CarlaTown05": (
        "networkFiles/CarlaTown05/Town05.net.xml",
        "networkFiles/CarlaTown05/carlavtypes.rou.xml,networkFiles/CarlaTown05/Town05.rou.xml"
    ),
    "bigInter": (
        "networkFiles/bigInter/bigInter.net.xml",
        "networkFiles/bigInter/bigInter.rou.xml",
    ),
    "roundabout": (
        "networkFiles/roundabout/roundabout.net.xml",
        "networkFiles/roundabout/roundabout.rou.xml",
    ),
    "bilbao":   (
        "networkFiles/bilbao/osm.net.xml",
        "networkFiles/bilbao/osm.rou.xml",
    ),
    #######
    # Please make sure you have request the access from https://github.com/ozheng1993/UCF-SST-CitySim-Dataset and put the road network files (.net.xml) in the relevent networkFiles/CitySim folder
    "freewayB": (
        "networkFiles/CitySim/freewayB/freewayB.net.xml",
        "networkFiles/CitySim/freewayB/freewayB.rou.xml",
    ),
    "Expressway_A": (
        "networkFiles/CitySim/Expressway_A/Expressway_A.net.xml",
        "networkFiles/CitySim/Expressway_A/Expressway_A.rou.xml",
    ),
    ########
}


def run_model(
    net_file,
    rou_file,
    ego_veh_id="10",
    data_base="egoTrackingTest.db",
    SUMOGUI="D:\sumo-win64-1.15.0\sumo-1.15.0\bin\sumo-gui.exe",
    sim_note="example simulation, ATSISP-v-1.0.",
    carla_cosim=False,
    max_sim_time=100, # 新增参数，单位秒
    communication = True,# 新增参数，全局通信管理器
    if_clear_message_file=False # 8.27 新增参数，是否清理消息文件
):
    # 加载配置文件
    from utils.load_config import load_config
    config = load_config("trafficManager/config.yaml")
    
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
        config=config  # 传递配置信息
    )
    model.start()
    planner = TrafficManager(model)
    # 8.27 新增：清理消息文件or清理消息内容：
    model.clear_message_files(planner, if_clear_message_file)
    
    while not model.tpEnd:
        model.moveStep()
        if model.timeStep % 5 == 0:
            roadgraph, vehicles = model.exportSce()
            if model.tpStart and roadgraph:
                trajectories = planner.plan(
                    model.timeStep * 0.1, roadgraph, vehicles
                )
                model.setTrajectories(trajectories)
            else:
                model.ego.exitControlMode()
        model.updateVeh()

    model.destroy()


if __name__ == "__main__":
    # 添加一条测试日志
    log.info("ModelExample Starts")
    try:
        net_file, rou_file = file_paths['CarlaTown05']
        run_model(net_file, rou_file, ego_veh_id="10", carla_cosim=False)
    except Exception as e:
        log.exception("ModelExample Error")
        raise
    # 程序正常结束时也记录一条日志
    log.info("ModelExample Ends")
