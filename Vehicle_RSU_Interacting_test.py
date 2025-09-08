#!/usr/bin/env python3
import traci
import json
import time

SUMO_CMD = ["sumo-gui", "-c", "networkFiles\\Vehicle_RSU_Interacting\\Vehicle_RSU_Interacting.sumocfg"]
traci.start(SUMO_CMD)

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    # 1. RSU 收集前方 200 m 路况
    rsu_x = 400  # RSU 位置
    detect_range = 200  # 检测范围
    vehicles = []
    for v in traci.vehicle.getIDList():
        x, _ = traci.vehicle.getPosition(v)
        # 如果RSU前方检测范围内存在车辆
        if rsu_x <= x <= rsu_x + detect_range:
            vehicles.append({
                "id": v,
                "lane": traci.vehicle.getLaneID(v),
                "speed": traci.vehicle.getSpeed(v),
                "pos": traci.vehicle.getLanePosition(v)
            })

    # 2. 广播 JSON 互操作语句
    msg = {
        "type": "RSU_BROADCAST",
        "range": detect_range,
        "vehicles": vehicles
    }
    # 车辆端用普通参数存取
    traci.vehicle.setParameter("AV_0", "rsu_msg", json.dumps(msg))

    # 3. AV 解析并决策
    if "AV_0" in traci.vehicle.getIDList():
        raw = traci.vehicle.getParameter("AV_0", "rsu_msg")
        if raw:
            data = json.loads(raw)
            # 简单推理：若前方外侧车道空 → 变道
            outer = [v for v in data["vehicles"] if "main_2" in v["lane"]]
            if not outer:
                traci.vehicle.changeLane("AV_0", 2, 3)  # 2=外侧车道，3 s
            else:
                traci.vehicle.setSpeed("AV_0", max(15, data["vehicles"][0]["speed"] - 2))

    time.sleep(0.1)

traci.close()