import traci

# 启动 SUMO 模拟
sumo_cmd = [
    'sumo-gui',
    '-n', 'D:/学习资料/董组/时空推理/LimSim/LimSim/networkFiles/ForwardCollisionWarning/ForwardCollisionWarning.net.xml',
    '-r', 'D:/学习资料/董组/时空推理/LimSim/LimSim/networkFiles/ForwardCollisionWarning/ForwardCollisionWarning.rou.xml',
    '--step-length', '0.1',
    '--xml-validation', 'never',
    '--error-log', 'sumo_errors.log',
    '--lateral-resolution', '10',
    '--start',
    '--quit-on-end',
    '--collision.action', 'remove'
]

traci.start(sumo_cmd,port=8813)

# 模拟循环
step = 0
vehicles_with_stops = {}
while step < 1000:
    traci.simulationStep()  # 执行一步模拟
    # 获取所有车辆的 ID
    vehicle_ids = traci.vehicle.getIDList()
    for vehicle_id in vehicle_ids:
        # 获取车辆的停止行为
        stops = traci.vehicle.getStops(vehicle_id)
        if stops:
            # 提取停止信息
            stops_info = []
            for stop in stops:
                lane = stop.lane
                end_pos = stop.endPos
                until = stop.until
                stops_info.append({
                    'lane': lane,
                    'end_pos': end_pos,
                    'until': until
                })
            vehicles_with_stops[vehicle_id] = stops_info
    step += 1

# 打印提取的停止信息
for vehicle_id, stops in vehicles_with_stops.items():
    print(f"Vehicle ID: {vehicle_id}")
    for stop in stops:
        print(f"  Lane: {stop['lane']}, End Position: {stop['end_pos']}, Until: {stop['until']}")

# 关闭连接
traci.close()