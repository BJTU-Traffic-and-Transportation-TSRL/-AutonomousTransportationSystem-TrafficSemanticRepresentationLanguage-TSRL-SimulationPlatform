"""
提取车辆停止信息——使用非traci的方法
"""

import sumolib

# 加载路由文件并提取停止信息
route_file = "D:/学习资料/董组/时空推理/LimSim/LimSim/networkFiles/ForwardCollisionWarning/ForwardCollisionWarning.rou.xml"

# 使用 sumolib.xml.parse 解析路由文件
vehicles_with_stops = {}
for vehicle in sumolib.xml.parse(route_file, "vehicle"): # 遍历所有车辆
    vehicle_id = vehicle.id
    depart = vehicle.depart
    stops = []
    # 检查车辆是否有停止行为
    if hasattr(vehicle, 'stop') and vehicle.stop is not None: # 检查 vehicle 是否有 stop 属性
        print(vehicle.stop) # 打印车辆停止信息
        for stop in vehicle.stop:
            lane = stop.getAttribute("lane")
            end_pos = stop.getAttribute("endPos")
            until = stop.getAttribute("until")
            """成功读取停车成分"""
            stops.append({
                'lane': lane,
                'end_pos': end_pos,
                'until': until
            })
    else:
        print(f"Vehicle ID: {vehicle_id} has no stop behavior")
    if stops:
        vehicles_with_stops[vehicle_id] = {
            'depart': depart,
            'stops': stops
        }

# 打印提取的停止信息
for vehicle_id, info in vehicles_with_stops.items():
    print(f"Vehicle ID: {vehicle_id}, Depart: {info['depart']}")
    for stop in info['stops']:
        print(f"  Lane: {stop['lane']}, End Position: {stop['end_pos']}, Until: {stop['until']}")