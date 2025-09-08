import traci
import sumolib
import time
import sys

"""
2025.7.20
# 定义方法：
# 1. 提取停车信息 
# 2. 验证停车信息是否已经被sumo正确应用，如没有则进行手动应用
"""

def extract_stop_info(rou_file):
    """
    从路由文件中提取停车信息
    
    参数:
    rou_file (str): 路由文件路径
    
    返回:
    dict: 包含车辆ID和对应停车信息的字典
    """
    vehicles_with_stops = {}
    print("stop info analysing...\n正在解析停车信息...")
    # 使用sumolib解析路由文件
    # 分割逗号分隔的多个文件路径
    for file_path in rou_file.split(','):
        # 处理每个文件路径
        for vehicle in sumolib.xml.parse(file_path.strip(), "vehicle"):
            vehicle_id = vehicle.getAttribute("id")
            stops = []
            
            # 检查车辆是否有停止行为
            if hasattr(vehicle, 'stop') and vehicle.stop is not None:
                for stop in vehicle.stop:
                    lane = stop.getAttribute("lane")
                    end_pos = float(stop.getAttribute("endPos"))
                    until = float(stop.getAttribute("until"))
                    
                    stops.append({
                        'lane': lane,
                        'end_pos': end_pos,
                        'until': until
                    })
            
            if stops:
                vehicles_with_stops[vehicle_id] = stops
                print(f"找到车辆 {vehicle_id} 的停车信息: {stops}")
        
        return vehicles_with_stops

from simModel.common.carFactory import Vehicle  # 导入Vehicle类

def assign_stops_to_vehicles(vehicles_with_stops, vehicles):
    """将停车信息分派给对应的Vehicle实例"""
    # 支持列表和字典两种类型的vehicles参数
    if isinstance(vehicles, dict):
        # 如果是字典类型，按原来的方式处理
        for vehicle_id, stops in vehicles_with_stops.items():
            if vehicle_id in vehicles:
                vehicles[vehicle_id].set_stop_info(stops)
    elif isinstance(vehicles, list):
        # 如果是列表类型，通过vehicle.id匹配
        for vehicle in vehicles:
            if vehicle.id in vehicles_with_stops:
                vehicle.set_stop_info(vehicles_with_stops[vehicle.id])


def validate_and_apply_stops(vehicles):
    """
    验证停车信息是否已经被sumo正确应用，如没有则进行手动应用
    
    参数:
    vehicles (list or dict): Vehicle实例列表或字典
    """
    try:
        vehicle_ids = traci.vehicle.getIDList()
        
        # 支持列表和字典两种类型的vehicles参数
        if isinstance(vehicles, dict):
            vehicle_list = vehicles.values()
        elif isinstance(vehicles, list):
            vehicle_list = vehicles
        else:
            vehicle_list = []
            
        for vehicle in vehicle_list:
            if isinstance(vehicle, Vehicle) and vehicle.id in vehicle_ids and vehicle.stop_info:
                # 检查车辆是否有停车信息
                current_stops = traci.vehicle.getStops(vehicle.id)
                if not current_stops:
                    # 如果sumo没有自动应用停车信息，手动应用
                    print(f"手动应用车辆 {vehicle.id} 的停车信息")
                    vehicle.apply_stop_info()
                else:
                    # 验证停车信息
                    print(f"车辆 {vehicle.id} 的停车信息已应用")

    except Exception as e:
        print(f"仿真错误: {str(e)}")