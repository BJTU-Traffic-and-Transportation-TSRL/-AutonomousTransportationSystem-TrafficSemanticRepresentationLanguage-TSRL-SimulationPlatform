# 导入traci模块
import traci

def validate_and_apply_stops(vehicles):
    """
    验证停车信息是否已经被sumo正确应用，如没有则进行手动应用
    
    参数:
    vehicles (list): Vehicle实例列表
    """
    try:
        vehicle_ids = traci.vehicle.getIDList()

        for vehicle in vehicles:
            if vehicle.id in vehicle_ids and vehicle.stop_info:
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