"""
此程序用来验证traci能否正常读取rou.xml文件
"""
import traci
import sys
import time
import os

# 配置文件路径
NET_FILE = r'D:\学习资料\董组\时空推理\LimSim\LimSim\networkFiles\ForwardCollisionWarning\ForwardCollisionWarning.net.xml'
ROU_FILE = r'D:\学习资料\董组\时空推理\LimSim\LimSim\networkFiles\ForwardCollisionWarning\ForwardCollisionWarning.rou.xml'

# SUMO启动配置
sumo_binary = r'D:\sumo-win64-1.15.0\sumo-1.15.0\bin\sumo-gui.exe'

if __name__ == '__main__':
    try:
        traci.start([
            sumo_binary,
            '-n', NET_FILE,
            '-r', ROU_FILE,
            '--step-length', '0.1',
            '--collision.action', 'remove',
            '--quit-on-end'
        ])
        
        stop_triggered = False  # 新增状态标志初始化

        # 主仿真循环
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            # # 控制1号车在E2车道30米处停车
            # if traci.vehicle.getIDList() and '1' in traci.vehicle.getIDList():
            #     lane_id = traci.vehicle.getLaneID('1')
            #     if lane_id.startswith('E2') and not stop_triggered:
            #         traci.vehicle.setStop(
            #             vehID='1',
            #             edgeID=lane_id.split('_')[0],
            #             pos=30.0,
            #             laneIndex=int(lane_id.split('_')[1]),
            #             duration=1000
            #         )
            #         print(f"车辆1已在{lane_id}车道30米处停车")
            #         stop_triggered = True

            time.sleep(0.05)

    except Exception as e:
        print(f"仿真错误: {str(e)}")
    finally:
        traci.close()
        sys.exit(0)