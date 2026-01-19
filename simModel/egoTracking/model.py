import os
import sqlite3
import threading
import time
from typing import List
import xml.etree.ElementTree as ET
from datetime import datetime
from queue import Queue
from math import sin, cos, pi

import dearpygui.dearpygui as dpg
import numpy as np
import traci
from rich import print
from traci import TraCIException
from traci import vehicle
from typing import Dict
from utils.roadgraph import RoadGraph

from read_stop_info import validate_and_apply_stops
from simModel.common.carFactory import Vehicle, egoCar
from simModel.common.gui import GUI
from simModel.egoTracking.movingScene import MovingScene
from simModel.common.networkBuild import NetworkBuild
from utils.trajectory import State, Trajectory
from utils.simBase import MapCoordTF, vehType

from evaluation.evaluation import RealTimeEvaluation
import read_stop_info # 7.20 添加停车解析内容

class Model:
    '''
        egoID: id of ego car,str;
        netFile: network files, e.g. `example.net.xml`;
        rouFile: route files, e.g. `example.rou.xml`. if you have 
                vehicle-type file as an input, define this parameter as 
                `examplevTypes.rou.xml,example.rou.xml`;
        obseFile: obstacle files, e.g. `example.obs.xml`;
        addFile: additional files, e.g. `example.add.xml`;
        dataBase: the name of the database, e.g. `example.db`. if it is not 
                specified, it will be named with the current timestamp.
        SUMOGUI: boolean variable, used to determine whether to display the SUMO 
                graphical interface;
        simNote: the simulation note information, which can be any information you 
                wish to record. For example, the version of your trajectory 
                planning algorithm, or the user name of this simulation.
    '''

    def __init__(self,
                 egoID: str,
                 netFile: str,
                 rouFile: str,
                 obsFile: str = None,
                 addFile: str = None,
                 dataBase: str = None,
                 SUMOGUI: int = 0,
                 simNote: str = None,
                 carla_cosim: bool = False,
                 max_steps: int = 1000,# 4.23 添加最大模拟步长
                 communication: bool = False, # 25.8.16 新增参数，全局通信管理器
                 Scenario_Name: str = None, # 25.10.20 场景名称
                 config: dict = None, # 新增参数，用于传递配置信息
                 ) -> None:

        print('[green bold]Model initialized at {}.[/green bold]'.format(
            datetime.now().strftime('%H:%M:%S.%f')[:-3]))
        self.netFile = netFile
        self.rouFile = rouFile
        self.obsFile = obsFile
        self.addFile = addFile
        self.SUMOGUI = SUMOGUI
        self.sim_mode: str = 'RealTime'
        self.timeStep = 0
        self.max_steps = max_steps
        # tpStart marks whether the trajectory planning is started,
        # when the ego car appears in the network, tpStart turns into 1.
        self.tpStart = 0 #自车出现在网络中时，tpStart变为1
        # tpEnd marks whether the trajectory planning is end,
        # when the ego car leaves the network, tpEnd turns into 1.
        self.tpEnd = 0 #自车离开网络时，tpEnd变为1
        # need carla cosimulation
        self.carla_cosim = carla_cosim # 是否需要Carla协同仿真
        self.communication=communication # 25.8.16 新增参数，是否添加全局通信管理器
        self.Scenario_Name = Scenario_Name # 25.10.20 场景名称
        self.config = config  # 保存配置信息
        
        # 从配置中获取DEAREA值，如果不存在则使用默认值50.0
        dearea = config.get("DEAREA", 50.0) if config else 50.0
        self.ego = egoCar(egoID, deArea=dearea)

        if dataBase:
            self.dataBase = dataBase
        else:
            self.dataBase = datetime.strftime(
                datetime.now(), '%Y-%m-%d_%H-%M-%S') + '_egoTracking' + '.db'
        
        # 7.20：添加模型车辆列表
        self.vehicles: List[Vehicle] = []
        # 7.20 添加停车解析内容
        self.vehicles_with_stops = read_stop_info.extract_stop_info(self.rouFile)        
        self.createDatabase() # 创建数据库
        self.simDescriptionCommit(simNote)
        self.dataQue = Queue()
        self.createTimer()
        # 使用完整的数据库路径（包含Database/前缀）传递给NetworkBuild类
        db_path = os.path.join("Database", self.dataBase)
        self.nb = NetworkBuild(db_path, self.netFile, self.obsFile, self.addFile)
        self.nb.getData()
        self.nb.buildTopology()

        self.ms = MovingScene(self.nb, self.ego, self.vehicles_with_stops)# 7.27 更新 Model 类初始化 MovingScene

        self.allvTypes = None

        try:
            self.gui = GUI('real-time-ego',self)
        except Exception as e:
            # 记录GUI初始化错误
            import logging
            logging.error(f"GUI初始化失败: {str(e)}", exc_info=True)
            raise  # 重新抛出异常以便上层处理

        self.evaluation = RealTimeEvaluation(dt=0.1)

     # 7.20 定义新方法，获得非Ego车辆列表
    def getVehicleList(self):
        # 8.17：获取roufile中的所有车辆ID，实例化车辆列表
        vehicles_ids = []
        processed_ids = set()  # 避免重复添加车辆ID
        # 处理逗号分隔的多个文件路径
        rou_files = self.rouFile.split(',')
        for rou_file in rou_files:
            rou_file = rou_file.strip()  # 去除可能的空格
            if not os.path.exists(rou_file):
                logging.warning(f"路由文件不存在: {rou_file}")
                continue
                
            try:
                elementTree = ET.parse(rou_file)
                root = elementTree.getroot()
                for child in root:
                    if child.tag == 'vehicle' and child.attrib['id'] != self.ego.id:
                        vehicle_id = child.attrib['id']
                        # 避免重复添加车辆ID
                        if vehicle_id not in processed_ids:
                            vehicles_ids.append(vehicle_id)
                            processed_ids.add(vehicle_id)
            except ET.ParseError as e:
                logging.error(f"解析路由文件失败 {rou_file}: {str(e)}")
                continue
                
        vehicles = []
        for vehicle_id in vehicles_ids:
            vehicle = Vehicle(vehicle_id)
            vehicles.append(vehicle)
        return vehicles

    # 创建数据库
    def createDatabase(self):
        # 确保Database目录存在
        db_dir = "Database"
        if not os.path.exists(db_dir):
            os.makedirs(db_dir)
        # if database exist then delete it
        db_path = os.path.join(db_dir, self.dataBase)
        if os.path.exists(db_path):
            os.remove(db_path)
        conn = sqlite3.connect(db_path)
        cur = conn.cursor()

        cur.execute('''CREATE TABLE IF NOT EXISTS simINFO(
                        startTime TIMESTAMP PRIMARY KEY,
                        localPosx REAL,
                        localPosy REAL,
                        radius REAL,
                        egoID TEXT,
                        netBoundary TEXT,
                        description TEXT,
                        note TEXT);''')

        cur.execute('''CREATE TABLE IF NOT EXISTS frameINFO(
                            frame INT NOT NULL,
                            vid TEXT NOT NULL,
                            vtag TEXT NOT NULL,
                            x REAL NOT NULL,
                            y REAL NOT NULL,
                            yaw REAL NOT NULL,
                            speed REAL NOT NULL,
                            accel REAL NOT NULL,
                            laneID TEXT NOT NULL,
                            lanePos REAL NOT NULL,
                            routeIdx INT NOT NULL,
                            PRIMARY KEY (frame, vid));''')

        cur.execute('''CREATE TABLE IF NOT EXISTS vehicleINFO(
                            vid TEXT PRIMARY KEY,
                            length REAL NOT NULL,
                            width REAL NOT NULL,
                            maxAccel REAL,
                            maxDecel REAL,
                            maxSpeed REAL,
                            vTypeID TEXT NOT NULL,
                            routes TEXT NOT NULL);''')

        cur.execute('''CREATE TABLE IF NOT EXISTS edgeINFO(
                            id TEXT RRIMARY KEY,
                            laneNumber INT NOT NULL,
                            from_junction TEXT,
                            to_junction TEXT);''')

        cur.execute('''CREATE TABLE IF NOT EXISTS laneINFO(
                            id TEXT PRIMARY KEY,
                            rawShape TEXT,
                            width REAL,
                            maxSpeed REAL,
                            edgeID TEXT,
                            length REAL);''')

        cur.execute('''CREATE TABLE IF NOT EXISTS junctionLaneINFO(
                            id TEXT PRIMARY KEY,
                            width REAL,
                            maxSpeed REAL,
                            length REAL,
                            tlLogicID TEXT,
                            tlsIndex INT);''')

        cur.execute('''CREATE TABLE IF NOT EXISTS junctionINFO(
                            id TEXT PRIMARY KEY,
                            rawShape TEXT);''')

        cur.execute('''CREATE TABLE IF NOT EXISTS tlLogicINFO(
                            id TEXT PRIMARY KEY,
                            tlType TEXT,
                            preDefPhases TEXT)''')

        cur.execute('''CREATE TABLE IF NOT EXISTS connectionINFO(
                            fromLane TEXT NOT NULL,
                            toLane TEXT NOT NULL,
                            direction TEXT,
                            via TEXT,
                            PRIMARY KEY (fromLane, toLane));''')

        cur.execute('''CREATE TABLE IF NOT EXISTS trafficLightStates(
                            frame INT NOT NULL,
                            id TEXT NOT NULL,
                            currPhase TEXT,
                            nextPhase TEXT,
                            switchTime REAL,
                            PRIMARY KEY (frame, id));''')

        cur.execute('''CREATE TABLE IF NOT EXISTS circleObsINFO(
                            id TEXT PRIMARY KEY,
                            edgeID TEXT NOT NULL,
                            centerx REAL NOT NULL,
                            centery REAL NOT NULL,
                            radius REAL NOT NULL);''')

        cur.execute('''CREATE TABLE IF NOT EXISTS rectangleObsINFO(
                            id TEXT PRIMARY KEY,
                            edgeID TEXT NOT NULL,
                            centerx REAL NOT NULL,
                            centery REAL NOT NULL,
                            length REAL NOT NULL,
                            width REAL NOT NULL,
                            yaw REAL NOT NULL);''')

        cur.execute('''CREATE TABLE IF NOT EXISTS geohashINFO(
                            ghx INT NOT NULL,
                            ghy INT NOT NULL,
                            edges TEXT,
                            junctions TEXT,
                            PRIMARY KEY (ghx, ghy));''')

        cur.execute('''CREATE TABLE IF NOT EXISTS evaluationINFO(
                    frame INT PRIMARY KEY,
                    offset REAL,
                    discomfort REAL,
                    collision REAL,
                    orientation REAL,
                    consumption REAL);''')

        conn.commit()
        cur.close()
        conn.close()
        
        
    def simDescriptionCommit(self, simNote: str):
        currTime = datetime.now()
        insertQuery = '''INSERT INTO simINFO VALUES (?, ?, ?, ?, ?, ?, ?, ?);'''
        conn = sqlite3.connect("Database/" + self.dataBase)
        cur = conn.cursor()
        cur.execute(
            insertQuery,
            (currTime, None, None, None, self.ego.id, '', 'ego track', simNote))

        conn.commit()
        cur.close()
        conn.close()

    # 创建定时器
    def createTimer(self):
        if not self.tpEnd or not self.dataQue.empty():
            t = threading.Timer(1, self.dataStore)
            t.daemon = True
            t.start()

    # 将数据存储到数据库
    def dataStore(self):
        # stime = time.time()
        cnt = 0
        conn = sqlite3.connect("Database/" + self.dataBase, check_same_thread=False)
        cur = conn.cursor()
        while cnt < 1000 and not self.dataQue.empty():
            tableName, data = self.dataQue.get()
            sql = 'INSERT INTO %s VALUES ' % tableName + \
                '(' + '?,'*(len(data)-1) + '?' + ')'
            try:
                cur.execute(sql, data)
            except sqlite3.IntegrityError:
                pass
            cnt += 1

        conn.commit()
        cur.close()
        conn.close()

        self.createTimer()

    # DEFAULT_VEHTYPE
    # 获取所有车辆类型
    def getAllvTypeID(self) -> list:
        allvTypesID = []
        if ',' in self.rouFile:
            vTypeFile = self.rouFile.split(',')[0]
            elementTree = ET.parse(vTypeFile)
            root = elementTree.getroot()
            for child in root:
                if child.tag == 'vType':
                    vtid = child.attrib['id']
                    allvTypesID.append(vtid)
        else:
            elementTree = ET.parse(self.rouFile)
            root = elementTree.getroot()
            for child in root:
                if child.tag == 'vType':
                    vtid = child.attrib['id']
                    allvTypesID.append(vtid)

        return allvTypesID

    # 启动SUMO模拟
    def start(self):
        if self.carla_cosim: # 如果需要Carla协同仿真
            num_clients = "2" # 设置客户端数量为2
        else:
            num_clients = "1" # 设置客户端数量为1
        print("SUMO starting...\n正在启动sumo仿真...")
        traci.start([
            'sumo' if not self.SUMOGUI else 'sumo-gui', # 启动SUMO或SUMO-GUI
            '-n', self.netFile, # 加载网络文件
            '-r', self.rouFile, # 加载路由文件
            '--step-length', '0.1', # 设置步长为0.1秒
            '--xml-validation', 'never', # 设置XML验证为从不
            '--error-log', 'sumo_errors.log', # 设置错误日志文件
            '--lateral-resolution', '10', # 设置侧向分辨率为10
            '--start', # 设置启动参数
            '--quit-on-end', # 设置结束时退出
            '-W', # 设置宽度
            '--collision.action', # 设置碰撞行为
            'remove', # 设置碰撞行为为移除
            "--num-clients",
            num_clients,
        ], port = 8813)
        traci.setOrder(1)
        print("route info analysing...\n正在解析rou.xml文件...")

        allvTypeID = self.getAllvTypeID() # 获取所有车辆类型ID
        allvTypes = {}
        if allvTypeID:
            for vtid in allvTypeID:
                vtins = vehType(vtid)
                vtins.maxAccel = traci.vehicletype.getAccel(vtid)
                vtins.maxDecel = traci.vehicletype.getDecel(vtid)
                vtins.maxSpeed = traci.vehicletype.getMaxSpeed(vtid)
                vtins.length = traci.vehicletype.getLength(vtid)
                vtins.width = traci.vehicletype.getWidth(vtid)
                vtins.vclass = traci.vehicletype.getVehicleClass(vtid)
                allvTypes[vtid] = vtins
        else:
            vtid = 'DEFAULT_VEHTYPE'
            vtins = vehType(vtid)
            vtins.maxAccel = traci.vehicletype.getAccel(vtid)
            vtins.maxDecel = traci.vehicletype.getDecel(vtid)
            vtins.maxSpeed = traci.vehicletype.getMaxSpeed(vtid)
            vtins.length = traci.vehicletype.getLength(vtid)
            vtins.width = traci.vehicletype.getWidth(vtid)
            vtins.vclass = traci.vehicletype.getVehicleClass(vtid)
            allvTypes[vtid] = vtins
            self.allvTypes = allvTypes
        self.allvTypes = allvTypes
        # 7.27：获取所有非Ego车辆实体
        self.vehicles=self.getVehicleList()
        # 8.19：加入Ego车辆实体
        self.vehicles.append(self.ego)
 
        # 7.27：停车信息分配
        # 如果self.vehicles非空，将self.vehicles_with_stops分配给对应车辆
        if self.vehicles:
            read_stop_info.assign_stops_to_vehicles(self.vehicles_with_stops, self.vehicles)
            # 同时更新 MovingScene 的停车信息
            self.ms.vehicles_with_stops = self.vehicles_with_stops
        # 打印所有车辆的stop_info
        print("所有车辆的stop_info：")
        for v in self.vehicles:
            if v.stop_info:
                print("车辆ID：", v.id, "停车信息：", v.stop_info)
            else:
                print("车辆ID：", v.id, "停车信息：无")

    # 绘制车辆状态
    def plotVState(self):
        if self.ego.speedQ:
            currLane = traci.vehicle.getLaneID(self.ego.id)
            if ':' not in currLane:
                try:
                    laneMaxSpeed = traci.lane.getMaxSpeed(currLane)
                except TraCIException:
                    laneMaxSpeed = 15
            else:
                laneMaxSpeed = 15
            dpg.set_axis_limits('v_y_axis', 0, laneMaxSpeed)
            if len(self.ego.speedQ) >= 50:
                vx = list(range(-49, 1))
                vy = list(self.ego.speedQ)[-50:]
            else:
                vy = list(self.ego.speedQ)
                vx = list(range(-len(vy) + 1, 1))
            dpg.set_value('v_series_tag', [vx, vy])

        if self.ego.accelQ:
            if len(self.ego.accelQ) >= 50:
                ax = list(range(-49, 1))
                ay = list(self.ego.accelQ)[-50:]
            else:
                ay = list(self.ego.accelQ)
                ax = list(range(-len(ay) + 1, 1))
            dpg.set_value('a_series_tag', [ax, ay])

        if self.ego.plannedTrajectory:
            if self.ego.plannedTrajectory.velQueue:
                vfy = list(self.ego.plannedTrajectory.velQueue)
                vfx = list(range(1, len(vfy) + 1))
                dpg.set_value('v_series_tag_future', [vfx, vfy])
            if self.ego.plannedTrajectory.accQueue:
                afy = list(self.ego.plannedTrajectory.accQueue)
                afx = list(range(1, len(afy) + 1))
                dpg.set_value('a_series_tag_future', [afx, afy])
    # 将模拟结构信息插入数据库
    def putFrameInfo(self, vid: str, vtag: str, veh: Vehicle):
        self.dataQue.put(
            ('frameINFO',
             (self.timeStep, vid, vtag, veh.x, veh.y, veh.yaw, veh.speed,
              veh.accel, veh.laneID, veh.lanePos, veh.routeIdxQ[-1])))
    # 将车辆信息插入数据库
    def putVehicleInfo(self, vid: str, vtins: vehType, routes: str):
        self.dataQue.put(
            ('vehicleINFO', (vid, vtins.length, vtins.width, vtins.maxAccel,
                             vtins.maxDecel, vtins.maxSpeed, vtins.id, routes)))
    # 将评估信息插入数据库
    def putEvaluationInfo(self, points: np.ndarray):
        self.dataQue.put(
            ('evaluationINFO', tuple([self.timeStep] + points.tolist())))
    # 绘制场景
    def drawScene(self):
        ex, ey = self.ego.x, self.ego.y
        # 确保父节点存在且有效
        if dpg.does_item_exist("Canvas"):
            node = dpg.add_draw_node(parent="Canvas") # 创建Canvas的子图节点node，用于组织所有场景绘图
        else:
            print("Warning: Canvas parent node does not exist")
            return
        self.ms.plotScene(node, ex, ey, self.gui.ctf) # 绘制道路背景与静态元素
        self.ego.plotSelf('ego', node, ex, ey, self.gui.ctf) # 绘制自车模型（矩形+方向箭头）
        self.ego.plotdeArea(node, ex, ey, self.gui.ctf) # 绘制自车检测区域（蓝色半透明圆形）
        self.ego.plotTrajectory(node, ex, ey, self.gui.ctf) # 绘制自车轨迹（黄色线条）
        self.putFrameInfo(self.ego.id, 'ego', self.ego) # 将自车信息插入数据库
        if self.ms.vehINAoI: # 绘制在自车检测区域内的车辆
            for v1 in self.ms.vehINAoI.values():
                v1.plotSelf('AoI', node, ex, ey, self.gui.ctf)
                v1.plotTrajectory(node, ex, ey, self.gui.ctf)
                self.putFrameInfo(v1.id, 'AoI', v1)
        if self.ms.outOfAoI: # 绘制不在自车检测区域内的车辆
            for v2 in self.ms.outOfAoI.values():
                v2.plotSelf('outOfAoI', node, ex, ey, self.gui.ctf)
                v2.plotTrajectory(node, ex, ey, self.gui.ctf)
                self.putFrameInfo(v2.id, 'outOfAoI', v2)
        # 绘制宏观地图动态元素（movingScene节点）中的AOI-ego橙色半透明圆形
        if dpg.does_item_exist('movingScene'):
            mvNode = dpg.add_draw_node(parent='movingScene') 
            mvCenterx, mvCentery = self.mapCoordTF.dpgCoord(ex, ey) 
            dpg.draw_circle((mvCenterx, mvCentery),
                            self.ego.deArea * self.mapCoordTF.zoomScale,
                            thickness=0,
                            fill=(243, 156, 18, 60),
                            parent=mvNode) # 橙色半透明节点
        # 左下角的仿真信息文本（simInfo）
        if dpg.does_item_exist('simInfo'):
            infoNode = dpg.add_draw_node(parent='simInfo') 
            # 在infoNode节点上进行文本写作
            dpg.draw_text((5, 5),
                          'Real time simulation ego tracking.',
                          color=(75, 207, 250),
                          size=20,
                          parent=infoNode) # 绘制模拟信息
            dpg.draw_text((5, 25),
                          'Time step: %.2f s.' % (self.timeStep / 10),
                          color=(85, 230, 193),
                          size=20,
                          parent=infoNode)
            dpg.draw_text((5, 45),
                          'Current lane: %s' % self.ego.laneID,
                          color=(249, 202, 36),
                          size=20,
                          parent=infoNode)
            dpg.draw_text((5, 65),
                          'Lane position: %.5f' % self.ego.lanePos,
                          color=(249, 202, 36),
                          size=20,
                          parent=infoNode)
        # 评估窗口雷达图（sEvaluation窗口）
        points = self.evaluation.output_result() # 获取评估指标（偏移量、舒适度等）
        self.putEvaluationInfo(self.evaluation.result) # 将评估信息插入数据库
        transformed_points = self._evaluation_transform_coordinate(points,
                                                                   scale=30) # 转换坐标，将评估指标转换为绘图坐标
        transformed_points.append(transformed_points[0]) # 雷达图绘制需要闭合，添加第一个点以闭合图形
        if dpg.does_item_exist('radarPlot'):
            radarNode = dpg.add_draw_node(parent='radarPlot') # 创建雷达图，节点类别为radarNode，为已有节点类别radarPlot的子类
            dpg.draw_polygon(transformed_points,
                             color=(75, 207, 250, 100), # 雷达图轮廓颜色
                             fill=(75, 207, 250, 100), # 雷达图填充颜色
                             thickness=5, # 雷达图轮廓宽度
                             parent=radarNode) # 在radarNode上进行绘画
    # 评估信息坐标转换，为了将评估数据的极坐标转换为GUI界面中窗口的屏幕坐标系统，以在屏幕上进行图标绘制
    def _evaluation_transform_coordinate(self, points: List[float],
                                         scale: float) -> List[List[float]]:
        # 检查sEvaluation窗口是否存在
        if dpg.does_item_exist('sEvaluation'):
            dpgHeight = dpg.get_item_height('sEvaluation') - 30
            dpgWidth = dpg.get_item_width('sEvaluation') - 20
        else:
            # 如果sEvaluation窗口不存在，使用默认尺寸
            dpgHeight = 300 - 30
            dpgWidth = 300 - 20
            
        centerx = dpgWidth / 2
        centery = dpgHeight / 2

        transformed_points = []
        for j in range(5):
            transformed_points.append([
                centerx + scale * points[j] * cos(pi / 10 + 2 * pi * j / 5),
                dpgHeight -
                (centery + scale * points[j] * sin(pi / 10 + 2 * pi * j / 5))
            ])

        return transformed_points

    # 获取车辆类型信息
    def getvTypeIns(self, vtid: str) -> vehType:
        return self.allvTypes[vtid]
    
    # 获取车辆信息
    def getVehInfo(self, veh: Vehicle):
        vid = veh.id
        # 车辆存在性检查
        if vid not in traci.vehicle.getIDList():
            return
        if veh.vTypeID:
            max_decel = veh.maxDecel
        # 车辆确认存在
        else:
            vtypeid = traci.vehicle.getTypeID(vid) # 获取车辆类型ID
            if '@' in vtypeid:
                vtypeid = vtypeid.split('@')[0]
            vtins = self.getvTypeIns(vtypeid) # 获取veh对应的车辆类型及其包含的信息
            veh.maxAccel = vtins.maxAccel
            veh.maxDecel = vtins.maxDecel
            veh.length = vtins.length
            veh.width = vtins.width
            veh.maxSpeed = vtins.maxSpeed
            # veh.targetCruiseSpeed = random.random()
            veh.vTypeID = vtypeid
            veh.routes = traci.vehicle.getRoute(vid)
            veh.LLRSet, veh.LLRDict, veh.LCRDict = veh.getLaneLevelRoute(
                self.nb)

            routes = ' '.join(veh.routes)
            self.putVehicleInfo(vid, vtins, routes)
            max_decel = veh.maxDecel
        veh.yawAppend(traci.vehicle.getAngle(vid)) # 添加veh车辆偏航角
        x, y = traci.vehicle.getPosition(vid) # 获取veh车辆位置
        veh.xAppend(x) # 添加veh车辆x坐标
        veh.yAppend(y) # 添加veh车辆y坐标

        # veh.getStopInfo(veh.id)
        veh.speedQ.append(traci.vehicle.getSpeed(vid)) # 添加veh车辆速度
        if max_decel == traci.vehicle.getDecel(vid): # 如果车辆最大减速度等于当前减速度
            accel = traci.vehicle.getAccel(vid)
        else:
            accel = -traci.vehicle.getDecel(vid)
        veh.accelQ.append(accel)
        laneID = traci.vehicle.getLaneID(vid)
        veh.routeIdxAppend(laneID)
        veh.laneAppend(self.nb)

    def clear_message_files(self, traffic_manager, if_clear_message_file=False):
        """清理消息文件或清空消息内容
        
        Args:
            traffic_manager: TrafficManager实例
            if_clear_message_file (bool): 
                True表示删除消息文件，
                False表示清空消息文件内容
        """
        try:
            # 确保目录存在
            message_history_path = os.path.join("message_history", self.Scenario_Name)
            os.makedirs(message_history_path, exist_ok=True)
            
            # 只有在通信功能启用时才清理通信相关的消息文件
            if hasattr(traffic_manager, 'communication_manager') and traffic_manager.communication_manager:
                # 8.27 新增：清理消息文件or清理消息内容：
                if if_clear_message_file == True:
                    # 8.19 新增：删除所有消息历史文件
                    traffic_manager.communication_manager.cleanup_message_files()
                    # 8.27 新增：删除display_text文件
                    traffic_manager.communication_manager.cleanup_display_text(loc=message_history_path)
                else:
                    # 8.19 新增：清空所有消息历史文件内容
                    traffic_manager.communication_manager.clear_message_files_content()
                    # 8.27 新增：清空display_text文件里的内容
                    traffic_manager.communication_manager.clear_display_text_content(loc=message_history_path)
            else:
                # 通信功能未启用，只清理本地文件
                if if_clear_message_file:
                    # 删除本地消息文件
                    import glob
                    pattern = os.path.join(message_history_path, "message_*_history.txt")
                    files_to_remove = glob.glob(pattern)
                    for file_path in files_to_remove:
                        try:
                            os.remove(file_path)
                        except Exception as e:
                            print(f"删除文件 {file_path} 时出错: {e}")
                    
                    # 删除display_text文件
                    display_file = os.path.join(message_history_path, "display_text.txt")
                    if os.path.exists(display_file):
                        try:
                            os.remove(display_file)
                        except Exception as e:
                            print(f"删除文件 {display_file} 时出错: {e}")
                else:
                    # 清空本地消息文件内容
                    import glob
                    pattern = os.path.join(message_history_path, "message_*_history.txt")
                    files_to_clear = glob.glob(pattern)
                    for file_path in files_to_clear:
                        try:
                            with open(file_path, 'w') as f:
                                f.truncate(0)
                        except Exception as e:
                            print(f"清空文件 {file_path} 内容时出错: {e}")
                    
                    # 清空display_text文件内容
                    display_file = os.path.join(message_history_path, "display_text.txt")
                    if os.path.exists(display_file):
                        try:
                            with open(display_file, 'w', encoding='utf-8') as f:
                                f.truncate(0)
                        except Exception as e:
                            print(f"清空文件 {display_file} 内容时出错: {e}")
                            
        except Exception as e:
            print(f"Error in clear_message_files: {str(e)}")
            raise

    # 控制车辆下一步长的移动
    def vehMoveStep(self, veh: Vehicle):
        # 控制车辆在更新数据后移动
        # control vehicles after update its data
        # control happens next timestep
        if veh.plannedTrajectory and veh.plannedTrajectory.xQueue:
            centerx, centery, yaw, speed, accel, stop_flag = veh.plannedTrajectory.pop_last_state(
            ) 
            try:
                veh.controlSelf(centerx, centery, yaw, speed, accel, stop_flag) # 控制车辆移动 6.16:添加stop_flag
            except:
                return
        else:
            veh.exitControlMode()

    def updateVeh(self): # 更新车辆状态
        self.vehMoveStep(self.ego) #首先更新ego主车状态
        if self.ms.currVehicles: # 如果当前场景的周边车辆列表不为空
            for v in self.ms.currVehicles.values(): # 遍历当前车辆列表
                self.vehMoveStep(v) # 控制车辆移动

    def setTrajectories(self, trajectories: Dict[str, Trajectory]):
        for k, v in trajectories.items():
            if k == self.ego.id:
                self.ego.plannedTrajectory = v
            else:
                veh = self.ms.currVehicles[k]
                veh.plannedTrajectory = v

    def update_evluation_data(self):
        current_lane = self.nb.getLane(self.ego.laneID)
        if current_lane is None:
            current_lane = self.nb.getJunctionLane(self.ego.laneID)
        agents = list(self.ms.vehINAoI.values())
        self.evaluation.update_data(self.ego, current_lane, agents)

    def getSce(self):
        if self.ego.id in traci.vehicle.getIDList():
            self.tpStart = 1
            dpg.delete_item("Canvas", children_only=True)
            dpg.delete_item("movingScene", children_only=True)
            dpg.delete_item("simInfo", children_only=True)
            dpg.delete_item("radarPlot", children_only=True)
            self.ms.updateScene(self.dataQue, self.timeStep) # 更新获取的场景信息
            self.getVehInfo(self.ego) # 获取ego主车的信息
            self.updateVeh() # 更新车辆状态，确保laneIDQ等队列有值
            self.ms.updateSurroudVeh() # 定义了AOI内车辆、AOI外但是场景内车辆、场景外车辆
            if self.ms.currVehicles:
                for v in self.ms.currVehicles.values():
                    self.getVehInfo(v) # 获取场景内周边车辆的信息

            self.update_evluation_data()
            self.drawScene() # 绘制ATPSIP场景
            self.plotVState() # 绘制车辆状态曲线
        else:
            if self.tpStart:
                print('[cyan]The ego car has reached the destination.[/cyan]')
                # self.tpEnd = 1 

        # if self.tpStart:
        #     if self.ego.arriveDestination(self.nb): #到达终点arriveDestination是True，否则是False
        #         # self.tpEnd = 1
        #         print('[cyan]The ego car has reached the destination.[/cyan]')

    def exportSce(self):
        if self.tpStart:
            return self.ms.exportScene()
        else:
            return None, None

    # 在GUI的sEvaluation窗口绘制雷达背景框架图
    def drawRadarBG(self):
        # 检查radarBackground是否存在
        if not dpg.does_item_exist('radarBackground'):
            return
            
        bgNode = dpg.add_draw_node(parent='radarBackground')
        # eliminate the bias
        # 检查sEvaluation窗口是否存在
        if dpg.does_item_exist('sEvaluation'):
            dpgHeight = dpg.get_item_height('sEvaluation') - 30
            dpgWidth = dpg.get_item_width('sEvaluation') - 20
        else:
            # 如果sEvaluation窗口不存在，使用默认尺寸
            dpgHeight = 300 - 30
            dpgWidth = 300 - 20
            
        centerx = dpgWidth / 2
        centery = dpgHeight / 2
        for i in range(4):
            dpg.draw_circle(center=[centerx, centery],
                            radius=30 * (i + 1),
                            color=(223, 230, 233),
                            parent=bgNode)

        radarLabels = [
            "offset", "discomfort", "collision", "orientation",
            "consumption"
        ]
        offset = np.array([[-0.3, 0.2], [-2.2, 0.3], [-2.3, 0.2], [-2.8, 0.5],
                           [-0.1, 0.5]]) * 30

        axis_points = self._evaluation_transform_coordinate([4, 4, 4, 4, 4],
                                                            scale=30)
        text_points = self._evaluation_transform_coordinate([1, 1, 1, 1, 1],
                                                            scale=140)
        for j in range(5):
            dpg.draw_line(
                [centerx, centery],
                axis_points[j],
                color=(223, 230, 233),
                parent=bgNode,
            )

            dpg.draw_text([
                text_points[j][0] + offset[j][0],
                text_points[j][1] - offset[j][1]
            ],
                          text=radarLabels[j],
                          size=20,
                          parent=bgNode)

    def drawMapBG(self):
        # left-bottom: x1, y1
        # top-right: x2, y2
        ((x1, y1), (x2, y2)) = traci.simulation.getNetBoundary()
        netBoundary = f"{x1},{y1} {x2},{y2}"
        conn = sqlite3.connect("Database/" + self.dataBase)
        cur = conn.cursor()
        cur.execute(f"""UPDATE simINFO SET netBoundary = '{netBoundary}';""")
        conn.commit()
        conn.close()
        self.mapCoordTF = MapCoordTF((x1, y1), (x2, y2), 'macroMap')
        mNode = dpg.add_draw_node(parent='mapBackground')
        for jid in self.nb.junctions.keys():
            self.nb.plotMapJunction(jid, mNode, self.mapCoordTF)

        self.gui.drawMainWindowWhiteBG((x1-100, y1-100), (x2+100, y2+100))
    
    def render(self):
        self.gui.update_inertial_zoom() # 更新 inertial zoom（未知）
        self.getSce() # 获取ATPSIP场景
        dpg.render_dearpygui_frame() # 渲染dearpygui框架

    def moveStep(self):
        if self.gui.is_running and self.timeStep < self.max_steps:
            traci.simulationStep() 
            # 7.15：[target]display函数的更新迭代：展示AOI内所有车辆此时刻的信息发出和接受信息
            self.timeStep += 1
            # 7.20：获取所有车辆ID，实例化车辆列表
            # 只在必要时更新车辆列表
            if not self.vehicles or self.timeStep % 10 == 0:
                self.vehicles=self.getVehicleList()
        elif self.timeStep >= self.max_steps: # 如果模拟步长达到最大步长
            self.tpEnd = 1 # 设置模拟结束标志
        if not dpg.is_dearpygui_running(): # 如果dearpygui未运行
            self.tpEnd = 1 # 设置模拟结束标志
        if self.ego.id in traci.vehicle.getIDList(): # 如果自车在场景中
            if not self.tpStart: # 如果模拟未开始
                self.gui.start() # 启动dearpygui
                self.drawRadarBG() # 绘制雷达背景   
                self.drawMapBG() # 绘制地图背景
                self.tpStart = 1 # 设置模拟开始标志
            self.render() # 渲染仿真界面场景

    def destroy(self):
        # stop the saveThread.
        time.sleep(1.1)
        traci.close()
        self.gui.destroy()
