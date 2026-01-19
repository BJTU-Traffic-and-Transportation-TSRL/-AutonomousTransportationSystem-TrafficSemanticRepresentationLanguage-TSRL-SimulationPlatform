import traci
from traci import TraCIException
from math import sqrt, pow
from queue import Queue
import dearpygui.dearpygui as dpg
import sqlite3


from simModel.common.networkBuild import NetworkBuild, Rebuild
from simModel.common.carFactory import Vehicle, egoCar, DummyVehicle
from simModel.common.facilitiesFactory import RSU
from utils.roadgraph import RoadGraph
from utils.simBase import CoordTF

from read_stop_info import assign_stops_to_vehicles

class MovingScene:
    def __init__(self, netInfo: NetworkBuild, ego: egoCar,vehicles_with_stops=None) -> None:
        self.netInfo = netInfo
        self.ego = ego
        self.edges: set = None
        self.junctions: set = None
        self.RSUs: dict[str, RSU] = {}
        self.rsuInAoI: dict[str, RSU] = {}  # 9.12 AOI内的RSU集合
        self.currVehicles: dict[str, Vehicle] = {}
        self.vehINAoI: dict[str, Vehicle] = {}
        self.outOfAoI: dict[str, Vehicle] = {}
        self.vehicles_with_stops = vehicles_with_stops  # 7.27添加停车信息

    # if lane-lenght <= the self.ego's deArea, return current edge, current
    # edge's upstream intersection and current edge's downstream intersection.
    # else, judge if the upstream intersection or downstream intersection
    # is in the range of the vehicle's deArea.
    def updateScene(self, dataQue: Queue, timeStep: int):
        # 9.7 添加更新RSUs的部分
        ex, ey = traci.vehicle.getPosition(self.ego.id) # 获取ego主车的位置
        currGeox = int(ex // 100)
        currGeoy = int(ey // 100)
        # 获取当前场景中的geohash
        sceGeohashIDs = (
            (currGeox-1, currGeoy-1),
            (currGeox, currGeoy-1),
            (currGeox+1, currGeoy-1),
            (currGeox-1, currGeoy),
            (currGeox, currGeoy),
            (currGeox+1, currGeoy),
            (currGeox-1, currGeoy+1),
            (currGeox, currGeoy+1),
            (currGeox+1, currGeoy+1),
        )

        NowEdges: set = set()
        NowJuncs: set = set()
        NowRSUs = set()

        for sgh in sceGeohashIDs:
            try:
                geohash = self.netInfo.geoHashes[sgh]
            except KeyError:
                continue
            NowEdges = NowEdges | geohash.edges
            NowJuncs = NowJuncs | geohash.junctions
            NowRSUs |= geohash.rsus

        self.edges = NowEdges
        self.junctions = NowJuncs
        # self.RSUs = NowRSUs
        self.RSUs = {rsu_id: self.netInfo.getRSU(rsu_id) for rsu_id in NowRSUs}
        
        NowTLs = {}
        for jid in NowJuncs:
            junc = self.netInfo.getJunction(jid)
            for jlid in junc.JunctionLanes:
                jl = self.netInfo.getJunctionLane(jlid)
                tlid = jl.tlLogic
                if tlid:
                    if tlid not in NowTLs.keys():
                        currPhaseIndex = traci.trafficlight.getPhase(tlid)
                        tlLogic = self.netInfo.getTlLogic(tlid)
                        currPhase = tlLogic.currPhase(currPhaseIndex)
                        nextPhase = tlLogic.nextPhase(currPhaseIndex)
                        switchTime = round(traci.trafficlight.getNextSwitch(
                            tlid) - traci.simulation.getTime(), 1)
                        NowTLs[tlid] = (currPhase, nextPhase, switchTime)
                        dataQue.put((
                            'trafficLightStates',
                            (timeStep, tlid, currPhase, nextPhase, switchTime)
                        ))
                    else:
                        currPhase, nextPhase, switchTime = NowTLs[tlid]
                    jl.currTlState = currPhase[jl.tlsIndex]
                    jl.nexttTlState = nextPhase[jl.tlsIndex]
                    jl.switchTime = switchTime

    def addVeh(self, vdict: dict, vid: str) -> None:
        if vdict and vid in vdict.keys():
            return
        else:
            vehIns = Vehicle(vid)
            vdict[vid] = vehIns

    # getSurroundVeh will update all vehicle's attributes
    # so don't update again in other steps
    def updateSurroudVeh(self):
        nextStepVehicles = set() # 下一个时间步的车辆集合
        for ed in self.edges: # 遍历所有的边
            nextStepVehicles = nextStepVehicles | set(
                traci.edge.getLastStepVehicleIDs(ed) 
            ) # 获取该边上的所有车辆id，并入下一个时间步的车辆集合

        for jc in self.junctions: # 遍历所有的路口
            jinfo = self.netInfo.getJunction(jc)
            if jinfo.JunctionLanes:
                for il in jinfo.JunctionLanes:
                    nextStepVehicles = nextStepVehicles | set(
                        traci.lane.getLastStepVehicleIDs(il)
                    ) # 获取该路口所有道路的车辆id，并入下一个时间步的车辆集合

        newVehicles = nextStepVehicles - self.currVehicles.keys() # 新加入场景的车辆集合：当前帧有但是上一帧没有的车辆
        for nv in newVehicles:
            self.addVeh(self.currVehicles, nv)

        ex, ey = traci.vehicle.getPosition(self.ego.id) # 获取ego主车的位置
        vehInAoI = {} # 当前帧在aoi内的车辆集合
        outOfAoI = {} # 当前帧不在aoi内的车辆集合
        outOfRange = set() # 当前帧超出监控范围的车辆集合
        
        # 检测AOI内的RSU
        rsuInAoI = {}  # 9.12 AOI内的RSU集合
        for rsu_id, rsu in self.RSUs.items():
            if rsu and self.ego.laneID:
               if rsu.isInAoI(self.ego.laneID,self.ego.lanePos,self.ego.deArea):
                rsuInAoI[rsu_id] = rsu
            else:
                continue
        
        for vk, vv in self.currVehicles.items(): #vk: 车辆id, vv: 车辆实例
            if vk == self.ego.id:
                continue
            try:
                x, y = traci.vehicle.getPosition(vk) # 获取周围车辆的位置
            except TraCIException: # 捕获异常，说明车辆已经离开网络
                # vehicle is leaving the network.
                outOfRange.add((vk, 0))
                continue
            if sqrt(pow((ex - x), 2) + pow((ey - y), 2)) <= self.ego.deArea: # 如果某周围车辆在ego主车的aoi内
                try:
                    vehArrive = vv.arriveDestination(self.netInfo)
                except:
                    vehArrive = False

                if vehArrive:
                    outOfRange.add((vk, 1))
                else:
                    vehInAoI[vk] = vv
            elif sqrt(pow((ex - x), 2) + pow((ey - y), 2)) <= 2 * self.ego.deArea:
                try:
                    vehArrive = vv.arriveDestination(self.netInfo)
                except:
                    vehArrive = False

                if vehArrive:
                    outOfRange.add((vk, 1))
                else:
                    outOfAoI[vk] = vv
            else:
                vv.exitControlMode()
                outOfRange.add((vk, 0))

        for vid, atag in outOfRange:
            del(self.currVehicles[vid])
            # if the vehicle arrived the destination, remove it from the simulation
            if atag:
                try:
                    traci.vehicle.remove(vid)
                except TraCIException:
                    pass

        self.vehINAoI = vehInAoI
        self.outOfAoI = outOfAoI
        self.rsuInAoI = rsuInAoI  # 更新AOI内的RSU集合
        # 7.27 同步停车信息
        if self.vehicles_with_stops:
            assign_stops_to_vehicles(self.vehicles_with_stops, self.currVehicles)
    
    # 绘制ATPSIP场景
    def plotScene(self, node: dpg.node, ex: float, ey: float, ctf: CoordTF):
        if self.edges:
            # 绘制道路
            for ed in self.edges:
                self.netInfo.plotEdge(ed, node, ex, ey, ctf)

        if self.junctions:
            # 绘制路口
            for jc in self.junctions:
                self.netInfo.plotJunction(jc, node, ex, ey, ctf)
        # 9.5 添加绘制RSU方法
        if self.RSUs:
            # 绘制RSU
            for rsu in self.RSUs.values():
                self.netInfo.plotRSU(rsu.id, node, ex, ey, ctf)

    def exportScene(self):
        roadgraph = RoadGraph()

        for eid in self.edges:
            Edge = self.netInfo.getEdge(eid)
            roadgraph.edges[eid] = Edge
            for lane in Edge.lanes:
                roadgraph.lanes[lane] = self.netInfo.getLane(lane)

        for junc in self.junctions:
            Junction = self.netInfo.getJunction(junc)

            for jl in Junction.JunctionLanes:
                juncLane = self.netInfo.getJunctionLane(jl)
                roadgraph.junction_lanes[juncLane.id] = juncLane

        # export vehicles' information using dict.
        vehicles = {
            'egoCar': self.ego.export2Dict(self.netInfo), #自车
            'carInAoI': [av.export2Dict(self.netInfo) for av in self.vehINAoI.values()] ,# AOI内的车
            'outOfAoI': [sv.export2Dict(self.netInfo) for sv in self.outOfAoI.values()] # AOI外的车
        }       

        # 9.12 添加RSU信息到导出数据中
        if self.RSUs:
            facilities = {
                'rsuInAoI': [rsu.export2Dict(self.netInfo) for rsu in self.rsuInAoI.values()]  # AOI内的RSU
            }
        else:
            facilities = {}

        return roadgraph, vehicles, facilities


class SceneReplay:
    def __init__(self, netInfo: Rebuild, ego: egoCar) -> None:
        self.netInfo = netInfo
        self.ego = ego
        self.currVehicles: dict[str, Vehicle] = {}
        self.vehINAoI: dict[str, Vehicle] = {}
        self.outOfAoI: dict[str, Vehicle] = {}
        self.outOfRange = set()
        self.edges: set = None
        self.junctions: set = None
        self.RSUs: dict[str, RSU] = {}  # 9.12 存储当前范围内的RSU
        self.rsuInAoI: dict[str, RSU] = {}  # 9.12 AOI内的RSU集合

    def updateScene(self, dataBase: str, timeStep: int):
        ex, ey = self.ego.x, self.ego.y
        currGeox = int(ex // 100)
        currGeoy = int(ey // 100)

        sceGeohashIDs = (
            (currGeox-1, currGeoy-1),
            (currGeox, currGeoy-1),
            (currGeox+1, currGeoy-1),
            (currGeox-1, currGeoy),
            (currGeox, currGeoy),
            (currGeox+1, currGeoy),
            (currGeox-1, currGeoy+1),
            (currGeox, currGeoy+1),
            (currGeox+1, currGeoy+1),
        )

        NowEdges: set = set()
        NowJuncs: set = set()
        NowRSUs: set = set()  # 9.6 新增：存储当前范围内的RSU

        for sgh in sceGeohashIDs:
            try:
                geohash = self.netInfo.geoHashes[sgh]
            except KeyError:
                continue
            NowEdges = NowEdges | geohash.edges
            NowJuncs = NowJuncs | geohash.junctions
            NowRSUs = NowRSUs | geohash.rsus  # 9.6 新增：获取RSU

        self.edges = NowEdges
        self.junctions = NowJuncs
        # 更新RSU信息
        self.RSUs = {rsu_id: self.netInfo.getRSU(rsu_id) for rsu_id in NowRSUs}

        NowTLs = {}
        conn = sqlite3.connect(dataBase)
        cur = conn.cursor()
        cur.execute(
            '''SELECT * FROM trafficLightStates WHERE frame=%i;''' % timeStep)
        tlsINFO = cur.fetchall()
        if tlsINFO:
            for tls in tlsINFO:
                frame, tlid, currPhase, nextPhase, switchTime = tls
                NowTLs[tlid] = (currPhase, nextPhase, switchTime)

        cur.close()
        conn.close()

        if NowTLs:
            for jid in NowJuncs:
                junc = self.netInfo.getJunction(jid)
                if junc:
                    for jlid in junc.JunctionLanes:
                        jl = self.netInfo.getJunctionLane(jlid)
                        tlid = jl.tlLogic
                        if tlid:
                            try:
                                currPhase, nextPhase, switchTime = NowTLs[tlid]
                            except KeyError:
                                continue
                            jl.currTlState = currPhase[jl.tlsIndex]
                            jl.nexttTlState = nextPhase[jl.tlsIndex]
                            jl.switchTime = switchTime

    def updateSurroudVeh(self):
        outOfRange = set()
        for vid in self.outOfRange:
            try:
                del (self.currVehicles[vid])
            except KeyError:
                pass
            self.outOfRange = set()

        ex, ey = self.ego.x, self.ego.y
        vehInAoI = {}
        outOfAoI = {}
        
        # 9.12 检测AOI内的RSU
        rsuInAoI = {}  # AOI内的RSU集合
        # 9.12 用于跟踪新进入AOI的RSU
        new_rsus_in_aoi = {}
        for rsu_id, rsu in self.RSUs.items():
            if rsu and rsu.isInAoI(ex, ey, self.ego.deArea):
                rsuInAoI[rsu_id] = rsu
                # 9.12 检查是否是新进入AOI的RSU
                if rsu_id not in self.rsuInAoI:
                    new_rsus_in_aoi[rsu_id] = rsu
        
        for vk, vv in self.currVehicles.items():
            if vk == self.ego.id:
                continue
            x, y = vv.x, vv.y
            if sqrt(pow((ex - x), 2) + pow((ey - y), 2)) <= self.ego.deArea:
                try: 
                    vehArrive = vv.arriveDestination(self.netInfo)
                except:
                    vehArrive = False

                if vehArrive:
                    outOfRange.add((vk, 1))
                else:
                    vehInAoI[vk] = vv
            else:
                outOfAoI[vk] = vv

        for vid, atag in outOfRange:
            del(self.currVehicles[vid])
            # if the vehicle arrived the destination, remove it from the simulation
            if atag:
                try:
                    del(self.currVehicles[vid])
                except KeyError:
                    pass

        self.vehINAoI = vehInAoI
        self.outOfAoI = outOfAoI
        self.rsuInAoI = rsuInAoI  #9.12  更新AOI内的RSU集合
        
        # 如果有新进入AOI的RSU，触发通信
        if new_rsus_in_aoi and hasattr(self.ego, 'communicator') and self.ego.communicator:
            for rsu_id, rsu in new_rsus_in_aoi.items():
                # RSU向Ego车辆发送消息
                content = f"RSU {rsu_id} entered AoI"
                self.ego.communicator.send(content, target_id=self.ego.id)
        
        # 7.27 同步停车信息
        if hasattr(self, 'vehicles_with_stops'):
            assign_stops_to_vehicles(self.vehicles_with_stops, self.currVehicles)

    def exportScene(self):
        roadgraph = RoadGraph()

        for eid in self.edges:
            Edge = self.netInfo.getEdge(eid)
            roadgraph.edges[eid] = Edge
            for lane in Edge.lanes:
                roadgraph.lanes[lane] = self.netInfo.getLane(lane)

        for junc in self.junctions:
            Junction = self.netInfo.getJunction(junc)
            for jl in Junction.JunctionLanes:
                juncLane = self.netInfo.getJunctionLane(jl)
                roadgraph.junction_lanes[juncLane.id] = juncLane

        # export vehicles' information using dict.
        vehicles = {
            'egoCar': self.ego.export2Dict(self.netInfo),
            'carInAoI': [av.export2Dict(self.netInfo) for av in self.vehINAoI.values()],
            'outOfAoI': [sv.export2Dict(self.netInfo) for sv in self.outOfAoI.values()]
        }

        # 添加RSU信息到导出数据中，修复返回值名称以匹配ForwardCollisionWarning.py中的期望
        facilities = {
            'rsuInAoI': [rsu.export2Dict(self.netInfo) for rsu in self.rsuInAoI.values()]  # AOI内的RSU
        }

        return roadgraph, vehicles, facilities

    def plotScene(self, node: dpg.node, ex: float, ey: float, ctf: CoordTF):
        if self.edges:
            for ed in self.edges:
                self.netInfo.plotEdge(ed, node, ex, ey, ctf)

        if self.junctions:
            for jc in self.junctions:
                self.netInfo.plotJunction(jc, node, ex, ey, ctf)
                
        # 9.12 绘制RSU
        if self.RSUs:
            for rsu in self.RSUs.values():
                self.netInfo.plotRSU(rsu.id, node, ex, ey, ctf)
