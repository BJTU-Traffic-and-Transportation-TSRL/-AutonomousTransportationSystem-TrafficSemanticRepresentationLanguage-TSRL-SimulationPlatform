"""
功能：创建LimSim仿真平台上的车辆(偏向可视化)
Vehicle：
    - 仿真基础 ：提供SUMO仿真环境中的基本车辆功能
    - 数据存储 ：存储车辆的历史轨迹数据（位置、速度、加速度等）
    - SUMO接口 ：直接与SUMO/traci交互
    - 路径管理 ：处理车辆的路径规划和车道选择
    - 可视化支持 ：支持在GUI中绘制车辆和轨迹
"""

from __future__ import annotations

from collections import deque
from math import cos, pi, sin
from collections import defaultdict

import dearpygui.dearpygui as dpg
from rich import print
import numpy as np
import traci
from traci import TraCIException

from simModel.common.networkBuild import NetworkBuild, Rebuild
from utils.simBase import CoordTF, deduceEdge
from utils.trajectory import Trajectory
from utils.roadgraph import NormalLane, JunctionLane

class Vehicle:# 定义车辆类别
    def __init__(self, id: str) -> None:
        self.id = id
        # store the last 10[s] x position for scenario rebuild
        # x, y: position(two doubles) of the named vehicle (center) within the last step
        self.xQ = deque(maxlen=100) # 存储车辆x坐标
        self.yQ = deque(maxlen=100) # 存储车辆y坐标
        self.yawQ = deque(maxlen=100) # 存储车辆航向角
        self.speedQ = deque(maxlen=100) # 存储车辆速度
        self.accelQ = deque(maxlen=100) # 存储车辆加速度
        self.laneIDQ = deque(maxlen=100) # 存储车辆车道ID
        # lanePos: The position of the vehicle along the lane (the distance
        # from the center of the car to the start of the lane in [m])
        self.lanePosQ = deque(maxlen=100) # 存储车辆车道位置
        self.routeIdxQ = deque(maxlen=100) # 存储车辆路径索引
        self.routes: list[str] = None # 存储车辆路径
        self.LLRSet: set[str] = None # 存储车辆车道集合
        self.LLRDict: dict[str, dict[str, set[str]]] = None # 存储车辆车道字典
        self.LCRDict: dict[str, str] = None # 存储车辆车道对应路径索引
        self.length: float = 5.0   # SUMO默认值，车辆长度
        self.width: float = 1.8   # SUMO默认值，车辆宽度
        self.maxAccel: float = 3.0   # SUMO默认值，车辆最大加速度
        self.maxDecel: float = 8.0  # SUMO默认值，车辆最大减速度
        self.maxSpeed: float = 13.89 # SUMO默认值，车辆最大速度
        self.vTypeID: str = None # 存储车辆类型ID
        self._iscontroled: bool = 0 # 存储车辆是否受控
        self.lookForward: float = 100 # 存储车辆前瞻距离
        self.noChange: float = 5.0 # 存储车辆不换道距离
        self.plannedTrajectory: Trajectory = None # 存储车辆计划轨迹
        self.dbTrajectory: Trajectory = None # 存储车辆数据库轨迹
        self.stop_info = []  # 7.20：添加单车停车信息存储列表

    # LLR: lane-level route
    # 获取车道级别路径
    def getLaneLevelRoute(self, nb: NetworkBuild) -> tuple[set, dict]:
        LLRSet: set[str] = set()
        LLRDict: dict[str, dict[str, set[str]]] = {}
        for i in range(len(self.routes)-1):
            eid = self.routes[i]
            LLRDict[eid] = {}
            edgeIns = nb.getEdge(eid)
            LLRSet = LLRSet | edgeIns.lanes
            LLRDict[eid]['edgeLanes'] = edgeIns.lanes
            nextEid = self.routes[i+1]
            changeLanes = edgeIns.next_edge_info[nextEid]
            LLRDict[eid]['changeLanes'] = changeLanes
            nextEdge = nb.getEdge(nextEid)
            changeJuncLanes = set()
            for targetLane in nextEdge.lanes:
                for cl in changeLanes:
                    clIns = nb.getLane(cl)
                    try:
                        changeJuncLanes.add(clIns.next_lanes[targetLane][0])
                    except KeyError:
                        pass

            LLRSet = LLRSet | changeJuncLanes
            LLRDict[eid]['junctionLanes'] = changeJuncLanes

        lastEdgeID = self.routes[-1]
        lastEdgeIns = nb.getEdge(lastEdgeID)
        LLRSet = LLRSet | lastEdgeIns.lanes
        LLRDict[lastEdgeID] = {
            'edgeLanes': lastEdgeIns.lanes
        }

        # Lane corresponded route
        LCRDict: dict[str, list[int]] = defaultdict(list)
        for i in range(len(self.routes)):
            eid = self.routes[i]
            edge = nb.getEdge(eid)
            for lid in edge.lanes:
                LCRDict[lid].append(i)
            nextJunction = nb.getJunction(edge.to_junction)
            for jlid in nextJunction.JunctionLanes:
                LCRDict[jlid].append(i)

        return LLRSet, LLRDict, LCRDict

    @property
    def iscontroled(self):
        return self._iscontroled

    @property
    def yaw(self): 
        if self.yawQ:
            return self.yawQ[-1]
        else:
            raise TypeError('Please call Model.updateVeh() at first.')

    # return center x of the vehicle
    @property
    def x(self):
        if self.xQ:
            return self.xQ[-1]
        else:
            raise TypeError('Please call Model.updateVeh() at first.')

    # return center y of the vehicle
    @property
    def y(self):
        if self.yQ:
            return self.yQ[-1]
        else:
            raise TypeError('Please call Model.updateVeh() at first.')

    @property
    def speed(self):
        if self.speedQ:
            return self.speedQ[-1]
        else:
            raise TypeError('Please call Model.updateVeh() at first.')

    @property
    def accel(self):
        if self.accelQ:
            return self.accelQ[-1]
        else:
            raise TypeError('Please call Model.updateVeh() at first.')

    @property
    def laneID(self) -> str:
        if self.laneIDQ:
            return self.laneIDQ[-1]
        else:
            raise TypeError('Please call Model.updateVeh() at first.')

    @property
    def lanePos(self) -> float:
        if self.lanePosQ:
            return self.lanePosQ[-1]
        else:
            raise TypeError('Please call Model.updateVeh() at first.')

    @property
    def edgeID(self) -> str:
        if self.routeIdxQ:
            currIdx = self.routeIdxQ[-1]
            currEdge = self.routes[currIdx]
            if currIdx < len(self.routes) - 1:
                searchLanes = self.LLRDict[currEdge]['edgeLanes'] | self.LLRDict[currEdge]['junctionLanes']
            else:
                searchLanes = self.LLRDict[currEdge]['edgeLanes']
            if self.laneID in searchLanes:
                return currEdge
            else:
                if currIdx !=0:
                    return self.routes[currIdx-1]
                else:
                    return currEdge
        else:
            raise TypeError('Please call Model.updateVeh() at first.')

    @property
    def nextEdgeID(self) -> str:
        currIdx = self.routeIdxQ[-1]
        currEdge = self.routes[currIdx]
        if currIdx < len(self.routes) - 1:
            searchLanes = self.LLRDict[currEdge]['edgeLanes'] | self.LLRDict[currEdge]['junctionLanes']
        else:
            searchLanes = self.LLRDict[currEdge]['edgeLanes']
        if self.laneID in searchLanes:
            if currIdx < len(self.routes) - 1:
                return self.routes[currIdx+1]
            else:
                return 'Destination edge'
        else:
            return self.routes[currIdx]
    # 判断车辆是否到达目的地：输出是或否
    def arriveDestination(self, nb: NetworkBuild | Rebuild) -> bool:
        nextEdge = self.nextEdgeID
        if nextEdge == 'Destination edge':
            try:
                desLength = nb.getLane(self.laneID).sumo_length
            except AttributeError:
                return False
            if self.lanePos > desLength - 10:
                return True
            else:
                return False
        else:
            return False
    # 获取车辆可用车道：输出车道集合
    def availableLanes(self, nb: NetworkBuild):
        if ':' in self.laneID:
            if self.nextEdgeID == 'Destination edge':
                return self.LLRDict[self.edgeID]['edgeLanes']
            else:
                output = set()
                output = output | self.LLRDict[self.edgeID]['changeLanes']
                output = output | self.LLRDict[self.edgeID]['junctionLanes']
                output = output | self.LLRDict[self.nextEdgeID]['edgeLanes']
                return output
        else:
            if self.nextEdgeID == 'Destination edge':
                return self.LLRDict[self.edgeID]['edgeLanes']
            laneLength = nb.getLane(self.laneID).sumo_length
            if laneLength < self.lookForward:
                if self.lanePos < 5:
                    return self.LLRDict[self.edgeID]['edgeLanes']
                else:
                    output = set()
                    output = output | self.LLRDict[self.edgeID]['changeLanes']
                    output = output | self.LLRDict[self.edgeID]['junctionLanes']
                    return output
            remainDis = laneLength - self.lanePos
            if remainDis > max(laneLength / 3, self.lookForward):
                return self.LLRDict[self.edgeID]['edgeLanes']
            else:
                output = set()
                output = output | self.LLRDict[self.edgeID]['changeLanes']
                output = output | self.LLRDict[self.edgeID]['junctionLanes']
                return output
    
    # 7.20：设置车辆的停车信息
    def set_stop_info(self, stops):
        """设置车辆的停车信息"""
        self.stop_info = stops
    
    # 7.20：应用车辆的停车信息
    def apply_stop_info(self):
        """应用车辆的停车信息"""
        for stop in self.stop_info:
            lane_parts = stop['lane'].split('_')
            edge_id = lane_parts[0]
            lane_index = int(lane_parts[1]) if len(lane_parts) > 1 else 0
            try:
                traci.vehicle.setStop(
                    vehID=self.id,
                    edgeID=edge_id,
                    pos=stop['end_pos'],
                    laneIndex=lane_index,
                    until=stop['until']
                )
                print(f"手动应用停车信息：车辆{self.id}, 目标停车道路{edge_id}, 停车位置{stop['end_pos']}, 目标车道{lane_index}, 目标时间{stop['until']}")
            except Exception as e:
                print(f"应用停车信息失败：{str(e)}")
    # entry control mode and control vehicles
    # used for real-time simulation mode.
    # 进入控制模式并控制车辆
    # 用于实时仿真模式
    def controlSelf(
        self, centerx: float, centery: float,
        yaw: float, speed: float, accel: float, stop_flag: bool
    ):
        x = centerx + (self.length / 2) * cos(yaw)
        y = centery + (self.length / 2) * sin(yaw)
        angle = (pi / 2 - yaw) * 180 / pi
        
        if self._iscontroled:
            traci.vehicle.moveToXY(self.id, '', -1, x, y,
                                   angle=angle, keepRoute=2)
            traci.vehicle.setSpeed(self.id, speed)
            if accel >= 0: # 如果车辆加速度大于等于0
                traci.vehicle.setAccel(self.id, accel)
                traci.vehicle.setDecel(self.id, self.maxDecel)
            else:
                traci.vehicle.setAccel(self.id, self.maxAccel)
                traci.vehicle.setDecel(self.id, -accel)
            # 7.17: 当stop_flag为True时触发紧急停车流程
            # 尝试获取车辆的停止状态
            # print(traci.vehicle.getStopState(self.id)) 

            if stop_flag:
                self.emergency_stop()  # 7.17：调用紧急停车方法
        else:
            traci.vehicle.setLaneChangeMode(self.id, 0)
            traci.vehicle.setSpeedMode(self.id, 0)
            traci.vehicle.moveToXY(self.id, '', -1, x, y,
                                   angle=angle, keepRoute=2)
            traci.vehicle.setSpeed(self.id, speed)
            if accel >= 0: # 如果车辆加速度大于等于0
                traci.vehicle.setAccel(self.id, accel)
                traci.vehicle.setDecel(self.id, self.maxDecel)
            else:
                traci.vehicle.setAccel(self.id, self.maxAccel)
                traci.vehicle.setDecel(self.id, -accel)
        
        self._iscontroled = 1
    # exit control mode and set self.iscontroled = 0
    # 退出控制模式并设置self.iscontroled = 0
    def exitControlMode(self):
        if self._iscontroled:
            try:
                traci.vehicle.setLaneChangeMode(self.id, 0b101010101010)
                traci.vehicle.setSpeedMode(self.id, 0b010111)
                traci.vehicle.setSpeed(self.id, 20)
            except TraCIException:
                pass
            self._iscontroled = 0

    # 重放更新
    def replayUpdate(self):
        # if plannedTrajectory and dbTrajectory are both empty, return 'Failure',
        # else, return 'Success'.
        # 如果plannedTrajectory和dbTrajectory都为空，返回'Failure'，否则返回'Success'
        if self.plannedTrajectory and self.plannedTrajectory.xQueue:
            x, y, yaw, speed, accel, laneID, lanePos, _ = \
                self.plannedTrajectory.pop_last_state_r()
            self._iscontroled = 1
        elif self.dbTrajectory and self.dbTrajectory.xQueue:
            x, y, yaw, speed, accel, laneID, lanePos, routeIdx = \
                self.dbTrajectory.pop_last_state_r()
        else:
            return 'Failure'
        self.xQ.append(x)
        self.yQ.append(y)
        self.yawQ.append(yaw)
        self.speedQ.append(speed)
        self.accelQ.append(accel)
        self.laneIDQ.append(laneID)
        self.lanePosQ.append(lanePos)
        if ':' not in laneID:
            edge = deduceEdge(laneID)
            self.routeIdxQ.append(self.routes.index(edge))
        else:
            if self.routeIdxQ:
                self.routeIdxQ.append(self.routeIdxQ[-1])
            else:
                self.routeIdxQ.append(routeIdx)
        return 'Success'

    # 导出车辆信息to字典
    def export2Dict(self, nb: NetworkBuild | Rebuild) -> dict:
        return {
            'id': self.id, 'vTypeID': self.vTypeID,
            'xQ': self.xQ, 'yQ': self.yQ, 'yawQ': self.yawQ,
            'speedQ': self.speedQ, 'accelQ': self.accelQ,
            'laneIDQ': self.laneIDQ, 'lanePosQ': self.lanePosQ,
            'availableLanes': self.availableLanes(nb),
            'stop_info': self.stop_info,  # 7.21 添加停车信息
        }

    # 绘制车辆
    def plotSelf(self, vtag: str, node: dpg.node, ex: float, ey: float, ctf: CoordTF):
        if not self.xQ:
            raise TypeError('Please call Model.updateVeh() at first.')
        # 旋转矩阵
        rotateMat = np.array(
            [
                [cos(self.yaw), -sin(self.yaw)],
                [sin(self.yaw), cos(self.yaw)]
            ]
        )
        # 车辆顶点
        vertexes = [
            np.array([[self.length/2], [self.width/2]]),
            np.array([[self.length/2], [-self.width/2]]),
            np.array([[-self.length/2], [-self.width/2]]),
            np.array([[-self.length/2], [self.width/2]])
        ]
        # 旋转顶点
        rotVertexes = [np.dot(rotateMat, vex) for vex in vertexes]
        # 相对顶点
        relativeVex = [[self.x+rv[0]-ex, self.y+rv[1]-ey]
                       for rv in rotVertexes]
        drawVex = [
            [
                ctf.zoomScale*(ctf.drawCenter+rev[0]+ctf.offset[0]),
                ctf.zoomScale*(ctf.drawCenter-rev[1]+ctf.offset[1])
            ] for rev in relativeVex
        ]
        if vtag == 'ego':
            vcolor = (211, 84, 0)
        elif vtag == 'AoI':
            vcolor = (41, 128, 185)
        else:
            vcolor = (99, 110, 114)
        dpg.draw_polygon(drawVex, color=vcolor, fill=vcolor, parent=node)
        dpg.draw_text(
            ctf.dpgCoord(self.x, self.y, ex, ey),
            self.id,
            color=(0, 0, 0),
            size=20,
            parent=node
        )

    # 绘制轨迹
    def plotTrajectory(self, node: dpg.node, ex: float, ey: float, ctf: CoordTF):
        if self.plannedTrajectory and self.plannedTrajectory.xQueue:
            # 生成轨迹点
            tps = [
                ctf.dpgCoord(
                    self.plannedTrajectory.xQueue[i],
                    self.plannedTrajectory.yQueue[i],
                    ex, ey
                ) for i in range(len(self.plannedTrajectory.xQueue))
            ]
            # 绘制计划轨迹
            dpg.draw_polyline(tps, color=(205, 132, 241),
                              parent=node, thickness=2)

    # 绘制实际轨迹
    def plotDBTrajectory(self, node: dpg.node, ex: float, ey: float, ctf: CoordTF):
        if self.dbTrajectory and self.dbTrajectory.xQueue:
            tps = [
                ctf.dpgCoord(
                    self.dbTrajectory.xQueue[i],
                    self.dbTrajectory.yQueue[i],
                    ex, ey
                ) for i in range(len(self.dbTrajectory.xQueue))
            ]
            dpg.draw_polyline(tps, color=(225, 112, 85),
                              parent=node, thickness=2)

    # append yaw of the car
    # 添加车辆偏航角
    def yawAppend(self, angle: float):
        self.yawQ.append((90 - angle) * (pi / 180))

    # 添加车辆中心x坐标
    def xAppend(self, x: float):
        self.xQ.append(x - self.length / 2 * cos(self.yaw))

    # append the center y
    def yAppend(self, y: float):
        self.yQ.append(y - self.length / 2 * sin(self.yaw))

    # append the lanepos of the center of the car
    def lanePosAppend(self, lanePos: float):
        self.lanePosQ.append(lanePos - self.length / 2)

    def laneAppend(self, nb: NetworkBuild):
        traciLaneID = traci.vehicle.getLaneID(self.id)
        # 车道空值检查
        if traciLaneID == '':
            print(f"车辆{self.id}进入无效区域，准备移除")
            traci.vehicle.remove(self.id)
            return
        traciLanePos = traci.vehicle.getLanePosition(self.id)
        routeIndex = self.routeIdxQ[-1]
        if routeIndex >= 1:
            currEdge = self.routes[routeIndex]
            lastEdge = self.routes[routeIndex-1]
            edgeLanes = self.LLRDict[currEdge]['edgeLanes'] | \
                self.LLRDict[lastEdge]['edgeLanes']
            try:
                currJunctionLanes = self.LLRDict[currEdge]['junctionLanes']
            except KeyError:
                currJunctionLanes = set()
            lastJunctionLanes = self.LLRDict[lastEdge]['junctionLanes']
            junctionLanes = currJunctionLanes | lastJunctionLanes
        else:
            currEdge = self.routes[routeIndex]
            edgeLanes = self.LLRDict[currEdge]['edgeLanes']
            try:
                junctionLanes = self.LLRDict[currEdge]['junctionLanes']
            except:
                junctionLanes = set()
        searchLanes = edgeLanes | junctionLanes
        if traciLaneID in searchLanes:
            self.laneIDQ.append(traciLaneID)
            self.lanePosQ.append(traciLanePos - self.length / 2)
        else:
            for lid in searchLanes:
                if ':' in lid:
                    laneINS = nb.getJunctionLane(lid)
                else:
                    laneINS = nb.getLane(lid)
                s, d = laneINS.course_spline.cartesian_to_frenet1D(
                    self.x, self.y)
                if abs(d) < 2.0:
                    self.laneIDQ.append(lid)
                    self.lanePosQ.append(s)

    def routeIdxAppend(self, laneID: str):
        curIndexList = self.LCRDict[laneID]
        if self.routeIdxQ:
            lastIndex = self.routeIdxQ[-1]
            for curIndex in curIndexList:
                if curIndex - lastIndex == 0 or curIndex - lastIndex == 1:
                    self.routeIdxQ.append(curIndex)
                    return
        else:
            self.routeIdxQ.append(traci.vehicle.getRouteIndex(self.id))

    def __hash__(self) -> int:
        return hash(self.id)

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, self.__class__):
            return self.id == __o.id
        else:
            raise TypeError('Only class:Vehicle can be added into this set!')

    def __str__(self) -> str:
        return 'ID: {}, x: {:.5f}, y: {:.5f}, yaw: {:.5f}, speed: {:.5f}, accel: {:.5f}, vType: {}'.format(
            self.id, self.x, self.y,
            self.yaw, self.speed, self.accel, self.vTypeID
        )
        
# 定义Ego Car类
class egoCar(Vehicle):
    def __init__(
        self,
        id: str,
        deArea: float = 50.0,  # 默认检测区域半径
        sceMargin: float = 20.0  # 默认场景边缘距离
    ) -> None:
        # 只传递id参数给父类
        super().__init__(id)
        self.deArea = deArea  # 检测区域半径
        self.sceMargin = sceMargin  # 场景边缘距离
        self.id = id # 确保id属性正确赋值
        
    # 在GUI中绘制自车的检测区域（黄色半透明圆形）
    def plotdeArea(self, node: dpg.node, ex: float, ey: float, ctf: CoordTF):
        cx, cy = ctf.dpgCoord(self.x, self.y, ex, ey)
        dpg.draw_circle(
            (cx, cy),
            ctf.zoomScale*self.deArea,
            thickness=0,
            fill=(243, 156, 18, 20),
            parent=node
        )

        # dpg.draw_circle(
        #     (cx, cy),
        #     2 * zoomScale * self.deArea,
        #     color=(37, 204, 247),
        #     parent=node
        #     )

# 仿真中的虚拟障碍物或区域标记
class DummyVehicle:
    def __init__(self, x: float, y: float, radius: float) -> None:
        self.x = x
        self.y = y
        if radius > 100:
            print(
                '[yellow]The given local radius is too large, and is resized to 100![/yellow]')
            self.radius = 100
        elif radius < 20:
            print(
                '[yellow]The given local radius is too small, and is resized to 20![/yellow]')
            self.radius = 20
        else:
            self.radius = radius

    def plotArea(self, node: dpg.node, ex: float, ey: float, ctf: CoordTF):
        cx, cy = ctf.dpgCoord(self.x, self.y, ex, ey)
        dpg.draw_circle(
            (cx, cy),
            ctf.zoomScale*self.radius,
            thickness=0,
            fill=(243, 156, 18, 20),
            parent=node
        )
