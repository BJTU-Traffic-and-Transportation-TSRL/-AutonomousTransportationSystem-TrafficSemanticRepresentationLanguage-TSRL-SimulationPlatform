# Preprocess for scenario generation
# Get each element's shape from `.net.xml`
# Get network topology from `.net.xml`

# for: NetworkBuild with Frenet
from __future__ import annotations
from utils.simBase import CoordTF, deduceEdge, MapCoordTF
from utils.cubic_spline import Spline2D
from utils.roadgraph import Junction, Edge, NormalLane, OVERLAP_DISTANCE, JunctionLane, TlLogic
from simModel.common.facilitiesFactory import RSU,RSU_detector # 在networkBuild.py文件的导入部分添加RSU导入
from queue import Queue
import sqlite3
from threading import Thread
import numpy as np
import xml.etree.ElementTree as ET
import dearpygui.dearpygui as dpg
from rich import print
from datetime import datetime


class geoHash:
    def __init__(self, id: tuple[int]) -> None:
        self.id = id
        self.edges: set[str] = set()
        self.junctions: set[str] = set()
        self.rsus :set[str] = set()  # 9.6 添加RSU集合


class NetworkBuild:
    def __init__(self,
                 dataBase: str,
                 networkFile: str,
                 obsFile: str = None,
                 addFile: str = None  # 9.7 添加add文件参数
                 ) -> None:
        self.dataBase = dataBase
        self.networkFile = networkFile
        self.obsFile = obsFile # 未被使用过
        self.addFile = addFile  # 9.7 添加addFile参数
        self.edges: dict[str, Edge] = {}
        self.lanes: dict[str, NormalLane] = {}
        self.junctions: dict[str, Junction] = {}
        self.junctionLanes: dict[str, JunctionLane] = {}
        self.rsus: dict[str, RSU] = {}  # 9.6 存储RSU对象
        self.tlLogics: dict[str, TlLogic] = {}
        # self.obstacles: dict[str, circleObs | rectangleObs] = {}
        self.dataQue = Queue()
        self.geoHashes: dict[tuple[int], geoHash] = {}

    def getEdge(self, eid: str) -> Edge:
        try:
            return self.edges[eid]
        except KeyError:
            return

    def getLane(self, lid: str) -> NormalLane:
        try:
            return self.lanes[lid]
        except KeyError:
            return

    def getJunction(self, jid: str) -> Junction:
        try:
            return self.junctions[jid]
        except KeyError:
            return

    def getJunctionLane(self, jlid: str) -> JunctionLane:
        try:
            return self.junctionLanes[jlid]
        except KeyError:
            return

    def getTlLogic(self, tlid: str) -> TlLogic:
        try:
            return self.tlLogics[tlid]
        except KeyError:
            return

    # def getObstacle(self, obsid: str) -> circleObs | rectangleObs:
    #     try:
    #         return self.obstacles[obsid]
    #     except KeyError:
    #         return

    def affGridIDs(self, centerLine: list[tuple[float]]) -> set[tuple[int]]:
        affGridIDs = set()
        for poi in centerLine:
            poixhash = int(poi[0] // 100)
            poiyhash = int(poi[1] // 100)
            affGridIDs.add((poixhash, poiyhash))

        return affGridIDs

    def insertCommit(self):
        conn = sqlite3.connect(self.dataBase, check_same_thread=False)
        cur = conn.cursor()
        commitCnt = 0
        while not self.dataQue.empty():
            tableName, data, process = self.dataQue.get()
            sql = '{} INTO {} VALUES '.format(process, tableName) + \
                '(' + '?,'*(len(data)-1) + '?' + ')'
            try:
                cur.execute(sql, data)
            except sqlite3.OperationalError as e:
                print(sql, data)
            commitCnt += 1
            if commitCnt == 10000:
                conn.commit()
                commitCnt = 0
        conn.commit()
        cur.close()
        conn.close()

        print('[green bold]Network information commited at {}.[/green bold]'.format(
            datetime.now().strftime('%H:%M:%S.%f')[:-3]))

    def processRawShape(self, rawShape: str) -> list:
        rawList = rawShape.split(' ')
        floatShape = [list(map(float, p.split(','))) for p in rawList]
        return floatShape

    def processEdge(self, eid: str, child: ET.Element):
        if eid[0] == ':':
            for gchild in child:
                ilid = gchild.attrib['id']
                try:
                    ilspeed = float(gchild.attrib['speed'])
                except:
                    ilspeed = 13.89
                try:
                    ilwidth = float(gchild.attrib['width'])
                except KeyError:
                    ilwidth = 3.2
                ilLength = float(gchild.attrib['length'])
                self.junctionLanes[ilid] = JunctionLane(
                    id=ilid, width=ilwidth, speed_limit=ilspeed,
                    sumo_length=ilLength,
                )
                self.dataQue.put((
                    'junctionLaneINFO', (
                        ilid, ilwidth, ilspeed, ilLength,
                        None, 0
                    ), 'INSERT'
                ))
        else:
            fromNode = child.attrib['from']
            toNode = child.attrib['to']
            edge = Edge(id=eid, from_junction=fromNode, to_junction=toNode)
            laneNumber = 0
            for gchild in child:
                if gchild.tag == 'lane':
                    lid = gchild.attrib['id']
                    try:
                        lwidth = float(gchild.attrib['width'])
                    except KeyError:
                        lwidth = 3.2
                    lspeed = float(gchild.attrib['speed'])
                    rawShape = gchild.attrib['shape']
                    lshape = self.processRawShape(rawShape)
                    llength = float(gchild.attrib['length'])
                    lane = NormalLane(id=lid, width=lwidth, speed_limit=lspeed,
                                      sumo_length=llength, affiliated_edge=edge)
                    self.dataQue.put((
                        'laneINFO', (
                            lid, rawShape, lwidth, lspeed, eid, llength
                        ), 'INSERT'
                    ))
                    shapeUnzip = list(zip(*lshape))

                    # interpolate shape points for better represent shape
                    shapeUnzip = [
                        np.interp(
                            np.linspace(0, len(shapeUnzip[0])-1, 50),
                            np.arange(0, len(shapeUnzip[0])),
                            shapeUnzip[i]
                        ) for i in range(2)
                    ]
                    lane.course_spline = Spline2D(shapeUnzip[0], shapeUnzip[1])
                    lane.getPlotElem()
                    self.lanes[lid] = lane
                    edge.lanes.add(lane.id)
                    laneAffGridIDs = self.affGridIDs(lane.center_line)
                    edge.affGridIDs = edge.affGridIDs | laneAffGridIDs
                    laneNumber += 1
            edge.lane_num = laneNumber
            for gridID in edge.affGridIDs:
                try:
                    geohash = self.geoHashes[gridID]
                except KeyError:
                    geohash = geoHash(gridID)
                    self.geoHashes[gridID] = geohash
                geohash.edges.add(eid)
            self.edges[eid] = edge
            self.dataQue.put((
                'edgeINFO', (eid, laneNumber, fromNode, toNode), 'INSERT'
            ))

    def processConnection(self, child: ET.Element):
        fromEdgeID = child.attrib['from']
        fromEdge = self.getEdge(fromEdgeID)
        fromLaneIdx = child.attrib['fromLane']
        fromLaneID = fromEdgeID + '_' + fromLaneIdx
        fromLane = self.getLane(fromLaneID)
        toEdgeID = child.attrib['to']
        toLaneIdx = child.attrib['toLane']
        toLaneID = toEdgeID + '_' + toLaneIdx
        toLane = self.getLane(toLaneID)
        if fromLane and toLane:
            direction = child.attrib['dir']
            junctionLaneID = child.attrib['via']
            junctionLane = self.getJunctionLane(junctionLaneID)
            self.dataQue.put((
                'connectionINFO', (
                    fromLaneID, toLaneID, direction, junctionLaneID
                ), 'INSERT'
            ))
            if junctionLane.sumo_length < 1:
                fromLane.next_lanes[toLaneID] = (toLaneID, 's')
                fromEdge.next_edge_info[toEdgeID].add(fromLaneID)
            else:
                # junctionLane = self.getJunctionLane(junctionLaneID)
                if 'tl' in child.attrib.keys():
                    tl = child.attrib['tl']
                    linkIndex = int(child.attrib['linkIndex'])
                    junctionLane.tlLogic = tl
                    junctionLane.tlsIndex = linkIndex
                self.dataQue.put((
                    'junctionLaneINFO', (
                        junctionLane.id, junctionLane.width,
                        junctionLane.speed_limit,
                        junctionLane.sumo_length,
                        junctionLane.tlLogic,
                        junctionLane.tlsIndex
                    ), 'REPLACE'
                ))
                center_line = []
                for si in np.linspace(
                    fromLane.course_spline.s[-1] - OVERLAP_DISTANCE,
                    fromLane.course_spline.s[-1], num=20
                ):
                    center_line.append(
                        fromLane.course_spline.calc_position(si))
                for si in np.linspace(0, OVERLAP_DISTANCE, num=20):
                    center_line.append(
                        toLane.course_spline.calc_position(si)
                    )
                junctionLane.course_spline = Spline2D(
                    list(zip(*center_line))[0], list(zip(*center_line))[1]
                )
                junctionLane.getPlotElem()
                junctionLane.last_lane_id = fromLaneID
                junctionLane.next_lane_id = toLaneID
                fromLane.next_lanes[toLaneID] = (junctionLaneID, direction)
                fromEdge.next_edge_info[toEdgeID].add(fromLaneID)
                # add this junctionLane to it's parent Junction's JunctionLanes
                fromEdge = self.getEdge(fromEdgeID)
                juncID = fromEdge.to_junction
                junction = self.getJunction(juncID)
                junctionLane.affJunc = juncID
                jlAffGridIDs = self.affGridIDs(junctionLane.center_line)
                junction.affGridIDs = junction.affGridIDs | jlAffGridIDs
                junction.JunctionLanes.add(junctionLaneID)

    def getData(self):
        elementTree = ET.parse(self.networkFile)
        root = elementTree.getroot()
        for child in root:
            if child.tag == 'edge':
                eid = child.attrib['id']
                # Some useless internal lanes will be generated by the follow codes.
                self.processEdge(eid, child)
            elif child.tag == 'junction':
                jid = child.attrib['id']
                junc = Junction(jid)
                if jid[0] != ':':
                    intLanes = child.attrib['intLanes']
                    if intLanes:
                        intLanes = intLanes.split(' ')
                        for il in intLanes:
                            ilins = self.getJunctionLane(il)
                            ilins.affJunc = jid
                            junc.JunctionLanes.add(il)
                    jrawShape = child.attrib['shape']
                    juncShape = self.processRawShape(jrawShape)
                    # Add the first point to form a closed shape
                    juncShape.append(juncShape[0])
                    junc.shape = juncShape
                    self.junctions[jid] = junc
                    self.dataQue.put((
                        'junctionINFO', (jid, jrawShape), 'INSERT'
                    ))
            elif child.tag == 'connection':
                # in .net.xml, the elements 'edge' come first than elements
                # 'connection', so the follow codes can work well.
                self.processConnection(child)
            elif child.tag == 'tlLogic':
                tlid = child.attrib['id']
                tlType = child.attrib['type']
                preDefPhases = []
                for gchild in child:
                    if gchild.tag == 'phase':
                        preDefPhases.append(gchild.attrib['state'])

                self.tlLogics[tlid] = TlLogic(tlid, tlType, preDefPhases)
                self.dataQue.put((
                    'tlLogicINFO',
                    (tlid, tlType, ' '.join(preDefPhases)), 'INSERT'
                ))
        for junction in self.junctions.values():
            for gridID in junction.affGridIDs:
                try:
                    geohash = self.geoHashes[gridID]
                except KeyError:
                    geohash = geoHash(gridID)
                    self.geoHashes[gridID] = geohash
                geohash.junctions.add(junction.id)

        for ghid, ghins in self.geoHashes.items():
            ghx, ghy = ghid
            ghEdges = ','.join(ghins.edges)
            ghJunctions = ','.join(ghins.junctions)
            self.dataQue.put((
                'geohashINFO',
                (ghx, ghy, ghEdges, ghJunctions), 'INSERT'
            ))
        
        # 9.6新增：解析add.xml文件中的RSU信息
        if self.addFile:
            try:
                addTree = ET.parse(self.addFile)
                addRoot = addTree.getroot()
                for child in addRoot:
                    if child.tag == 'poi' and child.attrib.get('id', '').startswith('RSU_'):
                        # 解析RSU的投影信息
                        rsu_id = child.attrib['id']
                        rsu_x = float(child.attrib['x'])
                        rsu_y = float(child.attrib['y'])
                        rsu = RSU(rsu_id)
                        rsu.x=rsu_x
                        rsu.y=rsu_y
                    elif child.tag == 'laneAreaDetector' and child.attrib.get('id', '').startswith('RSU_'):
                        # 解析RSU的实际探测能力
                        rsu_detector_id = child.attrib['id']
                        rsu_detector_lane=child.attrib['lane']
                        rsu_pos=float(child.attrib['pos'])
                        rsu_detectlenth=float(child.attrib['length'])
                        rsu_detectfreq=float(child.attrib['freq'])
                        rsu_output=child.attrib['file']
                        rsu_detector=RSU_detector(id=rsu_detector_id)
                        rsu_detector.lane=rsu_detector_lane
                        rsu_detector.pos=rsu_pos
                        rsu_detector.detectlenth=rsu_detectlenth
                        rsu_detector.detectfreq=rsu_detectfreq
                        rsu_detector.output=rsu_output
                        rsu.addDetector(rsu_detector)
                self.addRSU(rsu)
            except Exception as e:
                print(f'Error parsing add.xml file: {str(e)}')

    def buildTopology(self):
        for eid, einfo in self.edges.items():
            fj = self.getJunction(einfo.from_junction)
            tj = self.getJunction(einfo.to_junction)
            fj.outgoing_edges.add(eid)
            tj.incoming_edges.add(eid)

        print('[green bold]Network building finished at {}.[/green bold]'.format(
            datetime.now().strftime('%H:%M:%S.%f')[:-3]))

        Th = Thread(target=self.insertCommit)
        Th.start()

    # These plots should be modified, since the y-axis is different in dpg
    # def plotLane(
    #         self, lane: NormalLane, flag, node: dpg.node, ex: float, ey: float,
    #         ctf: CoordTF):
    #     if flag & 0b10:
    #         left_bound_tf = [
    #             ctf.dpgCoord(wp[0], wp[1], ex, ey) for wp in lane.left_bound
    #         ]
    #         dpg.draw_polyline(
    #             left_bound_tf,
    #             color=(0, 0, 0), thickness = 2,
    #             parent=node)
    #     else:
    #         left_bound_tf = [
    #             ctf.dpgCoord(wp[0], wp[1], ex, ey) for wp in lane.left_bound
    #         ]
    #         # dpg.draw_polyline(left_bound_tf, color=(178, 190, 195), parent=node)
    #         dpg.draw_polyline(left_bound_tf, color=(0, 0, 0, 70), thickness=1, parent=node)
    #     if flag & 0b01:
    #         right_bound_tf = [
    #             ctf.dpgCoord(wp[0], wp[1], ex, ey) for wp in lane.right_bound
    #         ]
    #         dpg.draw_polyline(
    #             right_bound_tf, color=(0, 0, 0), thickness = 2, parent=node
    #         )

    def plotLane(
            self, lane: NormalLane, flag: int, ex: float, ey: float,
            node: dpg.node, ctf: MapCoordTF
    ):
        if flag & 0b10:
            return
        else:
            left_bound_tf = [
                ctf.dpgCoord(wp[0], wp[1], ex, ey) for wp in lane.left_bound
            ]
            dpg.draw_polyline(
                left_bound_tf, color=(0, 0, 0, 100), parent=node
            )

    def plotMapLane(
            self, lane: NormalLane, flag: int, node: dpg.node, ctf: MapCoordTF
    ):
        if flag & 0b10:
            left_bound_tf = [
                ctf.dpgCoord(wp[0], wp[1]) for wp in lane.left_bound
            ]
            # print(lane.left_bound)
            # print('#'*20)
            # print(left_bound_tf)
            dpg.draw_polyline(left_bound_tf, color=(
                255, 255, 255), parent=node)
        if flag & 0b01:
            right_bound_tf = [
                ctf.dpgCoord(wp[0], wp[1]) for wp in lane.right_bound
            ]
            dpg.draw_polyline(
                right_bound_tf, color=(255, 255, 255, 100), parent=node
            )

    def plotMapEdge(self, eid: str, node: dpg.node, ctf: MapCoordTF):
        edge = self.getEdge(eid)
        for lane_index in range(edge.lane_num):
            lane_id = edge.id + '_' + str(lane_index)
            lane = self.getLane(lane_id)
            flag = 0b00
            if lane_index == 0:
                flag += 1
            if lane_index == edge.lane_num - 1:
                flag += 2
            self.plotMapLane(lane, flag, node, ctf)

    # def plotEdge(self, eid: str, node: dpg.node, ex: float, ey: float,
    #              ctf: CoordTF):
    #     edge = self.getEdge(eid)
    #     for lane_index in range(edge.lane_num):
    #         lane_id = edge.id + '_' + str(lane_index)
    #         lane = self.getLane(lane_id)
    #         flag = 0b00
    #         if lane_index == 0:
    #             flag += 1
    #         if lane_index == edge.lane_num - 1:
    #             flag += 2
    #         self.plotLane(lane, flag, node, ex, ey, ctf)

    def plotEdge(self, eid: str, node: dpg.node, ex: float, ey: float,
                 ctf: CoordTF):
        edge = self.getEdge(eid)
        for lane_index in range(edge.lane_num):
            lane_id = edge.id + '_' + str(lane_index)
            lane = self.getLane(lane_id)
            flag = 0b00
            if lane_index == 0:
                flag += 1
            if lane_index == edge.lane_num - 1:
                flag += 2
            self.plotLane(lane, flag, ex, ey, node, ctf)
            if flag & 0b10:
                left_bound_tf = [
                    ctf.dpgCoord(wp[0], wp[1], ex, ey) for wp in lane.left_bound
                ]
            if flag & 0b01:
                right_bound_tf = [
                    ctf.dpgCoord(wp[0], wp[1], ex, ey) for wp in lane.right_bound
                ]
        # 根据左右边界获取 edge 的封闭图形
        left_bound_tf.reverse()
        right_bound_tf.extend(left_bound_tf)
        right_bound_tf.append(right_bound_tf[0])
        dpg.draw_polygon(right_bound_tf, color=(0, 0, 0),
                         thickness=2, fill=(0, 0, 0, 30), parent=node)

    def plotJunctionLane(self, jlid: str, node: dpg.node, ex: float, ey: float,
                         ctf: CoordTF):
        juncLane = self.getJunctionLane(jlid)
        if juncLane:
            try:
                center_line = juncLane.center_line
            except AttributeError:
                return
            center_line_tf = [
                ctf.dpgCoord(wp[0], wp[1], ex, ey) for wp in center_line
            ]
            if juncLane.currTlState:
                if juncLane.currTlState == 'r':
                    # jlColor = (232, 65, 24)
                    jlColor = (255, 107, 129, 100)
                elif juncLane.currTlState == 'y':
                    jlColor = (251, 197, 49, 100)
                elif juncLane.currTlState == 'g' or juncLane.currTlState == 'G':
                    jlColor = (39, 174, 96, 50)
            else:
                jlColor = (0, 0, 0, 30)
            dpg.draw_polyline(center_line_tf, color=jlColor,
                              thickness=17, parent=node)

    def plotJunction(
            self, jid: str, node: dpg.node, ex: float, ey: float, ctf: CoordTF):
        junction = self.getJunction(jid)
        polyShape = [ctf.dpgCoord(p[0], p[1], ex, ey) for p in junction.shape]
        dpg.draw_polyline(polyShape, color=(0, 0, 0), thickness=2, parent=node)
        for jl in junction.JunctionLanes:
            self.plotJunctionLane(jl, node, ex, ey, ctf)

    def plotMapJunction(self, jid: str, node: dpg.node, ctf: MapCoordTF):
        junction = self.getJunction(jid)
        polyShape = [ctf.dpgCoord(p[0], p[1]) for p in junction.shape]
        dpg.draw_polyline(polyShape, color=(255, 255, 255, 200), parent=node)
        for inEdgeID in junction.incoming_edges:
            self.plotMapEdge(inEdgeID, node, ctf)
        for outEdgeID in junction.outgoing_edges:
            self.plotMapEdge(outEdgeID, node, ctf)

    def addRSU(self, rsu: RSU) -> None:
        """9.6新增：添加RSU到网络，并更新geohash"""
        self.rsus[rsu.id] = rsu
        # 计算RSU所在的geohash网格
        rsu_x, rsu_y = rsu.x, rsu.y
        geox = int(rsu_x // 100)
        geoy = int(rsu_y // 100)
        # 添加到对应的geohash
        gridID = (geox, geoy)
        try:
            geohash = self.geoHashes[gridID]
        except KeyError:
            geohash = geoHash(gridID)
            self.geoHashes[gridID] = geohash
        
        geohash.rsus.add(rsu.id)

    def getRSU(self, rsu_id: str) -> RSU:
        """9.6新增：获取RSU对象"""
        try:
            return self.rsus[rsu_id]
        except KeyError:
            return None

    def getRSUsInRange(self, center_x: float, center_y: float, radius: float) -> set[str]:
        """9.6新增：获取指定范围内的RSU ID集合"""
        rsu_ids = set()
        
        # 计算影响范围内的geohash网格
        min_geox = int((center_x - radius) // 100)
        max_geox = int((center_x + radius) // 100)
        min_geoy = int((center_y - radius) // 100)
        max_geoy = int((center_y + radius) // 100)
    
        # 遍历所有可能的geohash网格
        for gx in range(min_geox, max_geox + 1):
            for gy in range(min_geoy, max_geoy + 1):
                gridID = (gx, gy)
                try:
                    geohash = self.geoHashes[gridID]
                    rsu_ids |= geohash.rsus
                except KeyError:
                    continue
        
        # 进一步过滤在圆形范围内的RSU
        final_rsu_ids = set()
        for rsu_id in rsu_ids:
            rsu = self.getRSU(rsu_id)
            if rsu and sqrt(pow(rsu.xQ - center_x, 2) + pow(rsu.yQ - center_y, 2)) <= radius:
                final_rsu_ids.add(rsu_id)
        
        return final_rsu_ids

    def plotRSU(self, rsu_id: str, node: dpg.node, ex: float, ey: float, ctf: CoordTF):
        """9.6新增：绘制单个RSU"""
        rsu = self.getRSU(rsu_id)
        if not rsu:
            return
        
        # RSU中心坐标
        center_x, center_y = rsu.x, rsu.y
        
        # RSU尺寸
        length = rsu.length
        width = rsu.width
        
        # 计算RSU的四个角点（矩形）
        half_length = length / 2
        half_width = width / 2
        
        # 创建RSU的矩形顶点
        vertices = [
            (center_x - half_length, center_y - half_width),
            (center_x + half_length, center_y - half_width),
            (center_x + half_length, center_y + half_width),
            (center_x - half_length, center_y + half_width)
        ]
        
        # 转换坐标到dearpygui坐标系
        vertices_tf = [ctf.dpgCoord(v[0], v[1], ex, ey) for v in vertices]
        
        # 绘制RSU主体（蓝色矩形）
        dpg.draw_polygon(
            vertices_tf,
            color=(0, 100, 255),  # 蓝色边框
            thickness=2,
            fill=(0, 100, 255, 100),  # 半透明蓝色填充
            parent=node
        )
        
        # 绘制RSU中心点
        center_tf = ctf.dpgCoord(center_x, center_y, ex, ey)
        dpg.draw_circle(
            center_tf,
            radius=3,
            color=(255, 255, 255),
            thickness=2,
            fill=(0, 100, 255),
            parent=node
        )
        
        # 绘制RSU检测范围（圆形）
        detect_radius = rsu.detectors[0].detectlenth
        dpg.draw_circle(
            center_tf,
            radius=detect_radius * ctf.zoomScale,
            color=(0, 100, 255, 150),
            thickness=1,
            parent=node
        )
        
        # 添加RSU ID标签
        label_pos = (center_tf[0], center_tf[1] - 15)
        dpg.draw_text(
            label_pos,
            f"RSU-{rsu_id}",
            color=(0, 0, 0),
            size=12,
            parent=node
        )

    def plotMapRSU(self, rsu_id: str, node: dpg.node, ctf: MapCoordTF):
        """在地图模式下绘制RSU"""
        rsu = self.rsus.get(rsu_id)
        if not rsu:
            return
        
        # 直接使用地图坐标
        center_x, center_y = ctf.dpgCoord(rsu.x, rsu.y)
        
        # RSU尺寸
        length = rsu.length
        width = rsu.width
        
        # 计算绘制尺寸（考虑地图缩放）
        draw_length = length * ctf.zoomScale
        draw_width = width * ctf.zoomScale
        
        # 创建RSU的矩形顶点
        half_length = draw_length / 2
        half_width = draw_width / 2
        
        vertices = [
            (center_x - half_length, center_y - half_width),
            (center_x + half_length, center_y - half_width),
            (center_x + half_length, center_y + half_width),
            (center_x - half_length, center_y + half_width)
        ]
        
        # 绘制RSU主体
        dpg.draw_polygon(
            vertices,
            color=(0, 150, 255),
            thickness=2,
            fill=(0, 150, 255, 150),
            parent=node
        )
        
        # 绘制中心点
        dpg.draw_circle(
            (center_x, center_y),
            radius=3,
            color=(255, 255, 255),
            thickness=2,
            fill=(0, 150, 255),
            parent=node
        )
        
        # 绘制检测范围
        detect_radius = rsu.detectors[0].detectlenth * ctf.zoomScale
        dpg.draw_circle(
            (center_x, center_y),
            radius=detect_radius,
            color=(0, 150, 255, 100),
            thickness=1,
            parent=node
        )
        
        # 添加标签
        dpg.draw_text(
            (center_x, center_y - 20),
            f"RSU-{rsu_id}",
            color=(255, 255, 255),
            size=12,
            parent=node
        )


class Rebuild(NetworkBuild):
    def __init__(self,
                 dataBase: str,
                 ) -> None:
        networkFile: str = None,
        obsFile: str = None
        super().__init__(dataBase, networkFile, obsFile)

    def getData(self):
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()

        cur.execute('SELECT * FROM junctionINFO;')
        junctionINFO = cur.fetchall()
        if junctionINFO:
            for ji in junctionINFO:
                junctionID = ji[0]
                jrawShape = ji[1]
                juncShape = self.processRawShape(jrawShape)
                # Add the first point to form a closed shape
                juncShape.append(juncShape[0])
                junc = Junction(junctionID)
                junc.shape = juncShape
                self.junctions[junctionID] = junc

        cur.execute('SELECT * FROM edgeINFO;')
        edgeInfo = cur.fetchall()
        if edgeInfo:
            for ed in edgeInfo:
                eid, laneNumber, fromJunction, toJunction = ed
                self.edges[eid] = Edge(
                    id=eid, lane_num=laneNumber,
                    from_junction=fromJunction,
                    to_junction=toJunction
                )

        cur.execute('SELECT * FROM laneINFO;')
        laneINFO = cur.fetchall()
        if laneINFO:
            for la in laneINFO:
                lid, rawShape, lwidth, lspeed, eid, llength = la
                lshape = self.processRawShape(rawShape)
                lane = NormalLane(lid, lwidth, lspeed, eid, llength)
                lane = NormalLane(
                    id=lid, width=lwidth, speed_limit=lspeed,
                    affiliated_edge=self.getEdge(eid), sumo_length=llength
                )
                shapeUnzip = list(zip(*lshape))
                # interpolate shape points for better represent shape
                shapeUnzip = [
                    np.interp(
                        np.linspace(0, len(shapeUnzip[0])-1, 50),
                        np.arange(0, len(shapeUnzip[0])),
                        shapeUnzip[i]
                    ) for i in range(2)
                ]
                lane.course_spline = Spline2D(shapeUnzip[0], shapeUnzip[1])
                lane.getPlotElem()
                self.lanes[lid] = lane
                self.getEdge(eid).lanes.add(lid)

        cur.execute('SELECT * FROM junctionLaneINFO;')
        JunctionLaneINFO = cur.fetchall()
        if JunctionLaneINFO:
            for jl in JunctionLaneINFO:
                jlid, jlwidth, jlspeed, jlLength, tlLogicID, tlsIndex = jl
                self.junctionLanes[jlid] = JunctionLane(
                    id=jlid, width=jlwidth,
                    speed_limit=jlspeed, sumo_length=jlLength,
                    tlLogic=tlLogicID, tlsIndex=tlsIndex
                )

        cur.execute('SELECT * FROM tlLogicINFO;')
        tlLogicINFO = cur.fetchall()
        if tlLogicINFO:
            for tll in tlLogicINFO:
                tlid, tlType, preDefPhases = tll
                self.tlLogics[tlid] = TlLogic(
                    tlid, tlType, preDefPhases.split(' '))

        cur.execute('SELECT * FROM connectionINFO;')
        connectionINFO = cur.fetchall()
        if connectionINFO:
            for ci in connectionINFO:
                fromLaneID, toLaneID, direction, junctionLaneID = ci
                fromLane = self.getLane(fromLaneID)
                fromEdgeID = deduceEdge(fromLaneID)
                fromEdge = self.getEdge(fromEdgeID)
                junctionLane = self.getJunctionLane(junctionLaneID)
                if not junctionLane:
                    print(
                        'The JunctionLane is not found in database: ',
                        junctionLaneID
                    )
                toEdgeID = deduceEdge(toLaneID)
                if junctionLane.sumo_length < 1:
                    fromLane.next_lanes[toLaneID] = (toLaneID, 's')
                    fromEdge.next_edge_info[toEdgeID].add(fromLaneID)
                else:
                    junctionLane = self.getJunctionLane(junctionLaneID)
                    fromEdgeID = deduceEdge(fromLaneID)
                    center_line = []
                    for si in np.linspace(
                        fromLane.course_spline.s[-1] - OVERLAP_DISTANCE,
                        fromLane.course_spline.s[-1], num=20
                    ):
                        center_line.append(
                            fromLane.course_spline.calc_position(si))
                    for si in np.linspace(0, OVERLAP_DISTANCE, num=20):
                        center_line.append(
                            self.getLane(
                                toLaneID).course_spline.calc_position(si)
                        )
                    junctionLane.course_spline = Spline2D(
                        list(zip(*center_line))[0], list(zip(*center_line))[1]
                    )
                    junctionLane.getPlotElem()
                    junctionLane.last_lane_id = fromLaneID
                    junctionLane.next_lane_id = toLaneID
                    fromLane.next_lanes[toLaneID] = (
                        junctionLaneID, direction)
                    fromEdge.next_edge_info[toEdgeID].add(fromLaneID)
                    # add this junctionLane to it's parent Junction's JunctionLanes
                    fromEdge = self.getEdge(fromEdgeID)
                    junction = self.getJunction(fromEdge.to_junction)
                    junction.JunctionLanes.add(junctionLaneID)

        cur.execute('SELECT * FROM geohashINFO;')
        geohashINFO = cur.fetchall()
        if geohashINFO:
            for gi in geohashINFO:
                ghx, ghy, ghEdges, ghJunctions = gi
                ghID = (ghx, ghy)
                geohash = geoHash(ghID)
                if ghEdges:
                    geohash.edges = set(ghEdges.split(','))
                if ghJunctions:
                    geohash.junctions = set(ghJunctions.split(','))
                self.geoHashes[ghID] = geohash

        cur.close()
        conn.close()

    def buildTopology(self):
        for k, v in self.edges.items():
            fj = self.getJunction(v.from_junction)
            tj = self.getJunction(v.to_junction)
            fj.outgoing_edges.add(k)
            tj.incoming_edges.add(k)

        print('[green bold]Network building finished at {}.[/green bold]'.format(
            datetime.now().strftime('%H:%M:%S.%f')[:-3]))
