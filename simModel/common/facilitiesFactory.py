"""
功能：创建LimSim仿真平台上的其他固定设施(偏向可视化)
Facilities：
    - 仿真基础 ：提供SUMO仿真环境中的基本设施功能
    - 数据存储 ：存储设施的历史轨迹数据（位置、速度、加速度等）
    - SUMO接口 ：直接与SUMO/traci交互
    - 路径管理 ：处理设施的路径规划和车道选择
    - 可视化支持 ：支持在GUI中绘制设施和轨迹
"""
class RSU_detector:
    def __init__(self, id: str) -> None:
        self.id = id
        self.lane : str = ''
        self.pos : float = 0.0
        self.detectlenth: float = 0.0
        self.detectfreq: float = 0.0
        self.output: str = ''

class RSU:
    """
    功能：创建LimSim仿真平台上的RSU(偏向可视化)
    """
    def __init__(self, id: str) -> None:
        self.id = id
        self.x :float = 0.0 # 存储RSU投影中心坐标x
        self.y :float = 0.0 # 存储RSU投影中心坐标y
        self.length: float = 5.0   # SUMO默认值，RSU投影长度
        self.width: float = 1.8   # SUMO默认值，RSU投影宽度
        self.detectors: list = [] # 存储RSU的检测器
    
    def addDetector(self,detector:RSU_detector):
        self.detectors.append(detector)

    