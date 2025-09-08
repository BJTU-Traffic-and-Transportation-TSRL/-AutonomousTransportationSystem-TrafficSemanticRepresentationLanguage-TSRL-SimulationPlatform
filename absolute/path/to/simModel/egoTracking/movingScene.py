class MovingScene:
    def __init__(self, netInfo: NetworkBuild, ego: egoCar, vehicles_with_stops=None) -> None:
        self.netInfo = netInfo
        self.ego = ego
        self.edges: set = None
        self.junctions: set = None
        self.currVehicles: dict[str, Vehicle] = {}
        self.vehINAoI: dict[str, Vehicle] = {}
        self.outOfAoI: dict[str, Vehicle] = {}
        self.vehicles_with_stops = vehicles_with_stops  # 添加停车信息