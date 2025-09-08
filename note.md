# 代码修改中的记录
##  一、trafficManager/common/vehicle.py 和 simModel/common/carFactory.py 中的 Vehicle 类的主要区别

1. trafficManager/common/vehicle.py::Vehicle：

* 这是仿真交通管理系统中的**逻辑车辆**类，主要用于**交通决策、行为、状态管理**等
* 关注点在于车辆的行为（如变道、停车、跟车等）、状态（如位置、速度、加速度等）、与道路结构的交互（如可用车道、车道变换等）。
* 适合做**决策规划、行为模拟**。

2. simModel/common/carFactory.py::Vehicle
* 这是**仿真底层（如SUMO仿真）**中的“物理车辆”类，主要用于**与仿真引擎（如SUMO）交互、数据采集、轨迹记录、可视化**等
* 关注点在于车辆的**轨迹数据存储**（如xQ, yQ, speedQ等队列）、与仿真引擎的接口（如traci）、可视化绘制、导出数据等
* 适合做**数据采集、轨迹回放、可视化**

### 总结
* trafficManager/common/vehicle.py::Vehicle

偏重于**决策与行为建模**，用于交通仿真中的“智能体”层，关注车辆的行为、状态、与道路的交互

* simModel/common/carFactory.py::Vehicle

偏重于**数据采集与仿真接口**，用于与底层仿真引擎（如SUMO）交互，关注轨迹数据、可视化、与仿真引擎的同步。