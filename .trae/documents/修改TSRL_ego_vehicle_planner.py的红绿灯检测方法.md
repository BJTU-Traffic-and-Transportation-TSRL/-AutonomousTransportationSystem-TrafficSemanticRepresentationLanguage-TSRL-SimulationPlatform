### 修改计划

**修改目标**：仿照`TSRL_multi_vehicle_planner.py`的红绿灯检测形式，优化`TSRL_ego_vehicle_planner.py`的红绿灯检测方法，提高代码可读性和可维护性。

**修改内容**：

1. **在TSRL_ego_vehicle_planner.py中添加辅助方法**
   - 添加`is_in_intersection`方法，用于检查车辆是否在交叉路口
   - 添加`is_waiting_for_green_light`方法，用于检查车辆是否在等待绿灯
   - 这两个方法的实现与`TSRL_multi_vehicle_planner.py`中的对应方法保持一致

2. **修改KL行为中的红绿灯检测逻辑**
   - 将原来复杂的条件判断替换为调用新添加的`is_waiting_for_green_light`方法
   - 保持相同的功能，但提高代码可读性

**具体修改点**：

1. **添加is_in_intersection方法**
   - 位置：在TSRL_ego_vehicle_planner.py类中
   - 功能：检查当前车道是否为NormalLane，下一个车道是否为JunctionLane
   - 返回：布尔值，表示车辆是否在交叉路口

2. **添加is_waiting_for_green_light方法**
   - 位置：在TSRL_ego_vehicle_planner.py类中
   - 功能：先调用is_in_intersection，再检查下一个车道的红绿灯状态
   - 返回：布尔值，表示车辆是否在等待绿灯

3. **修改KL行为中的红绿灯检测**
   - 位置：TSRL_ego_vehicle_planner.py第102-104行
   - 将原来的复杂条件判断替换为：`if self.is_waiting_for_green_light(current_lane, next_lane):`

**预期效果**：
- 代码结构更加清晰，可读性提高
- 与多车辆规划器保持一致的红绿灯检测逻辑
- 便于未来维护和扩展

**修改文件**：
- `e:/Analysis and Inference/Analysis and Inference/Autonomous Transportation Semantic Interaction Simulation Platform-2025.12-结题联合仿真 - under version/ATSISP/trafficManager/planner/TSRL_ego_vehicle_planner.py`