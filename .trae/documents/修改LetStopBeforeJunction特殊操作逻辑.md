## 修改LetStopBeforeJunction特殊操作逻辑计划

### 1. 分析现有代码

#### TSRL\_ego\_vehicle\_planner.py 中的LetStopBeforeJunction处理

* 当前代码在第251-258行处理SPECIAL\_HANDLING行为，当action包含"LetStopBeforeJunction"时，调用stop\_trajectory\_generator并传递special\_prompt="LetStopBeforeJunction"

* 但没有区分车辆是在道路上即将进入交叉口，还是已经在交叉口内

#### trajectory\_generator.py 中的停止位置计算

* 第242-244行处理红灯停止情况，停在车道终点前5米

* 需要将此逻辑扩展到LetStopBeforeJunction情况

#### 交叉口检测逻辑

* 需要区分车辆当前位置：道路上即将进入交叉口 或 已在交叉口内

### 2. 修改计划

#### 步骤1：增强is\_in\_intersection方法

* 完善is\_in\_intersection方法，确保能准确检测车辆是否在交叉口内

* 检查当前车道是否为JunctionLane，或即将进入JunctionLane

#### 步骤2：修改LetStopBeforeJunction处理逻辑

* 在TSRL\_ego\_vehicle\_planner.py的SPECIAL\_HANDLING分支中，添加位置判断

* 情况1：车辆在道路上，即将进入拥堵交叉口

  * 调用stop\_trajectory\_generator，初始化并传递CongestionJunction=True参数，仿照红灯停止逻辑（停在车道终点前5米）写一个新的“防止进入拥堵交叉口”逻辑

* 情况2：车辆已在交叉口内

  * 调用stop\_trajectory\_generator，传递force\_stop=True参数，立即停止

#### 步骤3：优化stop\_trajectory\_generator中的特殊处理

* 确保stop\_trajectory\_generator能正确处理LetStopBeforeJunction特殊提示

* 验证“防止进入拥堵交叉口”逻辑能被LetStopBeforeJunction情况复用

### 3. 预期结果

修改后，当车辆收到LetStopBeforeJunction消息时：

* 如果在道路上即将进入拥堵交叉口，会停在车道终点前5米

* 如果已在交叉口内，会立即停止

* 能正确处理不同位置的停止需求，确保安全

### 4. 具体修改点

1. 修改TSRL\_ego\_vehicle\_planner.py第251-258行的SPECIAL\_HANDLING处理逻辑
2. 可能需要修改is\_in\_intersection方法，增强交叉口检测能力
3. 确保stop\_trajectory\_generator能正确处理传递的参数

### 5. 验证计划

* 检查修改后的代码逻辑是否完整

* 验证位置判断是否准确

* 确保轨迹生成器能正确响应不同参数

* 检查是否有语法错误或逻辑漏洞

