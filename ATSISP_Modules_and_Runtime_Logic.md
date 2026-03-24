# ATSISP 模块组成与运行逻辑说明

## 1. 文档目的

本文档基于 `Start.bat` 的启动流程、项目代码结构以及 `README.md` 的说明内容，对自主式交通系统语义交互仿真平台（ATSISP）的模块组成与模块间运行逻辑进行梳理。  
重点围绕以下四个大模块展开：

1. 前端输入
2. 底层仿真
3. 可视化
4. TSRL 模块

## 2. ATSISP 各大模块的内容与功能

### 2.1 前端输入模块

前端输入模块负责项目启动、场景选择、人工交互输入以及自定义场景文件创建，是用户与系统交互的入口层。

#### 主要组成

* `Start.bat`
* `Transportation_Semantic_Selector.py`
* `tkinter_scenario_selector.py`
* `Classic_Scenarios_Selection.py`
* `manual_network_creation.html`
* `loc_config.yaml`

#### 主要功能

* 启动项目主入口。
* 提供交通方式选择界面，包括道路交通、轨道交通和水运交通。
* 对道路交通进一步提供典型场景选择、自定义场景创建、场景列表查看等功能。
* 在仿真运行过程中提供人工交互弹窗，支持暂停仿真、输入人工指令、恢复仿真。
* 支持自定义场景文件的创建，包括路网命名、Netedit 打开、手动编辑 `nodes.xml`、`edges.xml`、`connections.xml`，以及调用 `netconvert` 合成 `.net.xml`。

#### 模块定位

该模块是 ATSISP 的“控制台”和“任务编排入口”。  
它本身不负责车辆运动学仿真和语义推理，而是负责收集用户意图，并把运行请求发送给后续仿真模块。

### 2.2 底层仿真模块

底层仿真模块负责场景加载、SUMO 联动、车辆状态推进、场景数据导出、轨迹回写和仿真主循环，是 ATSISP 的运行核心。

#### 主要组成

* `simModel/`
* `trafficManager/traffic_manager.py`
* `trafficManager/common/`
* `trafficManager/predictor/`
* `trafficManager/planner/`
* `networkFiles/`
* `Classic_Scenarios_Selection.py`

#### 主要功能

* 按场景名称读取对应的 `.net.xml`、`.rou.xml`、`.add.xml` 文件。
* 初始化 `simModel.egoTracking.model.Model`，建立与 SUMO 的连接。
* 按时间步推进仿真，包括车辆位置更新、场景状态导出、仿真终止判断等。
* 从 SUMO 场景中提取 `roadgraph`、车辆信息和路侧设施信息。
* 将环境状态传递给 `TrafficManager`。
* 调用预测器、决策器和规划器，生成新的车辆轨迹。
* 将规划得到的轨迹重新写回仿真模型，驱动车辆后续运动。

#### 模块定位

该模块是 ATSISP 的“执行引擎”。  
前端输入模块只负责告诉系统“运行哪个场景”，而底层仿真模块真正负责让场景跑起来。

### 2.3 可视化模块

可视化模块负责把场景运行状态、语义交互内容和 TSRL 推理结果展示给用户，帮助用户理解仿真过程。

#### 主要组成

* SUMO GUI
* `add/display.py`
* `message_history/`
* `assets_new/`
* `TSRL_txt/`
* `Transportation_Semantic_Selector.py`
* `tkinter_scenario_selector.py`

#### 主要功能

* 使用 Tkinter 图形界面展示交通方式选择、道路场景选择、自定义场景创建界面。
* 在道路仿真中可选择是否启用 `SUMO GUI`，直接显示底层交通流运行情况。
* 使用非阻塞窗口展示 `message_history/<场景名>/display_text.txt` 中的语义交互展示内容。
* 使用非阻塞窗口展示 TSRL 推理的详细过程，包括：
  * 触发规则
  * 读取的消息历史
  * 推理输入文件
  * 推理输出文件
  * 解析后的决策结果
* 通过 `message_history/` 和相关输出目录保存运行过程中的中间结果，便于调试与复盘。

#### 模块定位

该模块是 ATSISP 的“反馈层”和“解释层”。  
它既服务于演示，也服务于开发调试，使系统不仅能运行，还能把内部交互与推理过程展示出来。

### 2.4 TSRL 模块

TSRL 模块负责交通语义表示、消息驱动的规则匹配、语法解析、逻辑推理以及推理结果到车辆行为的映射，是 ATSISP 的语义智能核心。

#### 主要组成

* `TSRL_parsing_and_inference/`
* `TSRL_txt/`
* `TSRL_interaction/`
* `trafficManager/decision_maker/TSRL_decision_maker.py`
* `trafficManager/common/environment_adapter.py`

#### 主要功能

* 建立交通主体之间的语义通信机制，包括车车、车路、环境到车辆的信息传递。
* 定义通信消息体 `Message` 和述行词 `Performative`，支持 `Inform`、`Query`、`Request` 等语义交互行为。
* 将通信消息写入 `message_history/`，形成可供推理使用的消息历史。
* 从 `TSRL_txt/` 中读取规则、推理输入输出和展示文本。
* 使用 `TSRL_parsing_and_inference/TSRL.py` 进行词法分析、语法分析、解释执行和推理。
* 在 `TSRL_decision_maker.py` 中读取消息历史和规则，触发推理，得到决策结果。
* 将推理结果映射为车辆行为，例如保持车道、加速、减速、停车、进入路口等。

#### 模块定位

该模块是 ATSISP 的“语义决策层”。  
它区别于传统仅依赖数值状态的控制逻辑，能够基于交通语义消息和规则做出行为决策。

## 3. 四大模块之间的运行逻辑

### 3.1 总体运行链路

ATSISP 的总体运行链路可以概括为：

`前端输入 -> 底层仿真 -> TSRL 模块 -> 底层仿真 -> 可视化`

更具体地说：

1. 用户在前端输入模块中选择交通方式和场景。
2. 系统根据场景配置启动底层仿真。
3. 底层仿真持续导出环境状态给 `TrafficManager`。
4. `TrafficManager` 一方面调用预测与规划模块，另一方面调用 TSRL 模块进行语义推理和决策。
5. TSRL 推理结果转化为车辆行为与轨迹。
6. 新轨迹写回仿真模型，驱动下一轮仿真。
7. 交互消息、推理内容和仿真状态通过可视化模块反馈给用户。

### 3.2 从启动入口到场景选择的逻辑

项目运行从 `Start.bat` 开始：

1. `Start.bat` 检查 Python 环境是否存在。
2. 若环境正常，则启动 `Transportation_Semantic_Selector.py`。
3. `Transportation_Semantic_Selector.py` 打开总界面，用户可选择：
   * 道路交通场景
   * 轨道交通场景
   * 水运交通场景
4. 若选择道路交通，则通过 `subprocess.Popen` 启动 `tkinter_scenario_selector.py`。
5. 若选择轨道或水运，则直接打开对应 AnyLogic 工程文件。

这里可以看出，道路场景是当前项目中最完整、最深度集成 ATSISP 仿真链路的一部分。

### 3.3 道路交通场景的前端输入逻辑

进入 `tkinter_scenario_selector.py` 后，系统提供三个入口：

1. 典型交通场景
2. 自定义交通场景
3. 查看场景列表

其中：

* 典型交通场景会进一步调用 `Classic_Scenarios_Selection.py`，运行预设场景。
* 自定义交通场景用于创建新的路网文件和辅助配置文件。
* 查看场景列表会以命令行形式读取 `Classic_Scenarios_Selection.py -l` 的输出结果。

典型场景的启动方式是：

* 前向碰撞预警场景
* 车辆-RSU 交互场景
* 人车加速交互场景
* 车辆交互场景

当用户点击具体场景按钮后：

1. `tkinter_scenario_selector.py` 通过命令行参数 `-s <scenario_name>` 启动 `Classic_Scenarios_Selection.py`。
2. 若勾选 `SUMO GUI`，则附加图形界面运行参数。
3. 同时弹出人工指令输入对话框，用于后续暂停仿真和发送人工指令。

### 3.4 道路交通场景的仿真主循环逻辑

`Classic_Scenarios_Selection.py` 是道路交通典型场景的主驱动程序。其运行逻辑如下：

1. 根据场景名确定对应的路网文件、路由文件和附加文件路径。
2. 创建 `Model` 实例，建立与 SUMO 的连接。
3. 创建 `TrafficManager` 实例，作为感知、预测、决策、规划的统一调度器。
4. 清理消息历史文件和显示文本文件。
5. 进入主循环 `while not model.tpEnd`。

在主循环内部，系统反复执行以下过程：

1. 检查是否存在暂停信号文件。
2. 检查是否存在人工输入信号文件，若有则读入并交给 `planner._handle_user_input(...)`。
3. 调用 `model.moveStep()` 推进 SUMO 一步。
4. 周期性读取 `model.exportSce()` 的场景导出结果。
5. 将导出的 `roadgraph`、车辆、设施信息传给 `TrafficManager.plan(...)`。
6. `TrafficManager` 完成感知、预测、决策和规划。
7. 将生成的轨迹通过 `model.setTrajectories(...)` 回写。
8. 调用 `model.updateVeh()` 完成车辆状态更新。

因此，底层仿真模块和上层控制模块并不是分离运行的，而是在每个仿真步长上持续耦合运行。

### 3.5 TrafficManager 对四类模块的衔接逻辑

`TrafficManager` 是 ATSISP 中最关键的中间层。  
它将前端输入、底层仿真、TSRL 推理和可视化贯通起来。

其职责可以概括为：

1. 从底层仿真中提取当前车辆、主车、RSU、历史轨迹和静态障碍物。
2. 处理来自用户的人工输入，例如方向或加速指令。
3. 管理通信模块，触发车辆与 RSU、环境之间的语义消息交换。
4. 生成观测对象 `Observation`。
5. 调用预测模块预测其他车辆行为。
6. 调用 `TSRL_decision_maker` 生成语义决策。
7. 调用轨迹规划模块生成可执行轨迹。
8. 将显示文本和推理结果交给可视化模块展示。

从架构上说，`TrafficManager` 是整个项目的“运行中枢”。

### 3.6 TSRL 模块与底层仿真的交互逻辑

TSRL 模块并不是独立离线运行，而是嵌入在仿真回路中的在线决策模块。

其交互逻辑如下：

1. 通信模块在仿真过程中持续生成语义消息。
2. 消息被记录到 `message_history/<场景名>/message_<vehicle_id>_history.txt`。
3. `TSRL_decision_maker.py` 在决策时读取对应车辆的消息历史。
4. 同时读取规则文件，将“交通事实 + 匹配规则 + ASK 查询”组装成推理输入。
5. 调用 `TSRL.main(...)` 执行词法分析、语法分析、解释和推理。
6. 将推理输出解析为具体行为结果。
7. 将行为结果映射为 `Behaviour`，如 `KL`、`AC`、`DC`、`STOP` 等。
8. 决策结果再传递给规划器，由规划器生成轨迹。

因此，TSRL 模块不直接控制 SUMO，而是通过“消息历史 -> 规则推理 -> 行为决策 -> 轨迹规划”的链条间接影响仿真结果。

### 3.7 可视化模块与运行主链路的关系

可视化模块主要在两个层面参与运行：

#### 1. 前端界面可视化

* 负责展示交通方式入口、场景入口、自定义场景创建步骤。
* 负责展示人工指令输入弹窗。

#### 2. 运行过程可视化

* 若启用 `SUMO GUI`，则由 SUMO 直接展示车辆运行过程。
* 通信模块将交互内容写入 `display_text.txt`，再通过非阻塞窗口显示。
* TSRL 决策模块将推理细节写入展示文件，并通过非阻塞窗口显示。

这意味着 ATSISP 的可视化不仅仅是“看车辆怎么动”，还包括“看车辆为何这么动”。

## 4. ATSISP 的模块协同总结

从工程结构上看，ATSISP 可以理解为一个“以交通仿真为底座、以语义交互为驱动、以 TSRL 推理为核心、以图形展示为外显”的系统。

四个大模块的分工如下：

* 前端输入模块：负责收集用户意图，决定运行什么场景、何时输入人工指令。
* 底层仿真模块：负责加载场景、推进 SUMO、输出环境状态、执行轨迹。
* 可视化模块：负责展示选择过程、仿真过程、交互内容和推理结果。
* TSRL 模块：负责把语义消息和交通规则转化为车辆决策。

四者之间的主逻辑关系如下：

1. 前端输入触发仿真。
2. 底层仿真生成环境状态。
3. TSRL 模块基于语义消息与规则做推理决策。
4. 决策结果返回底层仿真执行。
5. 可视化模块将过程和结果反馈给用户。

## 5. 简化运行流程图

```text
Start.bat
  -> Transportation_Semantic_Selector.py
     -> 选择交通方式
        -> 道路交通
           -> tkinter_scenario_selector.py
              -> 选择典型场景
                 -> Classic_Scenarios_Selection.py
                    -> Model(SUMO)
                    -> TrafficManager
                       -> 感知/预测
                       -> 语义通信
                       -> TSRL推理决策
                       -> 轨迹规划
                    -> 回写轨迹到仿真
                    -> SUMO GUI / 文本展示 / 推理展示
        -> 轨道交通
           -> 打开 AnyLogic 工程
        -> 水运交通
           -> 打开 AnyLogic 工程
```

## 6. 说明

本文档重点描述 ATSISP 当前代码中已经实现并贯通的主链路。  
其中，道路交通场景的“前端输入-底层仿真-可视化-TSRL模块”闭环最为完整；轨道交通和水运交通目前更多表现为通过前端入口打开对应 AnyLogic 模型文件。
