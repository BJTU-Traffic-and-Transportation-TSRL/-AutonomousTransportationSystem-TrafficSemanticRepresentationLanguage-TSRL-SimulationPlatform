# EN-AutonomousTransportationSystem-TrafficSemanticRepresentationLanguage-TSRL-SimulationPlatform
# CN-自主式交通系统-交通语义表示语言-TSRL-仿真平台
## 项目简介

本项目是基于LimSim的自主交通语义交互仿真平台(Autonomous Transportation Semantic Interaction Simulation Platform, ATSISP)。它扩展了LimSim的功能，加入了车辆间以及车辆与路侧单元(RSU)的语义通信交互能力，并集成了基于交通语义表示语言(TSRL)的推理引擎，以模拟和分析交通场景下的复杂交互行为。

项目主要使用Python编写，依赖SUMO (Simulation of Urban MObility)进行底层交通仿真。

## 核心组件与功能

1.  **交通仿真核心 (simModel)**: 基于SUMO，负责构建和管理交通仿真环境，包括路网、车辆、轨迹等。支持实时仿真和回放模式 (`ModelExample.py`, `ReplayExample.py`)。

2.  **交通管理与规划 (trafficManager)**: 负责仿真中车辆的行为更新、决策和轨迹规划。核心是 `TrafficManager` 类，它整合了感知、预测、决策和规划模块。

3.  **语义交互层 (TSRL_interaction)**:
    *   实现了基于FIPA ACL标准的通信模型，定义了 `Message` 类和 `Performative` 枚举来表示交互消息体和行为类型（如Inform, Query, Request, Accept等）。
    *   `VehicleCommunicator` 和 `RSUCommunicator` 类负责处理车辆和RSU的通信逻辑，包括消息的发送、接收和处理。它们通过 `CommunicationManager` 进行消息路由和管理。

4.  **时空间推理引擎 (TSRL_representation)**:
    *   实现了一阶逻辑的知识库 (`FolKB`) 和推理算法（前向链接 `fol_fc_ask` 和反向链接 `fol_bc_ask`）。
    *   包含词法分析器(`Scanner`)、语法分析器(`Parser`)和解释器(`Interpreter`)，共同构成了TSIL (Temporal Spatial Interaction Language) 语言的处理流程，用于定义和执行语义交互规则。

5.  **应用场景示例**:
    *   `ForwardCollisionWarning.py`: 实现了前向碰撞预警场景，展示了如何在特定交通场景下应用语义交互和推理能力。该示例会在仿真开始时向TSRL推理引擎写入预定义的规则和查询。
    *   `Vehicle_RSU_Interacting.py`: 实现了车辆-RSU交互场景。
    *   `Human_Vehicle_Interacting.py`: 实现了人车加速交互场景。
    *   `Vehicle_Vehicle_Interacting.py`: 实现了车辆交互场景。

6.  **场景选择器 (`tkinter_scenario_selector.py`)**:
    *   提供了图形用户界面（GUI），方便用户快速选择和启动不同的仿真场景。
    *   包含典型交通场景和自定义交通场景的选项。

## 项目结构

*   `simModel/`: 交通仿真模型核心代码
*   `trafficManager/`: 交通管理与车辆规划逻辑
*   `TSRL_interaction/`: 语义交互通信模块
*   `TSRL_representation/`: 时空间推理引擎与TSIL语言实现
*   `networkFiles/`: 存放各种仿真场景的路网文件 (`.net.xml`) 和路由文件 (`.rou.xml`)
*   `message_history/`: 存放仿真过程中生成的车辆通信消息历史记录
*   `assets/`: 存放项目相关的图片资源
*   `evaluation/`, `logger/`, `utils/`: 辅助功能模块

## 开发与运行

### 环境要求

*   3.9.0 <= Python <= 3.11.0
*   SUMO >= 1.15.0
*   依赖库: `dearpygui`, `matplotlib`, `numpy`, `pandas`, `pynput`, `PyYAML`, `rich`, `sumolib`, `traci` (详见 `requirements.txt`)

### Quick Start

1.  环境配置
    1. Python
    
        **推荐版本**: 3.9.0 <= Python <= 3.11.0
    2. SUMO
        1. 安装SUMO（Windows）
        
            **推荐版本**: SUMO >= 1.15.0 (Recommended version: 1.15.0)
            
            *该版本的SUMO已包含在项目文件夹中，无需额外安装(`sumo-1.15.0`)。*
        2. 配置环境变量
            1. 将SUMO的 `bin/` 目录添加到**系统变量and环境变量**的 `PATH` 中。
            2. 新建并设置环境变量 `SUMO_HOME` 为SUMO安装目录 (e.g., `export SUMO_HOME=./sumo-1.15.0/`)
            3. 验证安装
                1. 打开命令行终端
                2. 输入 `sumo --version`
                3. 确认输出显示SUMO版本为1.15.0
                4. 输入 `sumo-gui`
                5. 确认SUMO GUI窗口打开，显示空的交通场景
    3. Anylogic
        1. 安装Anylogic（Windows）
        
             https://www.anylogic.com/downloads/
            
            **推荐版本**: 8.9.6 Personal Learning Edition（需手动安装）
        2. 配置Pypeline：在Anylogic中使用Python
            1. https://github.com/the-anylogic-company/AnyLogic-Pypeline/releases 下载Pypeline.jar v1.9.6，并解压到本地

                *所需的Pypeline.jar已包含在项目文件夹中，无需额外安装(`.venv\Pypeline.jar`)。*

            2. 打开已经安装好的Anylogic，找到”**面板（Palette）**“>>左下角的“+”>>**管理库……**

                ![Pypeline2Anylogic_step1](assets_new/Pypeline2Anylogic_1.png)
            3. 点击“**添加**”，找到下载好的Pypeline.jar文件，添加到Anylogic中
                ![Pypeline2Anylogic_step2](assets_new/Pypeline2Anylogic_2.png)
            4. 点击“**确定**”，Pypeline库就会被添加到Anylogic中
2.  安装所需依赖: 

    `cd ATSISP
pip install -r requirements.txt`

    `pip install -r requirements.txt`

### Instructions

1.  **交通语义交互场景选择界面**: 双击批处理文件`Start.bat`，即可打开交通语义交互场景选择界面。

    ![scenario_selector](assets_new/Start.png)
2.  **选择所需的交通方式**: 点击场景选择器中的交通方式按钮，即可启动对应的交通语义交互场景。
    1. 道路交通场景

        点击**道路交通场景**按钮，即可启动道路交通场景的仿真。

        在新弹出的窗口中，用户可以选择不同的场景类型，如**典型交通场景**、**自定义交通场景**等。

        ![roadsys_scenario](assets_new/RoadSys_Transportation_Senarios.png)

        * 典型交通场景
        
            点击**典型交通场景**按钮，即可启动典型交通场景的仿真。
            
            在新弹出的窗口中，用户可以选择不同的典型交通场景，如**前向碰撞预警场景**、**车辆-RSU交互场景**等。
            
            ![typical_scenario](assets_new/typical_roadway_senarios.png)
    2. 轨道交通场景
        
        点击**轨道交通场景**按钮，即可启动轨道交通场景的仿真。
        
        在新弹出的窗口中，用户可以选择不同的轨道交通场景，具体的场景介绍可以在弹窗文字中进行查看。
        
        ![railway_scenario](assets_new/railway_senarios.png)
    3. 水运交通场景
        
        点击**水运交通场景**按钮，即可启动水运交通场景的仿真。

### 自定义场景创建

通过场景选择器的"自定义交通场景"选项，用户可以创建新的仿真场景：

1.  创建路网文件（使用Netedit工具）
2.  创建路由文件
3.  添加其他必要的配置文件

## 开发约定与实践

*   **代码风格**: 遵循Python通用编码规范，部分文件包含中文注释以解释功能和逻辑。核心模块如 `trafficManager` 和 `TSRL_interaction` 有较详细的英文文档字符串 (docstring) 描述类和方法的功能。
*   **模块化**: 项目结构清晰，将仿真、规划、交互、推理等功能分离到不同模块和目录下，便于维护和扩展。
*   **通信机制**: 车辆和RSU通过 `CommunicationManager` 进行消息传递，消息内容遵循FIPA ACL标准，增强了交互的规范性和可扩展性。消息历史记录被保存到 `message_history/` 目录下，便于调试和分析。
