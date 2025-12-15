import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import sys
import os
import re
import webbrowser
import threading
import time
import json
import logger

# 设置日志
log = logger.get_logger(__name__)

# 获取项目根目录
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
# 导入地址配置文件
from utils.load_config import load_config
loc_config = load_config("E:\Analysis and Inference\Analysis and Inference\Autonomous Transportation Semantic Interaction Simulation Platform-2025.12-结题联合仿真 - under version\ATSISP\loc_config.yaml")

class ManualInstructionDialog:
    def __init__(self, parent, scenario_name, simulation_process):
        self.parent = parent
        self.scenario_name = scenario_name
        self.simulation_process = simulation_process
        self.instruction = ""
        self.mode = "view"  # 初始为查看模式，支持 'view' 和 'input' 两种模式
        
        # 创建弹窗
        self.dialog = tk.Toplevel(parent)
        self.dialog.title(f"人工指令输入 - {scenario_name}")
        self.dialog.geometry("500x300")
        self.dialog.resizable(False, False)
        self.dialog.transient(parent)
        self.dialog.grab_set()
        # 设置窗口置顶
        self.dialog.attributes('-topmost', True)
        # 设置弹窗样式
        self.dialog.configure(bg='#f0f0f0')
        # 创建UI组件
        self.create_widgets()
        # 绑定窗口关闭事件
        self.dialog.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def create_widgets(self):
        # 清除所有现有组件
        for widget in self.dialog.winfo_children():
            widget.destroy()
        
        # 标题标签
        title_label = tk.Label(
            self.dialog,
            text="人工指令输入",
            font=('Arial', 14, 'bold'),
            fg='#0066cc',
            bg='#f0f0f0'
        )
        title_label.pack(pady=20)
        
        if self.mode == "view":
            # 查看模式：使用较小的窗口尺寸
            self.dialog.geometry("500x300")
            info_label = tk.Label(
                self.dialog,
                text="仿真正在运行中，点击下方按钮进入人工交互模式：",
                font=('Arial', 10),
                bg='#f0f0f0'
            )
            info_label.pack(pady=20)
            
            button_frame = tk.Frame(self.dialog, bg='#f0f0f0')
            button_frame.pack(pady=30)
            
            # 进行人工交互按钮
            interaction_btn = tk.Button(
                button_frame,
                text="进行人工交互",
                font=('Arial', 12),
                width=15,
                command=self.switch_to_input_mode
            )
            interaction_btn.pack()
        elif self.mode == "input":
            # 输入模式：增加窗口高度以容纳所有控件
            self.dialog.geometry("500x400")
            info_label = tk.Label(
                self.dialog,
                text="请输入人工指令，仿真已暂停：",
                font=('Arial', 10),
                bg='#f0f0f0'
            )
            info_label.pack(pady=10)
            # 文本输入框
            self.text_frame = tk.Frame(self.dialog, bg='#f0f0f0')
            self.text_frame.pack(pady=10, padx=20, fill='both', expand=True)
            self.instruction_text = tk.Text(
                self.text_frame,
                font=('Arial', 12),
                wrap='word',
                width=50,
                height=8
            )
            self.instruction_text.pack(side='left', fill='both', expand=True)
            # 滚动条
            scrollbar = tk.Scrollbar(self.text_frame, command=self.instruction_text.yview)
            scrollbar.pack(side='right', fill='y')
            self.instruction_text.configure(yscrollcommand=scrollbar.set)
            # 绑定键盘事件
            self.instruction_text.bind('<Key>', self.on_key_press)
            # 按钮框架
            button_frame = tk.Frame(self.dialog, bg='#f0f0f0')
            button_frame.pack(pady=20)
            # 确认完成输入按钮
            confirm_btn = tk.Button(
                button_frame,
                text="确认完成输入",
                font=('Arial', 12),
                width=15,
                command=self.on_confirm
            )
            confirm_btn.pack(side='left', padx=10)
            
            # 取消输入，返回按钮
            cancel_btn = tk.Button(
                button_frame,
                text="取消输入，返回",
                font=('Arial', 12),
                width=15,
                command=self.on_cancel
            )
            cancel_btn.pack(side='right', padx=10)
    
    def on_key_press(self, input_content):
        # 处理完整的键盘输入，来自点击"确认完成输入"按钮后的文本输入
        if isinstance(input_content, str):
            # 输入内容是字符串，直接使用
            key_input = input_content.strip()
        else:
            # 输入内容是事件对象，不处理（保持向后兼容）
            return
        # 如果输入有效，保存到文件供仿真进程读取
        if key_input:
            # 使用与其他信号文件相同的命名模式
            key_input_file = os.path.join(PROJECT_ROOT, "trafficManager", "planner", "input_signal", f"input_{self.scenario_name}.signal")
            with open(key_input_file, 'w') as f:
                f.write(key_input)
            log.info(f"Sent keyboard text input {key_input} to {key_input_file}")
    
    def switch_to_input_mode(self):
        # 切换到输入模式
        self.mode = "input"
        self.create_widgets()
        # 暂停仿真
        self.pause_simulation()
    
    def switch_to_view_mode(self):
        # 切换到查看模式
        self.mode = "view"
        self.create_widgets()
        # 恢复仿真
        self.resume_simulation()
    
    def on_confirm(self):
        # 获取输入内容
        self.instruction = self.instruction_text.get('1.0', 'end-1c').strip()
        # 将文本输入作为键盘输入处理
        if self.instruction:
            # 调用on_key_press方法处理完整的文本输入
            self.on_key_press(self.instruction)
        # 切换回查看模式
        self.switch_to_view_mode()
    
    def on_cancel(self):
        # 切换回查看模式
        self.switch_to_view_mode()
    
    def on_close(self):
        # 窗口关闭时恢复仿真
        self.resume_simulation()
        self.dialog.destroy()
    
    def save_instruction(self):
        # 保存指令到文件
        instruction_file = os.path.join(PROJECT_ROOT, f"instruction_{self.scenario_name}.txt")
        with open(instruction_file, 'w', encoding='utf-8') as f:
            f.write(self.instruction)
        log.info(f"Saved manual instruction to {instruction_file}")
    
    def pause_simulation(self):
        # 向仿真进程发送暂停信号
        pause_file = os.path.join(PROJECT_ROOT, "trafficManager", "planner", "pause_resume_signal", f"pause_{self.scenario_name}.signal")
        with open(pause_file, 'w') as f:
            f.write("pause")
        log.info(f"Sent pause signal to {pause_file}")
    
    def resume_simulation(self):
        # 向仿真进程发送恢复信号
        resume_file = os.path.join(PROJECT_ROOT, "trafficManager", "planner", "pause_resume_signal", f"resume_{self.scenario_name}.signal")
        with open(resume_file, 'w') as f:
            f.write("resume")
        log.info(f"Sent resume signal to {resume_file}")

class ScenarioSelector:
    def __init__(self, root):
        self.root = root
        self.root.title("交通语义交互场景选择器")
        self.root.geometry("600x500")
        self.root.resizable(False, False)
        # 设置样式
        self.setup_styles()
        # 是否使用SUMO GUI的标志
        self.use_sumo_gui = tk.BooleanVar(value=False)
        # 保存正在运行的仿真进程
        self.running_processes = {}
        
        # 创建初始界面
        self.create_initial_widgets()
        
    def setup_styles(self):
        """设置界面样式"""
        self.root.configure(bg='#f0f0f0')
        
    def create_initial_widgets(self):
        """创建初始界面组件"""
        # 清除所有现有组件
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # 标题
        title_frame = tk.Frame(self.root, bg='#f0f0f0')
        title_frame.pack(pady=20, padx=20, fill='x')
        
        title_label = tk.Label(
            title_frame, 
            text="道路交通语义交互场景", 
            font=('Arial', 16, 'bold'),
            fg='#0066cc',
            bg='#f0f0f0'
        )
        title_label.pack()
        
        # 分隔线
        separator = ttk.Separator(self.root, orient='horizontal')
        separator.pack(fill='x', padx=20)
        
        # 说明文本
        info_frame = tk.Frame(self.root, bg='#f0f0f0')
        info_frame.pack(pady=30, padx=20, fill='x')
        
        info_label = tk.Label(
            info_frame,
            text="请选择场景类型:",
            font=('Arial', 12, 'bold'),
            bg='#f0f0f0'
        )
        info_label.pack(anchor='w')
        
        # 按钮区域
        button_frame = tk.Frame(self.root, bg='#f0f0f0')
        button_frame.pack(pady=30, padx=20, fill='both', expand=True)
        
        # 创建按钮样式
        button_style = {
            'font': ('Arial', 12),
            'width': 30,
            'height': 2,
            'bg': '#ffffff',
            'fg': '#333333',
            'relief': 'raised',
            'borderwidth': 1
        }
        
        # 典型交通场景按钮
        typical_btn = tk.Button(
            button_frame,
            text="典型交通场景",
            command=self.show_typical_scenarios,
            **button_style
        )
        typical_btn.pack(pady=10)
        
        # 自定义交通场景按钮
        custom_btn = tk.Button(
            button_frame,
            text="自定义交通场景",
            command=self.run_custom_scenario,
            **button_style
        )
        custom_btn.pack(pady=10)
        
        # 查看场景列表按钮
        list_btn = tk.Button(
            button_frame,
            text="查看场景列表",
            command=self.show_scenario_list,
            **button_style
        )
        list_btn.pack(pady=10)
        
        # 说明区域
        help_frame = tk.Frame(self.root, bg='#f0f0f0')
        help_frame.pack(pady=20, padx=20, fill='x')
        
        help_title = tk.Label(
            help_frame,
            text="说明:",
            font=('Arial', 9, 'bold'),
            fg='#666666',
            bg='#f0f0f0'
        )
        help_title.pack(anchor='w')
        
        help_text = tk.Label(
            help_frame,
            text="• 典型交通场景: 包含预设的常见交通场景\n• 自定义交通场景: 用户自定义的交通场景\n• 查看场景列表: 显示所有可用场景的详细信息",
            font=('Arial', 8),
            fg='#888888',
            bg='#f0f0f0',
            justify='left'
        )
        help_text.pack(anchor='w', pady=(5, 0))
        
    def show_typical_scenarios(self):
        """显示典型交通场景列表"""
        # 清除所有现有组件
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # 标题
        title_frame = tk.Frame(self.root, bg='#f0f0f0')
        title_frame.pack(pady=20, padx=20, fill='x')
        
        title_label = tk.Label(
            title_frame, 
            text="道路交通语义交互场景", 
            font=('Arial', 16, 'bold'),
            fg='#0066cc',
            bg='#f0f0f0'
        )
        title_label.pack()
        
        # 返回按钮
        back_btn = tk.Button(
            title_frame,
            text="返回",
            command=self.create_initial_widgets,
            font=('Arial', 8),
            width=8,
            height=1
        )
        back_btn.pack(side='right')
        
        # 分隔线
        separator = ttk.Separator(self.root, orient='horizontal')
        separator.pack(fill='x', padx=20)
        
        # 说明文本
        info_frame = tk.Frame(self.root, bg='#f0f0f0')
        info_frame.pack(pady=10, padx=20, fill='x')
        
        info_label = tk.Label(
            info_frame,
            text="请选择要运行的典型交通场景:",
            font=('Arial', 10),
            bg='#f0f0f0'
        )
        info_label.pack(anchor='w')
        
        # 按钮区域
        button_frame = tk.Frame(self.root, bg='#f0f0f0')
        button_frame.pack(pady=20, padx=20, fill='both', expand=True)
        
        # 创建按钮样式
        button_style = {
            'font': ('Arial', 10),
            'width': 40,
            'height': 2,
            'bg': '#ffffff',
            'fg': '#333333',
            'relief': 'raised',
            'borderwidth': 1
        }
        
        # 前向碰撞预警场景按钮
        btn1 = tk.Button(
            button_frame,
            text="1. 前向碰撞预警场景",
            command=self.run_forward_collision_warning,
            **button_style
        )
        btn1.pack(pady=5)
        
        # 车辆-RSU交互场景按钮
        btn2 = tk.Button(
            button_frame,
            text="2. 车辆-RSU交互场景",
            command=self.run_vehicle_rsu_interaction,
            **button_style
        )
        btn2.pack(pady=5)
        
        # 人车加速交互场景按钮
        btn3 = tk.Button(
            button_frame,
            text="3. 人车加速交互场景",
            command=self.run_human_vehicle_interaction,
            **button_style
        )
        btn3.pack(pady=5)
        
        # 车辆交互场景按钮
        btn4 = tk.Button(
            button_frame,
            text="4. 车辆交互场景",
            command=self.run_vehicle_vehicle_interaction,
            **button_style
        )
        btn4.pack(pady=5)
        
        # SUMO GUI选项
        options_frame = tk.Frame(self.root, bg='#f0f0f0')
        options_frame.pack(pady=10, padx=20, fill='x')
        
        gui_check = tk.Checkbutton(
            options_frame,
            text="使用SUMO GUI界面",
            variable=self.use_sumo_gui,
            font=('Arial', 9),
            bg='#f0f0f0'
        )
        gui_check.pack(anchor='w')
        
        # 说明区域
        help_frame = tk.Frame(self.root, bg='#f0f0f0')
        help_frame.pack(pady=20, padx=20, fill='x')
        
        help_text = tk.Label(
            help_frame,
            text="• 点击按钮启动对应的仿真场景\n• 每个场景将在新窗口中运行\n• 请确保SUMO已正确安装并配置",
            font=('Arial', 8),
            fg='#888888',
            bg='#f0f0f0',
            justify='left'
        )
        help_text.pack(anchor='w', pady=(5, 0))
        
    def run_scenario(self, scenario_key, use_gui=False):
        """运行指定场景"""
        try:
            # 构建命令行参数
            cmd = [sys.executable, loc_config["LOC_CLASSIC_SCENARIOS"], "-s", scenario_key]
            if use_gui:
                cmd.append("--sumo-gui")
            # 使用subprocess.Popen启动新进程
            process = subprocess.Popen(cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)
            # 触发人工指令输入弹窗
            self.show_manual_instruction_dialog(scenario_key, process)
            # 保存进程到字典
            self.running_processes[scenario_key] = process
            # 显示提示信息
            messagebox.showinfo("提示", f"已启动场景: {scenario_key}\n仿真将在您输入人工指令后开始运行。")
            
        except Exception as e:
            messagebox.showerror("错误", f"运行场景时出错: {str(e)}")
    
    def show_manual_instruction_dialog(self, scenario_key, process):
        """显示人工指令输入弹窗"""
        # 创建并显示人工指令输入弹窗，不阻塞主进程
        dialog = ManualInstructionDialog(self.root, scenario_key, process)
        # 不等待弹窗关闭，让仿真继续运行
        # 弹窗将持续显示，直到用户关闭它
        print(f"Created manual instruction dialog for {scenario_key}")
            
    def run_forward_collision_warning(self):
        """运行前向碰撞预警场景"""
        self.run_scenario("Forward_Collision_Warning", self.use_sumo_gui.get())
        
    def run_vehicle_rsu_interaction(self):
        """运行车辆-RSU交互场景"""
        self.run_scenario("Vehicle_RSU_Interacting", self.use_sumo_gui.get())
        
    def run_human_vehicle_interaction(self):
        """运行人车加速交互场景"""
        self.run_scenario("Human_Vehicle_Interacting", self.use_sumo_gui.get())
        
    def run_vehicle_vehicle_interaction(self):
        """运行车辆交互场景"""
        self.run_scenario("Vehicle_Vehicle_Interacting", self.use_sumo_gui.get())
        
    def show_scenario_list(self):
        """显示场景列表"""
        try:
            # 运行Classic_Scenarios_Selection.py的列表功能
            result = subprocess.run(
                [sys.executable, loc_config["LOC_CLASSIC_SCENARIOS"], "-l"],
                capture_output=True,
                text=True,
                cwd=PROJECT_ROOT
            )
            
            # 创建新窗口显示场景列表
            list_window = tk.Toplevel(self.root)
            list_window.title("可用场景列表")
            list_window.geometry("600x400")
            list_window.resizable(True, True)
            
            # 创建文本框和滚动条
            text_frame = tk.Frame(list_window)
            text_frame.pack(fill='both', expand=True, padx=10, pady=10)
            
            text_widget = tk.Text(text_frame, wrap='word', font=('Courier', 9))
            scrollbar = tk.Scrollbar(text_frame, command=text_widget.yview)
            text_widget.configure(yscrollcommand=scrollbar.set)
            
            text_widget.pack(side='left', fill='both', expand=True)
            scrollbar.pack(side='right', fill='y')
            
            # 插入场景列表信息
            if result.returncode == 0:
                text_widget.insert('1.0', result.stdout)
            else:
                text_widget.insert('1.0', f"获取场景列表失败:\n{result.stderr}")
            
            text_widget.configure(state='disabled')
            
            # 关闭按钮
            close_btn = tk.Button(list_window, text="关闭", command=list_window.destroy)
            close_btn.pack(pady=5)
            
        except Exception as e:
            messagebox.showerror("错误", f"显示场景列表时出错: {str(e)}")
    
    def run_custom_scenario(self):
        """运行自定义场景"""
        # 创建新窗口
        self.custom_window = tk.Toplevel(self.root)
        self.custom_window.title("自定义交通场景")
        self.custom_window.geometry("500x600")  # 增加窗口高度以容纳新内容
        self.custom_window.resizable(False, False)
        
        # 设置窗口样式
        self.custom_window.configure(bg='#f0f0f0')
        
        # 标题
        title_frame = tk.Frame(self.custom_window, bg='#f0f0f0')
        title_frame.pack(pady=20, padx=20, fill='x')
        
        title_label = tk.Label(
            title_frame, 
            text="自定义交通场景", 
            font=('Arial', 14, 'bold'),
            fg='#0066cc',
            bg='#f0f0f0'
        )
        title_label.pack()
        
        # 返回按钮
        back_btn = tk.Button(
            title_frame,
            text="返回",
            command=self.custom_window.destroy,  # 关闭当前窗口
            font=('Arial', 8),
            width=8,
            height=1
        )
        back_btn.pack(side='right')
        
        # 分隔线
        separator = ttk.Separator(self.custom_window, orient='horizontal')
        separator.pack(fill='x', padx=20)
        
        # 按钮区域
        self.button_frame = tk.Frame(self.custom_window, bg='#f0f0f0')
        self.button_frame.pack(pady=20, padx=20, fill='both', expand=True)
        
        # 创建按钮样式，统一所有按钮的尺寸
        button_style = {
            'font': ('Arial', 10),
            'width': 30,
            'height': 2,
            'bg': '#ffffff',
            'fg': '#333333',
            'relief': 'raised',
            'borderwidth': 1
        }
        
        # 创建路网文件按钮（初始可点击）
        self.create_net_btn = tk.Button(
            self.button_frame,
            text="1. 创建路网文件",
            state=tk.NORMAL,  # 初始可点击
            command=self.create_network_file,
            **button_style
        )
        self.create_net_btn.pack(pady=10)
        
        # 创建路由文件按钮（初始不可点击）
        self.create_route_btn = tk.Button(
            self.button_frame,
            text="2. 创建路由文件",
            state=tk.DISABLED,  # 初始不可点击
            **button_style
        )
        self.create_route_btn.pack(pady=10)
        
        # 其他添加文件按钮（初始不可点击）
        self.create_other_btn = tk.Button(
            self.button_frame,
            text="3. 其他添加文件",
            state=tk.DISABLED,  # 初始不可点击
            **button_style
        )
        self.create_other_btn.pack(pady=10)
        
        # 说明区域
        help_frame = tk.Frame(self.custom_window, bg='#f0f0f0')
        help_frame.pack(pady=20, padx=20, fill='x')
        
        help_text = tk.Label(
            help_frame,
            text="• 请按顺序创建文件\n• 创建路网文件后才能创建其他文件",
            font=('Arial', 8),
            fg='#888888',
            bg='#f0f0f0',
            justify='left'
        )
        help_text.pack(anchor='w', pady=(5, 0))

    def create_network_file(self):
        """创建路网文件"""
        # 清除按钮区域的内容
        for widget in self.button_frame.winfo_children():
            widget.destroy()
        
        # 路网命名标题
        name_label = tk.Label(
            self.button_frame,
            text="请为路网文件命名：",
            font=('Arial', 10, 'bold'),
            bg='#f0f0f0'
        )
        name_label.pack(pady=(20, 10))
        
        # 路网名称输入框
        self.network_name_var = tk.StringVar()
        self.network_name_entry = tk.Entry(
            self.button_frame,
            textvariable=self.network_name_var,
            font=('Arial', 10),
            width=30
        )
        self.network_name_entry.pack(pady=5)
        
        # 验证函数：只允许字母、数字、下划线和空格
        def validate_network_name(char):
            return re.match(r'^[a-zA-Z0-9_ ]*$', char) is not None
        
        # 注册验证函数
        vcmd = (self.network_name_entry.register(validate_network_name), '%S')
        self.network_name_entry.config(validate='key', validatecommand=vcmd)
        
        # 确认按钮
        confirm_btn = tk.Button(
            self.button_frame,
            text="确认",
            command=self.confirm_network_name,
            font=('Arial', 10),
            width=15,
            height=1
        )
        confirm_btn.pack(pady=10)

    def launch_netedit(self, net_file_path=None):
        """启动Netedit程序"""
        # 创建提示窗口
        notice_window = tk.Toplevel(self.custom_window)
        notice_window.title("提示")
        notice_window.geometry("400x150")
        notice_window.resizable(False, False)
        
        # 居中显示
        notice_window.transient(self.custom_window)
        notice_window.grab_set()
        
        # 提示文本
        label = tk.Label(notice_window, text="请在当前项目的networkFiles中的路网同名文件夹中创建所需路网文件", 
                        font=('Arial', 10), wraplength=380, justify='left')
        label.pack(pady=20)
        
        # 确认按钮
        def on_confirm():
            notice_window.destroy()
            self._launch_netedit_internal(net_file_path)
        
        confirm_btn = tk.Button(notice_window, text="确认", command=on_confirm, 
                               font=('Arial', 10), width=10)
        confirm_btn.pack(pady=10)
    
    def _launch_netedit_internal(self, net_file_path=None):
        """实际启动Netedit程序的内部方法"""
        netedit_path = loc_config["LOC_SUMO_NETEDIT"]
        try:
            if os.path.exists(netedit_path):
                if net_file_path and os.path.exists(net_file_path):
                    subprocess.Popen([netedit_path, net_file_path])
                else:
                    subprocess.Popen([netedit_path])
                # 隐藏自定义窗口
                self.custom_window.withdraw()
            else:
                messagebox.showerror("错误", f"Netedit程序不存在: {netedit_path}")
        except Exception as e:
            messagebox.showerror("错误", f"启动Netedit时出错: {str(e)}")
            # 如果出错，恢复显示自定义窗口
            self.custom_window.deiconify()

    def open_manual_creation_guide(self):
        """打开手动创建路网文件的说明网页"""
        # 获取当前网络名称和路径
        network_name = self.network_name_var.get().strip()
        network_files_path = os.path.join(PROJECT_ROOT, "networkFiles", network_name)
        
        # 清除custom_window中的现有内容
        for widget in self.custom_window.winfo_children():
            widget.destroy()
        
        # 设置窗口标题
        self.custom_window.title("手动创建路网文件指南")
        
        # 添加标题
        title_frame = tk.Frame(self.custom_window, bg='#f0f0f0')
        title_frame.pack(pady=20, padx=20, fill='x')
        
        title_label = tk.Label(
            title_frame, 
            text="自主式交通系统语义交互仿真平台", 
            font=('Arial', 16, 'bold'),
            fg='#0066cc',
            bg='#f0f0f0'
        )
        title_label.pack()
        
        # 分隔线
        separator = ttk.Separator(self.custom_window, orient='horizontal')
        separator.pack(fill='x', padx=20)
        
        # 说明文本（使用与"请选择场景类型"相同的字体样式）
        info_label = tk.Label(
            self.custom_window,
            text="请详细阅读《SUMO路网手动生成步骤详解》，并按照\n1.编写节点文件\n2.编写边文件\n3.编写连接文件\n的顺序来进行路网文件的手动创建。\n\n请在点击三个步骤的相应按钮后，在新创建的路网文件夹中，使用任意文本编辑器打开三个xml文件，并对其进行自由编辑。",
            font=('Arial', 12, 'bold'),
            wraplength=480,
            justify='left',
            bg='#f0f0f0'
        )
        info_label.pack(pady=20, padx=10)
        
        # 按钮框架
        buttons_frame = tk.Frame(self.custom_window)
        buttons_frame.pack(pady=20, fill='both', expand=True)
        
        # 创建按钮样式
        button_style = {
            'font': ('Arial', 10),
            'width': 20,
            'height': 2,
            'bg': '#ffffff',
            'fg': '#333333',
            'relief': 'raised',
            'borderwidth': 1
        }
        
        # 左侧按钮功能函数
        def open_nodes_file():
            """打开节点文件"""
            nodes_file_path = os.path.join(network_files_path, "nodes.xml")
            # 如果文件不存在，创建一个基本的模板
            if not os.path.exists(nodes_file_path):
                with open(nodes_file_path, 'w', encoding='utf-8') as f:
                    f.write('<?xml version="1.0" encoding="UTF-8"?>\n<nodes>\n\t<!-- 在此处添加节点定义 -->\n</nodes>')
            # 使用系统默认编辑器打开文件
            os.startfile(nodes_file_path)
        
        def open_edges_file():
            """打开边文件"""
            edges_file_path = os.path.join(network_files_path, "edges.xml")
            # 如果文件不存在，创建一个基本的模板
            if not os.path.exists(edges_file_path):
                with open(edges_file_path, 'w', encoding='utf-8') as f:
                    f.write('<?xml version="1.0" encoding="UTF-8"?>\n<edges>\n\t<!-- 在此处添加边定义 -->\n</edges>')
            # 使用系统默认编辑器打开文件
            os.startfile(edges_file_path)
        
        def open_connections_file():
            """打开连接文件"""
            connections_file_path = os.path.join(network_files_path, "connections.xml")
            # 如果文件不存在，创建一个基本的模板
            if not os.path.exists(connections_file_path):
                with open(connections_file_path, 'w', encoding='utf-8') as f:
                    f.write('<?xml version="1.0" encoding="UTF-8"?>\n<connections>\n\t<!-- 在此处添加连接定义 -->\n</connections>')
            # 使用系统默认编辑器打开文件
            os.startfile(connections_file_path)
        
        # 一键合成路网文件功能函数
        def generate_network_file():
            """一键合成路网文件"""
            try:
                # 检查必要的文件是否存在
                nodes_file = os.path.join(network_files_path, "nodes.xml")
                edges_file = os.path.join(network_files_path, "edges.xml")
                connections_file = os.path.join(network_files_path, "connections.xml")
                
                # 检查文件是否存在
                missing_files = []
                if not os.path.exists(nodes_file):
                    missing_files.append("nodes.xml")
                if not os.path.exists(edges_file):
                    missing_files.append("edges.xml")
                if not os.path.exists(connections_file):
                    missing_files.append("connections.xml")
                
                if missing_files:
                    messagebox.showerror("错误", f"缺少以下文件: {', '.join(missing_files)}\n请先创建这些文件。")
                    return
                
                # 构建netconvert命令
                netconvert_path = loc_config["LOC_SUMO_NETCONVERT"]
                output_file = os.path.join(network_files_path, f"{network_name}.net.xml")
                
                cmd = [
                    netconvert_path,
                    "--node-files", nodes_file,
                    "--edge-files", edges_file,
                    "--connection-files", connections_file,
                    "--output-file", output_file
                ]
                
                # 执行命令
                result = subprocess.run(cmd, capture_output=True, text=True)
                
                if result.returncode == 0:
                    messagebox.showinfo("成功", f"路网文件已成功生成: {output_file}")
                else:
                    messagebox.showerror("错误", f"生成路网文件失败:\n{result.stderr}")
            except Exception as e:
                messagebox.showerror("错误", f"执行一键合成时出错: {str(e)}")
        
        # 创建一个新的框架用于放置按钮
        button_grid_frame = tk.Frame(buttons_frame)
        button_grid_frame.pack(expand=True)
        
        # 使用网格布局放置按钮和符号
        # 第一列：文件编辑按钮和加号符号
        node_btn = tk.Button(
            button_grid_frame,
            text="编写节点文件",
            command=open_nodes_file,
            **button_style
        )
        node_btn.grid(row=0, column=0, padx=10, pady=5, sticky='w')
        
        # 第一个"+"号，位于编写节点文件和编写边文件之间
        plus_label1 = tk.Label(button_grid_frame, text="+", font=('Arial', 16, 'bold'), bg='#f0f0f0')
        plus_label1.grid(row=1, column=0, padx=10, pady=5)
        
        edge_btn = tk.Button(
            button_grid_frame,
            text="编写边文件",
            command=open_edges_file,
            **button_style
        )
        edge_btn.grid(row=2, column=0, padx=10, pady=5, sticky='w')
        
        # 第二个"+"号，位于编写边文件和编写连接文件之间
        plus_label2 = tk.Label(button_grid_frame, text="+", font=('Arial', 16, 'bold'), bg='#f0f0f0')
        plus_label2.grid(row=3, column=0, padx=10, pady=5)
        
        connection_btn = tk.Button(
            button_grid_frame,
            text="编写连接文件",
            command=open_connections_file,
            **button_style
        )
        connection_btn.grid(row=4, column=0, padx=10, pady=5, sticky='w')
        
        # 箭头符号，距离按钮有一定距离
        arrow_label = tk.Label(button_grid_frame, text="→", font=('Arial', 16), bg='#f0f0f0')
        arrow_label.grid(row=2, column=1, padx=20, pady=5)
        
        # 一键合成按钮
        generate_btn = tk.Button(
            button_grid_frame,
            text="一键合成路网文件",
            command=generate_network_file,
            **button_style
        )
        generate_btn.grid(row=2, column=2, padx=20, pady=5, sticky='e')
        
        # 打开网页说明文件
        html_path = loc_config["LOC_MANUAL_NETWORK_CREATION"]
        if os.path.exists(html_path):
            webbrowser.open(f"file://{html_path}")
        else:
            messagebox.showerror("错误", f"说明文件不存在: {html_path}")

    def confirm_network_name(self):
        """确认路网名称"""
        # 获取输入的路网名称
        network_name = self.network_name_var.get().strip()
        
        # 检查名称是否为空
        if not network_name:
            messagebox.showerror("错误", "路网名称不能为空！")
            return
        
        # 构建路网文件路径
        network_files_path = os.path.join(PROJECT_ROOT, "networkFiles", network_name)
        
        # 检查是否已存在同名文件夹
        if os.path.exists(network_files_path):
            # 创建提示窗口
            duplicate_window = tk.Toplevel(self.custom_window)
            duplicate_window.title("提示")
            duplicate_window.geometry("300x150")
            duplicate_window.resizable(False, False)
            
            # 居中显示
            duplicate_window.transient(self.custom_window)
            duplicate_window.grab_set()
            
            # 提示文本
            label = tk.Label(duplicate_window, text="已有相同名字的路网，请重新命名", 
                            font=('Arial', 10), wraplength=280, justify='center')
            label.pack(pady=30)
            
            # 确认按钮
            def on_confirm():
                duplicate_window.destroy()
            
            confirm_btn = tk.Button(duplicate_window, text="确定", command=on_confirm, 
                                   font=('Arial', 10), width=10)
            confirm_btn.pack(pady=10)
            return
        
        try:
            # 创建场景文件夹
            os.makedirs(network_files_path, exist_ok=True)
            
            # 清除按钮区域的内容
            for widget in self.button_frame.winfo_children():
                widget.destroy()
            
            # 请选择创建方式标题
            method_label = tk.Label(
                self.button_frame,
                text="请选择创建方式：",
                font=('Arial', 10, 'bold'),
                bg='#f0f0f0'
            )
            method_label.pack(pady=(20, 10))
            
            # 创建方式按钮框架（横向排布）
            button_frame = tk.Frame(self.button_frame, bg='#f0f0f0')
            button_frame.pack(pady=5)
            
            # 创建按钮样式，统一所有按钮的尺寸
            button_style = {
                'font': ('Arial', 10),
                'width': 15,  # 统一宽度
                'height': 2,
                'bg': '#ffffff',
                'fg': '#333333',
                'relief': 'raised',
                'borderwidth': 1
            }
            
            # 使用Netedit创建按钮
            netedit_btn = tk.Button(
                button_frame,
                text="使用Netedit创建",
                command=self.launch_netedit,  # 不传递文件路径
                **button_style
            )
            netedit_btn.pack(side=tk.LEFT, padx=10)
            
            # 手动创建按钮
            manual_btn = tk.Button(
                button_frame,
                text="手动创建",
                command=self.open_manual_creation_guide,
                **button_style
            )
            manual_btn.pack(side=tk.LEFT, padx=10)
        except Exception as e:
            messagebox.showerror("错误", f"创建路网文件时出错: {str(e)}")

def main():
    root = tk.Tk()
    app = ScenarioSelector(root)
    root.mainloop()

if __name__ == "__main__":
    main()