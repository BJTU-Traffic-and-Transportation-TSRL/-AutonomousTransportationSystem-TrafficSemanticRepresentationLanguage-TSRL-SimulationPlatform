import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import sys
import os
import re
import webbrowser

# 获取项目根目录
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
# 导入地址配置文件
from utils.load_config import load_config
loc_config = load_config(os.path.join(PROJECT_ROOT, "loc_config.yaml"))

class TrafficSemanticIntersectionSelector:
    def __init__(self, root):
        self.root = root
        self.root.title("交通语义交互场景")
        self.root.geometry("600x500")
        self.root.resizable(False, False)
        # 设置样式
        self.setup_styles()
        # 是否使用SUMO GUI的标志
        # self.use_sumo_gui = tk.BooleanVar(value=False)
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
            text="自主式交通系统语义交互仿真平台", 
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
            text="请选择交通方式:",
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
        
        # 道路交通场景按钮
        road_traffic_btn = tk.Button(
            button_frame,
            text="道路交通场景",
            command=self.run_tkinter_scenario_selector,
            **button_style
        )
        road_traffic_btn.pack(pady=10)
        # 轨道交通场景按钮
        railway_traffic_btn = tk.Button(
            button_frame,
            text="轨道交通场景",
            command=self.run_railway_scenario,
            **button_style
        )
        railway_traffic_btn.pack(pady=10)
        # 水运交通场景按钮
        list_btn = tk.Button(
            button_frame,
            text="水运交通场景",
            command=self.run_ship_scenario,
            **button_style
        )
        list_btn.pack(pady=10)
        # 说明区域
        help_frame = tk.Frame(self.root, bg='#f0f0f0')
        help_frame.pack(pady=(5, 20), padx=20, fill='x')
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
            text="请根据需要，选择某一种交通方式进行语义交互展示",
            font=('Arial', 8),
            fg='#888888',
            bg='#f0f0f0',
            justify='left'
        )
        help_text.pack(anchor='w', pady=(5, 0))
        
    def run_tkinter_scenario_selector(self):
        """运行tkinter场景选择器程序"""
        try:
            # 将相对路径转换为绝对路径
            tkinter_path = os.path.join(PROJECT_ROOT, loc_config["LOC_TKINTER"])
            # 使用subprocess.Popen启动tkinter_scenario_selector.py
            subprocess.Popen([sys.executable, tkinter_path], 
                           creationflags=subprocess.CREATE_NEW_CONSOLE)
            messagebox.showinfo("提示", "已启动交通场景选择器程序\n请查看新打开的窗口。")
        except Exception as e:
            messagebox.showerror("错误", f"运行场景选择器时出错: {str(e)}")

    def run_railway_scenario(self):
        """运行轨道交通场景"""
        # 清除所有现有组件
        for widget in self.root.winfo_children():
            widget.destroy()
        # 标题
        title_frame = tk.Frame(self.root, bg='#f0f0f0')
        title_frame.pack(pady=20, padx=20, fill='x')
        title_label = tk.Label(
            title_frame, 
            text="轨道交通语义交互场景", 
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
        # 轨道交通场景1按钮：列车与无线闭塞中心交互场景
        railway_btn_1 = tk.Button(
            button_frame,
            text="列车与无线闭塞中心交互场景",
            command=self.run_railway_model1,
            **button_style
        )
        railway_btn_1.pack(pady=10)
        # 轨道交通场景2按钮：会让站货运列车避让客运列车场景
        railway_btn_2 = tk.Button(
            button_frame,
            text="会让站货运列车避让客运列车场景",
            command=self.run_railway_path_conflict1,
            **button_style
        )
        railway_btn_2.pack(pady=10)
        # 说明区域
        help_frame = tk.Frame(self.root, bg='#f0f0f0')
        help_frame.pack(pady=(5, 20), padx=20, fill='x')
        help_title = tk.Label(
            help_frame,
            text="说明:",
            font=('Arial', 9, 'bold'),
            fg='#666666',
            bg='#f0f0f0'
        )
        help_title.pack(anchor='center')
        help_text = tk.Label(
            help_frame,
            text="请根据需要，选择两种轨道交通场景中的一种进行语义交互展示：\n"
            "1. 列车与无线闭塞中心交互场景：\n"
            "   无线闭塞中心根据区段不同情况与列车进行语义信息交互，列车根据语义交互信息执行相应的指令。\n"
            "2. 会让站货运列车避让客运列车场景：\n"
            "   使用语义交互信息调度客货运列车，确保具有高优先级的客运列车能够优先通行。",
            font=('Arial', 10),
            fg='#888888',
            bg='#f0f0f0',
            justify='left'
        )
        help_text.pack(anchor='center', pady=(5, 0))
    
    def run_railway_model1(self):
        """运行轨道交通场景1：列车与无线闭塞中心交互场景"""
        try:
            model1_path = os.path.join(PROJECT_ROOT, loc_config["LOC_RAILWAY_MODEL1"])
            if os.path.exists(model1_path):
                # 使用系统默认程序打开AnyLogic模型文件
                os.startfile(model1_path)
                messagebox.showinfo("提示", f"已打开轨道交通场景1模型文件:\n{model1_path}")
            else:
                messagebox.showerror("错误", f"模型文件不存在:\n{model1_path}")
        except Exception as e:
            messagebox.showerror("错误", f"打开轨道交通场景1模型时出错: {str(e)}")
    
    def run_railway_path_conflict1(self):
        """运行轨道交通场景2：会让站货运列车避让客运列车场景"""
        try:
            path_conflict1_path = os.path.join(PROJECT_ROOT, loc_config["LOC_RAILWAY_PATH_CONFLICT1"])
            if os.path.exists(path_conflict1_path):
                # 使用系统默认程序打开AnyLogic模型文件
                os.startfile(path_conflict1_path)
                messagebox.showinfo("提示", f"已打开轨道交通场景2模型文件:\n{path_conflict1_path}")
            else:
                messagebox.showerror("错误", f"模型文件不存在:\n{path_conflict1_path}")
        except Exception as e:
            messagebox.showerror("错误", f"打开轨道交通场景2模型时出错: {str(e)}")
    
    # def run_ship_scenario(self):
    #     """运行水运交通场景"""
    #     # 清除所有现有组件
    #     for widget in self.root.winfo_children():
    #         widget.destroy()
    #     # 标题
    #     title_frame = tk.Frame(self.root, bg='#f0f0f0')
    #     title_frame.pack(pady=20, padx=20, fill='x')
    #     title_label = tk.Label(
    #         title_frame, 
    #         text="水运交通语义交互场景选择器", 
    #         font=('Arial', 16, 'bold'),
    #         fg='#0066cc',
    #         bg='#f0f0f0'
    #     )
    #     title_label.pack()
    #     # 返回按钮
    #     back_btn = tk.Button(
    #         title_frame,
    #         text="返回",
    #         command=self.create_initial_widgets,
    #         font=('Arial', 8),
    #         width=8,
    #         height=1
    #     )
    #     back_btn.pack(side='right')
    #     # 分隔线
    #     separator = ttk.Separator(self.root, orient='horizontal')
    #     separator.pack(fill='x', padx=20)
    #     # 按钮区域
    #     button_frame = tk.Frame(self.root, bg='#f0f0f0')
    #     button_frame.pack(pady=30, padx=20, fill='both', expand=True)
    #     # 创建按钮样式
    #     button_style = {
    #         'font': ('Arial', 12),
    #         'width': 30,
    #         'height': 2,
    #         'bg': '#ffffff',
    #         'fg': '#333333',
    #         'relief': 'raised',
    #         'borderwidth': 1
    #     }
    #     # 水运交通场景1按钮
    #     ship_btn_1 = tk.Button(
    #         button_frame,
    #         text="水运交通场景1",
    #         command=self.run_ship_model1,
    #         **button_style
    #     )
    #     ship_btn_1.pack(pady=10)
    #     # 水运交通场景2按钮：会让站货运列车避让客运列车场景
    #     ship_btn_2 = tk.Button(
    #         button_frame,
    #         text="水运交通场景2",
    #         command=self.run_ship_path_conflict1,
    #         **button_style
    #     )
    #     ship_btn_2.pack(pady=10)
    #     # 说明区域
    #     help_frame = tk.Frame(self.root, bg='#f0f0f0')
    #     help_frame.pack(pady=(5, 20), padx=20, fill='x')
    #     help_title = tk.Label(
    #         help_frame,
    #         text="说明:",
    #         font=('Arial', 9, 'bold'),
    #         fg='#666666',
    #         bg='#f0f0f0'
    #     )
    #     help_title.pack(anchor='center')
    #     help_text = tk.Label(
    #         help_frame,
    #         text="请根据需要，选择两种水运交通场景中的一种进行语义交互展示：\n",
    #         font=('Arial', 10),
    #         fg='#888888',
    #         bg='#f0f0f0',
    #         justify='left'
    #     )
    #     help_text.pack(anchor='center', pady=(5, 0))
    
    def run_ship_scenario(self):
        # """运行水运交通场景"""
        # try:
        #     ship_model_path = os.path.join(PROJECT_ROOT, "Ship Transportation System","ship Model","Model4.alp")
        #     if os.path.exists(ship_model_path):
        #         # 使用系统默认程序打开AnyLogic模型文件
        #         os.startfile(ship_model_path)
        #         messagebox.showinfo("提示", f"已打开水运交通场景模型文件:\n{ship_model_path}")
        #     else:
        #         messagebox.showerror("错误", f"模型文件不存在:\n{ship_model_path}")
        # except Exception as e:
        #     messagebox.showerror("错误", f"打开水运交通场景模型时出错: {str(e)}")
        """运行水运交通场景"""
        # 清除所有现有组件
        for widget in self.root.winfo_children():
            widget.destroy()
        # 标题
        title_frame = tk.Frame(self.root, bg='#f0f0f0')
        title_frame.pack(pady=20, padx=20, fill='x')
        title_label = tk.Label(
            title_frame, 
            text="水运交通语义交互场景", 
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
        # 按钮区域 - 使用expand=True确保框架可以扩展，fill='both'让框架完全填充可用空间
        button_frame = tk.Frame(self.root, bg='#f0f0f0')
        button_frame.pack(pady=0, padx=20, fill='both', expand=True)
        
        # 创建一个内部框架，用于实现按钮的垂直居中
        inner_frame = tk.Frame(button_frame, bg='#f0f0f0')
        inner_frame.pack(expand=True, fill='none')  # 内部框架扩展但不填充，确保可以居中
        
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
        # 水运交通场景：船舶会遇场景 - 在内部框架中精确居中
        ship_btn = tk.Button(
            inner_frame,
            text="船舶会遇场景",
            command=self.run_ship_model,
            **button_style
        )
        ship_btn.pack(pady=0)  # 内部框架已经处理了垂直居中，这里不需要额外的pady
        # 说明区域
        help_frame = tk.Frame(self.root, bg='#f0f0f0')
        help_frame.pack(pady=(5, 20), padx=20, fill='x')
        # help_title = tk.Label(
        #     help_frame,
        #     text="说明:",
        #     font=('Arial', 9, 'bold'),
        #     fg='#666666',
        #     bg='#f0f0f0'
        # )
        help_title.pack(anchor='center')
        help_text = tk.Label(
            help_frame,
            # text="请根据需要，选择两种轨道交通场景中的一种进行语义交互展示：\n"
            # "1. 列车与无线闭塞中心交互场景：\n"
            # "   无线闭塞中心根据区段不同情况与列车进行语义信息交互，列车根据语义交互信息执行相应的指令。\n"
            # "2. 会让站货运列车避让客运列车场景：\n"
            # "   使用语义交互信息调度客货运列车，确保具有高优先级的客运列车能够优先通行。",
            font=('Arial', 10),
            fg='#888888',
            bg='#f0f0f0',
            justify='left'
        )
        help_text.pack(anchor='center', pady=(5, 0))

    def run_ship_model(self):
        """运行水运交通场景"""
        try:
            ship_model_path = os.path.join(PROJECT_ROOT, loc_config["LOC_SHIP_MODEL4"])
            if os.path.exists(ship_model_path):
                # 使用系统默认程序打开AnyLogic模型文件
                os.startfile(ship_model_path)
                messagebox.showinfo("提示", f"已打开水运交通场景模型文件:\n{ship_model_path}")
            else:
                messagebox.showerror("错误", f"模型文件不存在:\n{ship_model_path}")
        except Exception as e:
            messagebox.showerror("错误", f"打开水运交通场景模型时出错: {str(e)}")

def main():
    root = tk.Tk()
    app = TrafficSemanticIntersectionSelector(root)
    root.mainloop()

if __name__ == "__main__":
    main()