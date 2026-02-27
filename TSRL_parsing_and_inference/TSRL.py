import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 使用绝对导入替代相对导入
from Parser import Parser
from Scanner import Scanner
from errorHanding import *
from Interpreter import Interpreter
import Inference_engine
import Expr

class TSRL:
    # 在类级别定义TSRL_interpreter，确保在任何地方都可以访问
    TSRL_interpreter = Interpreter()

    @staticmethod
    def main(input_file, output_file=None):
        """
        主函数，接收输入文件路径和可选的输出文件路径
        :param input_file: 输入文件路径
        :param output_file: 输出文件路径（可选）
        """
        if output_file:
            # 设置输出文件路径
            TSRL.TSRL_interpreter.set_output_file(output_file)
        TSRL.__run_file(input_file)

    @staticmethod
    def __run_file(file_path):
        try:
            with open(file_path, 'rb') as file:
                bytes_data = file.read()
            TSRL.__run(bytes_data.decode('utf-8'))  
            if hadError: sys.exit(65)
        except IOError as e:
            print(f"An error occurred while reading the file: {e}")

    @staticmethod
    def __run_prompt():
        global hadError
        try:
            while True:
                user_input = input("> ")
                if user_input.strip() == "":
                    break
                TSRL.__run(user_input)
                hadError = False
        except EOFError:
            pass  # 捕获EOFError以处理用户中断输入的情况（例如Ctrl+D）

    @staticmethod
    def __run(source):
        scanner = Scanner(source)
        tokens = scanner.scan_tokens()
        parser = Parser(tokens)
        statements = parser.parse() 
        
        # 使用类级别的TSRL_interpreter
        # 如果没有设置输出文件，则使用默认路径
        if not TSRL.TSRL_interpreter.output_file:
            output_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Infer_output", "output.txt")
            TSRL.TSRL_interpreter.set_output_file(output_file_path)
        TSRL.TSRL_interpreter.interpret(statements)

"""
修改TSRL.main()，使其接收输入和输出文件路径作为参数
"""
# input_file = "TSRL_representation\Infer_input\input_1.txt"
# output_file = "TSRL_representation\Infer_output\output.txt"
# TSRL.main(input_file, output_file)