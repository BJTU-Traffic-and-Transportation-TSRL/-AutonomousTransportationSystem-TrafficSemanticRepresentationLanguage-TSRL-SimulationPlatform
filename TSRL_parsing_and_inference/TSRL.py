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
            TSRL.TSRL_interpreter.set_output_file(output_file) # 设置输出文件路径
        TSRL.__run_file(input_file) # 读取输入文件内容

    @staticmethod
    def __run_file(file_path):
        try:
            with open(file_path, 'rb') as file:
                bytes_data = file.read()
            TSRL.__run(bytes_data.decode('utf-8')) # 执行处理
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
        scanner = Scanner(source) # 实例化词法解析器Scanner类
        tokens = scanner.scan_tokens() # 调用词法解析器的scan_tokens()方法，返回tokens列表
        # 2026.2.27 分行打印tokens列表
        print("Tokens:\n")
        for token in tokens:
            print(token)
        parser = Parser(tokens) # 实例化语法解析器Parser类
        statements = parser.parse() # 调用语法解析器的parse()方法，返回tokens形成的语句列表对应的语句类型(statements)列表
        # 2026.3.19 分行打印AST列表
        print("\nAST:\n")
        for statement in statements:
            print(f"statement_class:{statement.__class__}\nstatement_expression:{statement.expression}\nstatement_op:{statement.expression.op}\nstatement_token:{statement.expression.token}\n")
        # 使用类级别的TSRL_interpreter
        # 如果没有设置输出文件，则使用默认路径
        if not TSRL.TSRL_interpreter.output_file:
            output_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Infer_output", "output.txt")
            TSRL.TSRL_interpreter.set_output_file(output_file_path)
        TSRL.TSRL_interpreter.interpret(statements) # 调用解释器的interpret()方法，对statements列表中的语句进行解释执行

"""
修改TSRL.main()，使其接收输入和输出文件路径作为参数
"""
if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(base_dir, "Infer_input", "input_FCW.txt")
    output_file = os.path.join(base_dir, "Infer_output", "output.txt")
    TSRL.main(input_file, output_file)