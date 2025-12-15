import sys
import os

from Parser import Parser
from Scanner import Scanner
from errorHanding import *
from Interpreter import Interpreter
import Inference_engine
import Expr

class TSIL:

    @staticmethod
    def main():
        if len(sys.argv) > 2:
            print("Usage: python lox.py [script]")
            sys.exit(64)
        elif len(sys.argv) == 2:
            TSIL.__run_file(sys.argv[1])
        else:
            # print(sys.argv)
            TSIL.__run_prompt()

    @staticmethod
    def __run_file(file_path):
        try:
            with open(file_path, 'rb') as file:
                bytes_data = file.read()
            TSIL.__run(bytes_data.decode('utf-8'))
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
                TSIL.__run(user_input)
                hadError = False
        except EOFError:
            pass  # 捕获EOFError以处理用户中断输入的情况（例如Ctrl+D）

    @staticmethod
    def __run(source):
        scanner = Scanner(source)
        tokens = scanner.scan_tokens()
        # 目前只是打印tokens
        # for token in tokens:
        #     print(token)
        parser = Parser(tokens)
        statements = parser.parse()
        TSIL_interpreter.interpret(statements)



TSIL_interpreter = Interpreter()
if len(sys.argv) < 2:
    sys.argv.append("input.txt")
else:
    sys.argv[1] = 'input.txt'
TSIL.main()

