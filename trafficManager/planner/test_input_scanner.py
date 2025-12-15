#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试Scanner词法分析器
功能：
1. 手动定义本地test.signal文件，内容为"LetAccelerate(0);"
2. 使用TSRL_representation.Scanner.py对该文件内容进行词法分析
3. 打印词法分析的结果
"""

import os
import sys

# 添加项目根目录到Python路径，确保能导入TSRL_representation模块
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

# 导入Scanner类和相关依赖
from TSRL_representation.Scanner import Scanner
from TSRL_representation.Tokentype import Token, TokenType

def main():
    # 1. 手动定义本地test.signal文件，内容为"LetAccelerate(0);"
    test_file_path = "test.signal"
    test_content = "LetAccelerate(0);"
    
    # 创建并写入文件
    with open(test_file_path, "w", encoding="utf-8") as f:
        f.write(test_content)
    
    print(f"已创建测试文件: {test_file_path}")
    print(f"文件内容: {test_content}")
    print("-" * 50)
    
    # 2. 使用Scanner对文件内容进行词法分析
    try:
        # 读取文件内容
        with open(test_file_path, "r", encoding="utf-8") as f:
            file_content = f.read()
        
        # 创建Scanner实例
        scanner = Scanner(file_content)
        
        # 进行词法分析
        tokens = scanner.scan_tokens()
        
        # 3. 打印词法分析的结果
        print("词法分析结果:")
        print(f"共识别到 {len(tokens)} 个Token:")
        print("-" * 50)
        
        # 打印每个Token的详细信息
        for i, token in enumerate(tokens):
            print(f"Token {i+1}:")
            print(f"  类型: {token.type}")
            print(f"  词素: '{token.lexeme}'")
            print(f"  字面量: {token.literal}")
            print(f"  行号: {token.line}")
            print("-" * 20)
        
    except Exception as e:
        print(f"词法分析过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理临时文件
        if os.path.exists(test_file_path):
            os.remove(test_file_path)
            print(f"已清理临时文件: {test_file_path}")

if __name__ == "__main__":
    main()
