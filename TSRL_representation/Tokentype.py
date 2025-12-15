from enum import Enum
from typing import List, Optional

#语言包括的各类词法单元
class TokenType(Enum):  #实际上相当于字典
    # 单字符标记

    LEFT_PAREN = 'LEFT_PAREN'
    RIGHT_PAREN = 'RIGHT_PAREN'
    LEFT_BRACE = 'LEFT_BRACE'
    RIGHT_BRACE = 'RIGHT_BRACE'
    COMMA = 'COMMA'
    DOT = 'DOT'
    MINUS = 'MINUS'
    PLUS = 'PLUS'
    SEMICOLON = 'SEMICOLON'
    SLASH = 'SLASH'
    STAR = 'STAR'

    # 一个或两个字符的标记
    BANG = 'BANG'
    BANG_EQUAL = 'BANG_EQUAL'
    EQUAL = 'EQUAL'
    EQUAL_EQUAL = 'EQUAL_EQUAL'
    GREATER = 'GREATER'
    GREATER_EQUAL = 'GREATER_EQUAL'
    LESS = 'LESS'
    LESS_EQUAL = 'LESS_EQUAL'
    IMPLICATE = "IMPLICATE" #蕴含:-
    ASK = "ASK"#提问 ?-

    # 字面量
    IDENTIFIER = 'IDENTIFIER'
    STRING = 'STRING'
    NUMBER = 'NUMBER'

    # 关键字
    PRINT = 'PRINT'
    AND = 'AND'
    OR = 'OR'

    # CLASS = 'CLASS'
    FALSE = 'FALSE'
    FOR = 'FOR'
    NIL = 'NIL'
    TRUE = 'TRUE'


    # 文件结束标记
    EOF = 'EOF'

    #声明
    PREDICATE = 'PREDICATE'#谓词
    VAR = 'VAR' #变量
    FUN = 'FUN' #函数
    RULE = "RULE" #规则



#词法单元
class Token:
    def __init__(self, type: TokenType, lexeme: str, literal: Optional[str], line: int):
        self.type = type
        self.lexeme = lexeme
        self.literal = literal
        self.line = line

    def __repr__(self):
        return f'Token({self.type.value}, {self.lexeme}, {self.literal}, {self.line})'