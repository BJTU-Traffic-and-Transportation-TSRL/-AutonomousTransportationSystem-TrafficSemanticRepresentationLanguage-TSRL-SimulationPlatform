'''
2024/12/20
词法分析器
ver:1.0
'''
import sys
import os
# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import Tokentype
from Tokentype import *
from typing import List, Optional
import errorHanding

class Scanner:
    def __init__(self, source: str):
        self.source = source
        self.tokens: List[Token] = []
        #start字段指向被扫描的词素中的第一个字符，current字段指向当前正在处理的字符。line字段跟踪的是current所在的源文件行数
        self.start = 0
        self.current = 0
        self.line = 1

    def scan_tokens(self) -> List[Token]:
        while not self.is_at_end():
            # We are at the beginning of the next lexeme.
            self.start = self.current
            self.scan_token()
        
        self.tokens.append(Token(TokenType.EOF, '', None, self.line))
        return self.tokens
    
    def is_at_end(self) -> bool:
        return self.current >= len(self.source)

    def scan_token(self):
        c = self.advance()
        if c == '(':
            self.add_token(TokenType.LEFT_PAREN)
        elif c == ')':
            self.add_token(TokenType.RIGHT_PAREN)
        elif c == '{':
            self.add_token(TokenType.LEFT_BRACE)
        elif c == '}':
            self.add_token(TokenType.RIGHT_BRACE)
        elif c == ',':
            self.add_token(TokenType.COMMA)
        elif c == '.':
            self.add_token(TokenType.DOT)
        elif c == '-':
            self.add_token(TokenType.MINUS)
        elif c == '+':
            self.add_token(TokenType.PLUS)
        elif c == ';':
            self.add_token(TokenType.SEMICOLON)
        elif c == '*':
            self.add_token(TokenType.STAR)
        elif c == '∨':
            self.add_token(TokenType.OR)
        elif c == '∧':
            self.add_token(TokenType.AND)
        #注释空格和换行
        elif c == '/':
            if self.match('/'):
                # A comment goes until the end of the line.
                while self.peek() != '\n' and not self.is_at_end():
                    self.advance()
            else:
                self.add_token(TokenType.SLASH)
        elif c in (' ', '\r', '\t'):
            # Ignore whitespace.
            pass
        elif c == '\n':
            self.line += 1        

        #字符串
        elif c == '"':
            self.string()

        #操作符    
        elif c == '!':
            if self.match('='):
                self.add_token(TokenType.BANG_EQUAL)
            else:
                self.add_token(TokenType.BANG)
        elif c == '=':
            if self.match('='):
                self.add_token(TokenType.EQUAL_EQUAL)
            else:
                self.add_token(TokenType.EQUAL)
        elif c == '<':
            if self.match('='):
                self.add_token(TokenType.LESS_EQUAL)
            else:
                self.add_token(TokenType.LESS)
        elif c == '>':
            if self.match('='):
                self.add_token(TokenType.GREATER_EQUAL)
            else:
                self.add_token(TokenType.GREATER)
        elif c == ':':
            if self.match('-'):
                self.add_token(TokenType.IMPLICATE)
            else:
                # 处理意外字符
                errorHanding.scanError(self.line, "Unexpected character.")
        elif c == '?':
            if self.match('-'):
                self.add_token(TokenType.ASK)
            else:
                # 处理意外字符
                errorHanding.scanError(self.line, "Unexpected character.")
        #数字
        elif self.is_digit(c):
            self.number()

        elif self.is_alpha(c):
            self.identifier()
        # 可以继续添加其他字符的处理逻辑

        else:
        # 处理意外字符
            errorHanding.scanError(self.line, "Unexpected character.")

    #处理字符串
    def string(self):
        while self.peek() != '"' and not self.is_at_end():
            if self.peek()== '\n':
                self.line+=1
            self.advance()
        if self.is_at_end():
            errorHanding.scanError(self.line, "Unterminated string.")
            return
        #关闭
        self.advance()
        value = self.source[self.start+1:self.current-1]
        self.add_token(TokenType.STRING, value)

    #处理数字
    def is_digit(self, c: str) -> bool:
        return c.isdigit()

    def number(self):
        # 这里实现处理数字的逻辑
        while self.is_digit(self.peek()):
            self.advance()
        #处理小数点
        if self.peek()== '.' and self.is_digit(self.peek_next()):
            self.advance()
            while self.is_digit(self.peek()):
                self.advance()
        number_str = self.source[self.start:self.current]
        # 根据需要将字符串转换为整数或浮点数
        try:
            value = int(number_str)
        except ValueError:
            value = float(number_str)
        self.add_token(TokenType.NUMBER, value)

    #处理保留字和标识符
    def identifier(self):
        while self.is_alphanumeric(self.peek()):
            self.advance()
        text = self.source[self.start:self.current]
        token_type = keywords.get(text, TokenType.IDENTIFIER)
        self.add_token(token_type)

    def is_alpha(self, c: str) -> bool:
        return (c >= 'a' and c <= 'z') or \
               (c >= 'A' and c <= 'Z') or \
               c == '_'

    def is_alphanumeric(self, c: str) -> bool:
        return self.is_alpha(c) or self.is_digit(c) or c == '.'

    def is_alpha(self, c: str) -> bool:
        return (c >= 'a' and c <= 'z') or \
               (c >= 'A' and c <= 'Z') or \
               c == '_' or c == '.'

    #词分析函数
        # 前瞻一个字符
    def peek(self) -> str:
        if self.is_at_end():
            return '\0'
        return self.source[self.current]

        # 前瞻两个字符
    def peek_next(self):
        if self.current + 1 >= len(self.source):
            return '\0'
        return self.source[self.current + 1]

        # 消费一个字符
    def advance(self) -> str:
        self.current += 1
        return self.source[self.current - 1]

        # 判断二元词法单元，并消费
    def match(self, expected: str) -> bool:
        if self.is_at_end() or self.peek() != expected:
            return False
        self.current += 1
        return True


    def add_token(self, token_type: TokenType, literal=None):
        lexeme = self.source[self.start:self.current]
        token = Token(token_type, lexeme, literal, self.line)
        self.tokens.append(token)


# 创建关键字映射
keywords = {
    "and":    TokenType.AND,
    # "class":  TokenType.CLASS,

    "false":  TokenType.FALSE,
    "for":    TokenType.FOR,
    # "fun":    TokenType.FUN,
    # "Rule":     TokenType.RULE,
    "nil":    TokenType.NIL,
    "or":     TokenType.OR,
    "true":   TokenType.TRUE,
    # "var":    TokenType.VAR,
    "ASK" : TokenType.ASK,
    'PRINT' :TokenType.PRINT,
    'Let':TokenType.LET,
    'Tell':TokenType.TELL,
}