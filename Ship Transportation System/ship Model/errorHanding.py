'''
2024/12/20
词法分析错误处理单元
ver:1.0
'''
import sys
from Tokentype import *

hadError = False  #错误标志
hadRuntimeError = False
def scanError(line, message):
    report(line, "", message)

def parseError(token: Token, message: str):
    if token.type == TokenType.EOF:
      report(token.line, " at end", message)
    else:
      report(token.line, " at '" + token.lexeme + "'", message)

def runtimeError(error) :
    global hadRuntimeError
    print('{}'.format(error) + "\n[line " + str(error.token.line) + "]")
    hadRuntimeError = True

def report(line, where, message):
    global hadError
    print(f"[line {line}] Error{where}: {message}\n", file=sys.stderr)
    hadError = True