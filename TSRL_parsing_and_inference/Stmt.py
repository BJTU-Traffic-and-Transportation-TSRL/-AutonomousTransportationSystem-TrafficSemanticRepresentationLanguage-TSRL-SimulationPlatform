"""
2025/1/5
语法树节点与语句类型
ver:1.0
*********************************************************
"""
#Expr AST nodes.
import sys
import os
# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from typing import Any, List
from Tokentype import Token
from abc import ABC, abstractmethod
import Expr
from Expr import Variable

class StmtVisitor(ABC):

    def visitExpressionStmt(self, stmt): #stmt:Expression
        pass

    def visitAskStmt(self, stmt): #stmt:Print
        pass

    def visitPrintStmt(self, stmt): #stmt:Print
        pass
    def visitTellStmt(self, stmt): #stmt:Print
        pass



class Stmt:
    #Abstract base class for all AST nodes.
    def accept(self, visitor: StmtVisitor):
        pass


class Expression(Stmt):
    def __init__(self, expression: Expr):
        self.expression = expression

    def accept(self, visitor: StmtVisitor):
        return visitor.visitExpressionStmt(self)

class Ask(Stmt):
    def __init__(self, expression:Expr):
        self.expression = expression

    def accept(self, visitor: StmtVisitor):
        return visitor.visitAskStmt(self)


class Print(Stmt):
    def __init__(self, expression:Expr):
        self.expression = expression

    def accept(self, visitor: StmtVisitor):
        return visitor.visitPrintStmt(self)

class Tell(Stmt):
    def __init__(self, expression:Expr):
        self.expression = expression

    def accept(self, visitor: StmtVisitor):
        return visitor.visitTellStmt(self)
