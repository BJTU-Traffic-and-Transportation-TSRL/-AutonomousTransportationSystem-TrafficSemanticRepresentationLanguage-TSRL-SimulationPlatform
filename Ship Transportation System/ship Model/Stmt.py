"""
2025/1/5
语法树节点与语句类型
ver:1.0
*********************************************************
"""
#Expr AST nodes.
from typing import Any, List
from Tokentype import Token
from abc import ABC, abstractmethod
from Expr import Expr
from Expr import Variable

class StmtVisitor(ABC):

    def visitExpressionStmt(self, stmt): #stmt:Expression
        pass

    def visitAskStmt(self, stmt): #stmt:Print
        pass

    def visitPrintStmt(self, stmt): #stmt:Print
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
