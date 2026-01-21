"""
2024/12/20
语法树节点与表达式类型
ver:1.0
"""
#Expr AST nodes.
import sys
import os
# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from typing import Any, List
from Tokentype import Token, TokenType
from abc import ABC, abstractmethod


class ExprVisitor(ABC):

    # def visitAssignExpr(self,expr):
    #     pass

    def visitBinary(self, expr):
        pass

    def visitCallExpr(self,expr):
        pass


    def visitLiteral(self, expr):
        pass

    def visitLogicalExpr(self, expr):
        pass

    def visitUnary(self, expr):
        pass

    def visitVariableExpr(self, expr):
        pass

    def visitPredicateExpr(self, expr):
        pass

    def visitImplicationExpr(self, expr):
        pass

    def visitConstantExpr(self, self1):

        pass


class Expr:
    # base class for all AST nodes.

    def __init__(self, op:str,token=None, *args):
        self.op = str(op)
        self.args = args
        self.token = token

    def __eq__(self, other):
        """x == y' evaluates to True or False; does not build an Expr."""
        return isinstance(other, Expr) and self.op == other.op and self.args == other.args

    def __hash__(self):
        return hash(self.op) ^ hash(self.args)

    def __repr__(self):
        op = self.op
        args = [str(arg) for arg in self.args]
        if op.isidentifier():  # f(x) or f(x, y)
            return '{}({})'.format(op, ', '.join(args)) if args else op
        elif len(args) == 1:  # -x or -(x + 1)
            return op + args[0]
        # elif len(args) == 0 and str.isdigit(op[0]): # 1,2.9,90.90,主要是为处理单一数字的情况
        elif len(args) == 0 :
            return op
        else:  # (x - y)
            opp = (' ' + op + ' ')
            return '(' + opp.join(args) + ')'

    def accept(self, visitor: ExprVisitor):
        pass

class Implication(Expr):
    def __init__(self,token=None, *args): #在args中条件在前，结论在后
        super().__init__(':-', *args)
        self.args = args
        self.token = token

    def accept(self, visitor: ExprVisitor):
        return visitor.visitImplicationExpr(self)

class Predicate(Expr):
    def __init__(self,op:str,token=None, *args):
        super().__init__(op, token,*args)

    def accept(self, visitor: ExprVisitor):
        return visitor.visitPredicateExpr(self)


class Binary(Expr):
    #Represents a binary expression.
    """
    所有的+-*/表达式均为二元表达式，二元表达式的前项和后项必须是可计算的表达式。
    """
    def __init__(self, left: Expr, op:str, operator: Token, right: Expr, ):
        super().__init__(op, operator,left, right)
        self.left = left
        self.right = right
        self.operator = operator

    def accept(self, visitor: ExprVisitor):
        return visitor.visitBinary(self)

class Literal(Expr):
    #Represents a literal expression.
    def __init__(self, value: object, op: Token, *args):
      super().__init__(op.lexeme,op, *args)
      self.value = value

    def accept(self, visitor: ExprVisitor):
        return visitor.visitLiteral(self)

class Logical(Expr):
    def __init__(self, left:Expr, operator: Token, right:Expr):
        super().__init__(operator.lexeme, operator,left, right)
        self.left = left
        self.operator = operator
        self.right = right

    def accept(self, visitor: ExprVisitor):
        return visitor.visitLogicalExpr(self)

class Unary(Expr):
    #Represents a unary expression.
    def __init__(self, operator: Token, right: Expr ):
        super().__init__(operator.lexeme,operator, right)
        self.operator = operator
        self.right = right

    def accept(self, visitor: ExprVisitor):
        return visitor.visitUnary(self)

class Variable(Expr):
    def __init__(self, op:str,name=None, *args):
        super().__init__(op,name, *args)
        self.name = name

    def accept(self, visitor: ExprVisitor):
        return visitor.visitVariableExpr(self)

class Constant(Expr):
    def __init__(self, op:str,name=None, *args):
        super().__init__(op,name, *args)
        self.name = name

    def accept(self, visitor: ExprVisitor):
        return visitor.visitConstantExpr(self)
