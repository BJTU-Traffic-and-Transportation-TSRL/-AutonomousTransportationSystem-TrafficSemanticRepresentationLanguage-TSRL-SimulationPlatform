"""
Version:1.0
Date:2025/4/4
Author:oufan
这是解释器或执行器，应是和仿真对接的部分，包含解释类Interpreter，Interpreter读取输入的语法树流（其实就是一个List<Stmt>），输出为仿真环境中对应的执行命令。
现在输入的谓词表达式中的谓词Token会暂存在temp中，读取temp中的谓词在方法interpret中的try中修改代码，可以根据输入的内容得到不同的输出结果
现在的输入文件为input，输出文件为output
"""

from typing import List

import Expr
import Stmt
import Inference_engine
from Tokentype import Token
from Tokentype import TokenType
# from LoxCallable import LoxCallable
# from LoxFunction import LoxFunction
# from LoxInstance import LoxInstance
# from LoxClass import LoxClass
import RuntimeError
import errorHanding
# import Environment
# import time
# from Return import Return


# 现在输入的谓词表达式中的谓词Token会暂存在temp中，读取temp中的谓词在方法interpret中的try中修改代码，可以根据输入的内容得到不同的输出结果
# 现在的输入文件为input，输出文件为output


class Interpreter(Expr.ExprVisitor, Stmt.StmtVisitor):
    kb = Inference_engine.FolKB()
    PreTemp = [] #暂存谓词
    ArgTemp = [] #暂存谓词的参数
    # single=[]
    def __init__(self):
        pass

    def interpret(self,  statements:List):
        for statement in statements:

            self.PreTemp = []
            self.ArgTemp = []
            self.__execute__(statement)
            with open('output.txt', 'w', encoding='utf-8') as file:
                if self.PreTemp[0].lexeme=="Vehicletype":
                    stringNum = self.ArgTemp[0][0]
                    stringNum = str(stringNum)
                    file.write("LetGoStopLine("+stringNum+");")
                else:
                    file.write(self.PreTemp[0].lexeme)

    def __evaluate__(self, expr: Expr.Expr):
        return expr.accept(self)

    def __execute__(self,stmt:Stmt.Stmt):
        stmt.accept(self)

    def visitExpressionStmt(self,stmt:Stmt.Expression):
        self.__evaluate__(stmt.expression)
        return None

    def visitLogicalExpr(self, expr:Expr.Logical):  #引入短路设计
        left = self.__evaluate__(expr.left)
        right = self.__evaluate__(expr.right)
        # self.temp.append(left)
        # self.temp.append(right)
        # print(left)
        # print(right)
        # if expr.operator.type == TokenType.OR:
        #     if self.__isTruthy__(left):
        #         return left
        #     else:
        #         if not self.__isTruthy__(left):
        #             return left

        # return self.__evaluate__(expr.right)

        return None

    #字面量求值
    def visitLiteral(self, expr: Expr.Literal):
        return expr.value

    def visitVariableExpr(self, expr: Expr.Variable ):
        #return self.__environment__.get(expr.name)
        # return self.__lookUpVariable__(expr.name, expr)
        return expr.name

    # def visitCallExpr(self,expr:Expr.Call)->object:
    #     # callee = self.__evaluate__(expr.callee) #通过赋值传回一个Loxfunction对象
    #     callee = self.__evaluate__(expr.callee) #此时传回callee的名字，他是一个token
    #     self.PreTemp.append(callee)
    #     arguments = []
    #     for argument in expr.arguments:
    #         arguments.append(self.__evaluate__(argument))
    #     self.ArgTemp.append(arguments)
    #     # # 对函数头进行类型检验
    #     # if  not isinstance(callee, LoxCallable):
    #     #     raise  RuntimeError.CustomRuntimeError(expr.paren,"Can only call functions and classes.")
    #     # function:LoxCallable = callee
    #     # #元数检验
    #     # if len(arguments) != function.arity():
    #     #     raise RuntimeError.CustomRuntimeError(expr.paren, "Expected " +str(function.arity()) + " arguments but got " +str(len(arguments)) + ".")
    #     # return function.call(self, arguments)
    #     return callee