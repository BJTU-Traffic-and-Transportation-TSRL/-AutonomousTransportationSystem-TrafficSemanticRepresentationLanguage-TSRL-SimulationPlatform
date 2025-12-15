"""
Version:1.0
Date:2025/5/26
Author:oufan
这是解释器或执行器，应是和仿真对接的部分，包含解释类Interpreter，Interpreter读取输入的语法树流（其实就是一个List<Stmt>），
解析后得到的合法谓词表达式会存储在知识库kb中，当使用ASK询问语句时，得到的推理结果(置换)会存储在output.txt文件中
现在的输入文件为test，输出文件为output。（test文件中有一个家族推理的例子）
"""
import json
from typing import List

from networkx import selfloop_edges

import Expr
import Stmt
import Inference_engine
from Tokentype import Token
from Tokentype import TokenType
import RuntimeError
import errorHanding
# import Environment
# import time
# from Return import Return




class Interpreter(Expr.ExprVisitor, Stmt.StmtVisitor):

    def __init__(self):
        self.kb = Inference_engine.FolKB()  # 存储知识库
        self.subset = {} # 储存置换表

    def interpret(self,  statements:List[Stmt.Stmt]):
        try:
            for statement in statements:
                self.__execute__(statement)

        except RuntimeError.CustomRuntimeError  as error:
            errorHanding.runtimeError(error)

    def __evaluate__(self, expr: Expr.Expr):
        return expr.accept(self)

    def __execute__(self,stmt:Stmt.Stmt):
        stmt.accept(self)


    def visitExpressionStmt(self,stmt:Stmt.Expression):
        """
        执行表达式语句，目的是将可行的逻辑语句载入知识库kb中
        """
        self.kb.tell(self.__evaluate__(stmt.expression))
        return None

    def visitPrintStmt(self,stmt:Stmt.Print):
        """
        执行打印输出语句，目的是输出print（）括号中的语句
        """
        value = self.__evaluate__(stmt.expression)
        print(value)
        return None

    def visitAskStmt(self, stmt):
        """
        执行询问语句，目的是推断ASK后的语句是否为真，并返回可能的置换,并写入output.txt文件中
        """
        Dict = self.kb.ask(self.__evaluate__(stmt.expression))
        d={}
        if Dict is not False:
            for key, value in Dict.items():
                if str(key)[:2] == "v_" and len(str(key))>9: #这里对于无关变量须在斟酌
                    pass
                else:
                    d[str(key)]=str(value)
            with open('output.txt', 'a', encoding='utf-8') as file:
                file.write(json.dumps(d, ensure_ascii=False) + "\n")
            print(d)
        else:
            with open('output.txt', 'w', encoding='utf-8') as file:
                file.write('False')
            print(Dict)


    def visitImplicationExpr(self, expr):
        """
        蕴含表达式不作处理，直接返回
        """
        antecedent, consequent = expr.args
        args = self.conjuncts(antecedent)
        value = self.__evaluate__(args[0])
        for arg in args[1:]:
            next_expr = self.__evaluate__(arg)
            value = Expr.Predicate('&', Token(TokenType.AND, '&', None, arg.token.line), value, next_expr)
        return Expr.Implication(expr.token,value,consequent)


    def visitPredicateExpr(self, expr):
        """
        谓词表达式不作处理，直接返回
        """
        return expr

    def visitLogicalExpr(self, expr:Expr.Logical):
        left = self.__evaluate__(expr.left)
        right = self.__evaluate__(expr.right)

        return None

    def visitBinary(self, expr:Expr.Binary):
        '''
        四则运算 不可计算的比如存在参数是未赋值的变量的，要暂存为Expr.Binary;可计算的要给出结果，非法的计算表达式应报错
        x>5+9
        '''
        left :Expr.Expr = self.__evaluate__(expr.left)
        right:Expr.Expr = self.__evaluate__(expr.right)

        computable = 0

        # 四则运算 不可计算的比如存在参数是未赋值的变量的，要暂存为Expr.Binary
        if type(left) == Expr.Variable :
            left = Expr.Constant('0',Token(TokenType.NUMBER, None,0,-1))
            return expr
        elif type(right) == Expr.Variable:
            right = Expr.Constant('0', Token(TokenType.NUMBER, None, 0, -1))
            return expr
        # if self.visitBinary(Expr.Binary(left,expr.op,expr.operator,right)):


        #四则运算 可计算的要给出结果，非法的计算表达式应报错
        if expr.operator.type == TokenType.MINUS:
            if self.__checkNumberOperands__(expr.operator, left, right):
                return Expr.Constant(str(left.token.literal-right.token.literal),Token(TokenType.NUMBER, None,(left.token.literal-right.token.literal),-1))
            raise RuntimeError.CustomRuntimeError(expr.operator, "Operands must be two numbers .")
        elif expr.operator.type == TokenType.PLUS:
            if self.__checkNumberOperands__(expr.operator, left, right):
                return Expr.Constant(str(left.token.literal+right.token.literal),Token(TokenType.NUMBER, None,(left.token.literal+right.token.literal),-1))
            elif self.__checkStringOperands__(expr.operator, left, right):
                return Expr.Constant(str(left.token.literal+right.token.literal),Token(TokenType.STRING, None,(left.token.literal+right.token.literal),-1))
            raise RuntimeError.CustomRuntimeError(expr.operator, "Operands must be two numbers or two strings.")

        elif expr.operator.type == TokenType.SLASH:
            if self.__checkNumberOperands__(expr.operator, left, right):
                return Expr.Constant(str(left.token.literal/right.token.literal),Token(TokenType.NUMBER, None,(left.token.literal/right.token.literal),-1))
            raise RuntimeError.CustomRuntimeError(expr.operator, "Operands must be two numbers .")
        elif expr.operator.type == TokenType.STAR:
            if self.__checkNumberOperands__(expr.operator, left, right):
                return Expr.Constant(str(left.token.literal*right.token.literal),Token(TokenType.NUMBER, None,(left.token.literal*right.token.literal),-1))
            raise RuntimeError.CustomRuntimeError(expr.operator, "Operands must be two numbers .")

        #比较不等式判断
        elif expr.operator.type == TokenType.GREATER:
            if self.__checkNumberOperands__(expr.operator, left, right):
                if  left.token.literal > right.token.literal :
                    return Expr.Constant('True', Token(TokenType.TRUE, None,True ,-1))
                else:
                    return Expr.Constant('False',Token(TokenType.FALSE, None, False ,-1))
            raise RuntimeError.CustomRuntimeError(expr.operator, "Operands must be two numbers .")
        elif expr.operator.type == TokenType.GREATER_EQUAL:
            if self.__checkNumberOperands__(expr.operator, left, right):
                if  left.token.literal >= right.token.literal :
                    return Expr.Constant('True', Token(TokenType.TRUE, None, True ,-1))
                else:
                    return Expr.Constant('False',Token(TokenType.FALSE, None,False ,-1))
            raise RuntimeError.CustomRuntimeError(expr.operator, "Operands must be two numbers .")
        elif expr.operator.type == TokenType.LESS:
            if self.__checkNumberOperands__(expr.operator, left, right):
                if  left.token.literal < right.token.literal :
                    return Expr.Constant('True', Token(TokenType.TRUE, None, True ,-1))
                else:
                    return Expr.Constant('False',Token(TokenType.FALSE, None,False ,-1))
            raise RuntimeError.CustomRuntimeError(expr.operator, "Operands must be two numbers .")
        elif expr.operator.type == TokenType.LESS_EQUAL:
            if self.__checkNumberOperands__(expr.operator, left, right):
                if  left.token.literal <= right.token.literal :
                    return Expr.Constant('True', Token(TokenType.TRUE, None, True ,-1))
                else:
                    return Expr.Constant('False',Token(TokenType.FALSE, None,False ,-1))
            raise RuntimeError.CustomRuntimeError(expr.operator, "Operands must be two numbers .")

        #等式运算
        elif expr.operator.type == TokenType.BANG_EQUAL:
            if left != right :
                return Expr.Constant('True', Token(TokenType.TRUE, None, True ,-1))
            else:
                return Expr.Constant('False', Token(TokenType.FALSE, None, False, -1))
        elif expr.operator.type == TokenType.EQUAL_EQUAL:
            if left == right:
                return Expr.Constant('True', Token(TokenType.TRUE, None, True, -1))
            else:
                return Expr.Constant('False', Token(TokenType.FALSE, None, False, -1))
        return None

    def visitUnary(self, expr: Expr.Unary):
        right = self.__evaluate__(expr.right)

        if expr.operator.type == TokenType.MINUS:
            if self.__checkNumberOperand__(expr.operator, right):
                return Expr.Constant(str(-right.token.literal),Token(TokenType.NUMBER, None,-right.token.literal,-1))

        if expr.operator.type == TokenType.BANG: #什么是真？
            return not self.__isTruthy__(right)
        return None

    #字面量求值
    def visitLiteral(self, expr: Expr.Literal):
        return expr.value

    def visitConstantExpr(self, expr:Expr.Constant):
        return expr

    def visitVariableExpr(self, expr: Expr.Variable ):
        #return self.__environment__.get(expr.name)
        # return self.__lookUpVariable__(expr.name, expr)
        return expr

    #一元计算验证器，验证一元操作数是否为数字或变量
    def __checkNumberOperand__(self, operator, operand):
        if operand.token.type == TokenType.NUMBER:
            return True
        else:
            return False
        # raise RuntimeError.CustomRuntimeError(operator, "Operand must be a number.")

    #二元计算验证器，验证二元操作数是否为数字
    def __checkNumberOperands__(self, operator, left:Expr.Expr, right:Expr.Expr):
        if left.token.type == TokenType.NUMBER and right.token.type== TokenType.NUMBER:
            return True
        else:
            return False
        # raise RuntimeError.CustomRuntimeError(operator, "Operand must be a number.")

    def __checkStringOperands__(self, operator, left:Expr.Expr, right:Expr.Expr):
        if left.token.type == TokenType.STRING and right.token.type== TokenType.STRING:
            return True
        else:
            return False
        # raise RuntimeError.CustomRuntimeError(operator, "Operand must be a number.")

    def __isTruthy__(self, object)->bool:
        if object is None:
            return False
        if type(object) == bool:
            return object
        return True

    def __isEqual__(self,a,b):
        if a is None and b is None:
            return True
        if a is None:
            return False
        return a==b   #此处的比较运算需要考虑一下

    # 将合取式或析取式拆分为列表
    def dissociate(self, op, args):
        result = []
        def collect(subargs):
            for arg in subargs:
                if arg.op == op:
                    collect(arg.args)
                else:
                    result.append(arg)

        collect(args)
        return result

    # 将合取式拆分为列表
    def conjuncts(self,s):
        """
        输入合取式，输出合取式的各子式的列表.
        """
        return self.dissociate('&', [s])