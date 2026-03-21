import sys
import os
# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import errorHanding
from Tokentype import *
import Expr 
import Stmt


class ParseError(Exception):
    pass
    # def __init__(self, token, message):
    #     super().__init__(f"{message} at line {token.line}, column {token.column}")
    #     self.token = token

class Parser:
    def __init__(self, tokens: List[Token]):
        self.tokens = tokens
        self.current = 0 # 当前解析的token索引

    def parse(self)->List[Stmt.Stmt]:
        # 解析token流，生成抽象语法树（AST）
        statements: List[Stmt.Stmt] = [] # 初始化语法类型列表
        while not self.is_at_end():
            decl = self.__declaration__() # 解析语句类型，进行语句预声明
            if decl is not None:
                statements.append(decl)

        return statements
        # try:
        #     return self.__expression__()
        # except ParseError as e:
        #     return None

    def __expression__(self) -> Expr.Expr:
        return self.__implicate__() # 调用__implicate__()方法，解析蕴含表达式

    def __declaration__(self) ->Stmt.Stmt:
        # 声明语句解析
        try:#暂无声明项
            return self.__statement__() # 解析语句，进行语句声明
        except ParseError as error:
            self.synchronize()
            return None

    def __statement__(self) ->Stmt.Stmt:
        # 根据 token 类型解析不同类型的语句
        if self.match(TokenType.LET):  # 新增Let语句匹配
            return self.__LetStatement__()
        if self.match(TokenType.ASK):
            return self.__ASKStatement__()
        if self.match(TokenType.PRINT):
            return self.__printStatement__()
        if self.match(TokenType.TELL):  # 新增 Tell 语句匹配
            return self.__TellStatement__()
        return self.__expressionStatement__()


    def __ASKStatement__(self) ->Stmt.Stmt:
        value = self.__expression__()
        self.consume(TokenType.SEMICOLON, "Expect ';' after value.")
        return Stmt.Ask(value)

    def __printStatement__(self) ->Stmt.Stmt:
        # self.consume(TokenType.LEFT_PAREN, "Expect ';' after value.")
        value = self.__expression__()
        # self.consume(TokenType.RIGHT_PAREN, "Expect ';' after value.")
        self.consume(TokenType.SEMICOLON, "Expect ';' after value.")
        return Stmt.Print(value)

    def __LetStatement__(self) -> Stmt.Stmt:
        # 解析谓词名（CheckChangeLane）
        pred_name = self.consume(TokenType.IDENTIFIER, "Expect predicate name after 'Let'.")
        # 解析左括号
        self.consume(TokenType.LEFT_PAREN, "Expect '(' after predicate name.")
        # 解析参数（Car_1）
        params = []
        if not self.check(TokenType.RIGHT_PAREN):
            while True:
                param = self.consume(TokenType.IDENTIFIER, "Expect parameter name in predicate.")
                params.append(param)
                if not self.match(TokenType.COMMA):
                    break
        # 解析右括号
        self.consume(TokenType.RIGHT_PAREN, "Expect ')' after parameters.")
        # 解析蕴含符 :-
        impl_token = self.consume(TokenType.IMPLICATE, "Expect ':-' after predicate parameters.")
        # 解析蕴含的表达式（IsSelfVehicle(Car_1) ∧ HasSituation(Car_0,EmergencyStop)）
        impl_expr = self.__expression__()
        # 解析分号结束
        self.consume(TokenType.SEMICOLON, "Expect ';' after let statement.")

        # 构造Let语句（复用Implication表达式封装）
        pred_expr = Expr.Predicate(pred_name.lexeme, pred_name, *params)
        let_expr = Expr.Implication(impl_token, impl_expr, pred_expr)
        return Stmt.Expression(let_expr)

    def __TellStatement__(self) -> Stmt.Stmt:
        value = self.__expression__()
        self.consume(TokenType.SEMICOLON, "Expect ';' after Tell statement.")
        return Stmt.Tell(value)

    def __expressionStatement__(self) ->Stmt.Stmt:
        expr = self.__expression__()
        self.consume(TokenType.SEMICOLON, "Expect ';' after expression.")
        return Stmt.Expression(expr)


    def __implicate__(self)-> Expr.Expr:
        # 蕴含表达式解析
        predicate = self.__Or__() # 调用__Or__()方法，解析析取表达式
        if self.match(TokenType.IMPLICATE):
            op = self.previous()
            value = self.__Or__()
            if self.match(TokenType.COMMA):
                while (True):
                    next_pred = self.__Or__()
                    value  = Expr.Predicate('&',Token(TokenType.AND,'&',None,self.previous().line), value,next_pred)
                    if not self.match(TokenType.COMMA):
                        break
            return Expr.Implication(op,value,predicate)

        return predicate

    def __Or__(self):
        # 析取表达式解析
        expr = self.__And__() # 调用__And__()方法，解析合取表达式
        while self.match(TokenType.OR):
            operator = self.previous()
            right = self.__And__()
            expr = Expr.Logical(expr, operator, right)
        return expr

    def __And__(self):
        # 合取表达式解析
        expr = self.__equality__() # 调用__equality__()方法，解析等值表达式
        while self.match(TokenType.AND):
            operator = self.previous()
            right = self.__equality__()
            expr = Expr.Logical(expr, operator, right)
        return expr


#等值运算
    def __equality__(self) -> Expr:
        # 等值表达式解析
        expr = self.__comparison__() # 调用__comparison__()方法，解析比较表达式
        while self.match(TokenType.BANG_EQUAL, TokenType.EQUAL_EQUAL):
            operator = self.previous()
            right = self.__comparison__()
            expr = Expr.Binary(expr, operator.lexeme, operator, right)
        return expr

#比较运算
    def __comparison__(self) -> Expr:
        # 比较表达式解析
        expr = self.__term__() # 调用__term__()方法，解析项表达式
        while self.match(TokenType.GREATER, TokenType.GREATER_EQUAL, TokenType.LESS, TokenType.LESS_EQUAL):
             operator = self.previous()
             right = self.__term__()
             expr = Expr.Binary(expr, operator.lexeme, operator, right)
        return expr

#加减运算
    def __term__(self) -> Expr:
        # 项表达式解析
        expr = self.__factor__() # 调用__factor__()方法，解析因子表达式
        while self.match(TokenType.MINUS, TokenType.PLUS):
            operator = self.previous()
            right = self.__factor__()
            expr = Expr.Binary(expr, operator.lexeme, operator, right)
        return expr

#乘除运算
    def __factor__(self) -> Expr:
        # 因子表达式解析
        expr = self.__unary__() # 调用__unary__()方法，解析一元表达式
        while self.match(TokenType.SLASH, TokenType.STAR):
            operator = self.previous()
            right = self.__factor__()
            expr = Expr.Binary(expr, operator.lexeme, operator, right)
        return expr

#一元运算
    def __unary__(self) -> Expr:
        # 一元表达式解析
        if self.match(TokenType.BANG, TokenType.MINUS):
            operator = self.previous()
            right = self.__unary__()
            return Expr.Unary(operator, right)
        return self.__predicate__() # 调用__predicate__()方法，解析谓词表达式

    def __predicate__(self)-> Expr.Expr:
        # 谓词表达式解析
        predicate = self.__primary__() # 调用__primary__()方法，解析基本表达式
        args = []
        if self.match(TokenType.LEFT_PAREN): # 解析左括号
            if not self.check(TokenType.RIGHT_PAREN): # 检查是否到达右括号
                while (True):
                    if len(args) >= 255:
                        self.error(self.peek(), "Can't have more than 255 parameters.")
                    args.append(self.__predicate__())
                    if not self.match(TokenType.COMMA): # 检查是否到达逗号
                        break
            self.consume(TokenType.RIGHT_PAREN, "Expect ')' after parameters.")

            return Expr.Predicate(predicate.token.lexeme,predicate.token,*tuple(args))
        return  predicate

#最顶级“原子”
    def __primary__(self):
        # 基本表达式解析（常量、变量、标识符等）
        if self.match(TokenType.FALSE):
            return  Expr.Constant('False',self.previous())
        if self.match(TokenType.TRUE):
            return Expr.Constant('True',self.previous())
        if self.match(TokenType.NIL):
            return Expr.Constant('Nil',self.previous())
        if self.match(TokenType.NUMBER, TokenType.STRING):
            return Expr.Constant(self.previous().literal,self.previous())
        if self.match(TokenType.IDENTIFIER):
            if self.previous().lexeme[0].islower() or self.previous().lexeme=="_": # 标识符以小写字母或下划线开头（此处因为match方法返回的是True，所以current+1，指向了下一个字符，便使用previous()方法获取到了原先标识符的第一个字符）
                return Expr.Variable(self.previous().lexeme,self.previous())
            elif self.previous().lexeme[0].isupper(): # 标识符以大写字母开头（使用previous的原因同上）
                return Expr.Constant(self.previous().lexeme,self.previous()) # 常量（大写字母开头的标识符）

        # if self.match(TokenType.LEFT_PAREN):
        #     expr = self.__expression__()
        #     self.consume(TokenType.RIGHT_PAREN, "Expect ')' after expression.")
        #     return Expr.Grouping(expr)

        raise self.error(self.peek(), "Expect expression.")

    # 其他解析方法...

    def peek(self) -> Token:
        return self.tokens[self.current]

    def peek_next(self):
        if self.current + 1 >= len(self.tokens):
            return '\0'
        return self.tokens[self.current + 1]

    def advance(self) -> Token:
        if not self.is_at_end():
            self.current += 1
        return self.previous()

    def is_at_end(self) -> bool:
        return self.peek().type == TokenType.EOF
        #return self.current >= len(self.tokens)

    def check(self, type: TokenType) -> bool :
        if self.is_at_end():
            return False
        return self.peek().type == type

    def match(self, *types: TokenType) -> bool:
        for type in types:
            if self.check(type):
                self.advance()
                return True
        return False

    def previous(self) -> Token:
        return self.tokens[self.current - 1]

    def error(self, token: Token, message: str) -> ParseError:
        errorHanding.parseError(token, message)
        e = ParseError()
        return e
        #raise SyntaxError(f"[line {token.line}] Error at '{token.lexeme}': {message}")

    def consume(self, type: TokenType, message: str) -> Token: # 消费指定类型的token，若不是则报错
        if self.check(type):
            return self.advance() # current+1
        raise  self.error(self.peek(), message)

    def synchronize(self):
        self.advance()
        while not self.is_at_end():
            if self.previous().type == TokenType.SEMICOLON: #此处认为结尾符为“;”
                return
            peeked_token_type = self.peek().type #当遇到以下保留字时，可认为下一条语句开始了
            # if peeked_token_type in [
            #     TokenType.CLASS,
            #     TokenType.FUN,
            #     TokenType.VAR,
            #     TokenType.FOR,
            # ]:
            #     return
            self.advance()
