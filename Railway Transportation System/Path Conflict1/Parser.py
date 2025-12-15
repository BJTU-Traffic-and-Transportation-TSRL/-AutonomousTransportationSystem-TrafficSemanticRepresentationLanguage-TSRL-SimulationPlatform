import errorHanding
from Tokentype import *
import Expr 
import Stmt
import errorHanding


class ParseError(Exception):
    pass
    # def __init__(self, token, message):
    #     super().__init__(f"{message} at line {token.line}, column {token.column}")
    #     self.token = token

class Parser:
    def __init__(self, tokens: List[Token]):
        self.tokens = tokens
        self.current = 0

    def parse(self)->List[Stmt.Stmt]:
        statements: List[Stmt.Stmt] = []
        while not self.is_at_end():
            statements.append(self.__declaration__())

        return statements
        # try:
        #     return self.__expression__()
        # except ParseError as e:
        #     return None

    def __expression__(self) -> Expr.Expr:
        return self.__implicate__()

    def __declaration__(self) ->Stmt.Stmt:
        try:#暂无声明项
            return self.__statement__()
        except ParseError as error:
            self.synchronize()
            return None

    def __statement__(self) ->Stmt.Stmt:
        if self.match(TokenType.ASK):
            return self.__ASKStatement__()
        if self.match(TokenType.PRINT):
            return self.__printStatement__()
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


    def __expressionStatement__(self) ->Stmt.Stmt:
        expr = self.__expression__()
        self.consume(TokenType.SEMICOLON, "Expect ';' after expression.")
        return Stmt.Expression(expr)


    def __implicate__(self)-> Expr.Expr:
        predicate = self.__Or__()
        if self.match(TokenType.IMPLICATE):
            op = self.previous()
            value = self.__predicate__()
            if self.match(TokenType.COMMA):
                while (True):
                    next_pred = self.__predicate__()
                    value  = Expr.Predicate('&',Token(TokenType.AND,'&',None,self.previous().line), value,next_pred)
                    if not self.match(TokenType.COMMA):
                        break
            return Expr.Implication(op,value,predicate)

        return predicate

    def __Or__(self):
        expr = self.__And__()
        while self.match(TokenType.OR):
            operator = self.previous()
            right = self.__And__()
            expr = Expr.Logical(expr, operator, right)
        return expr

    def __And__(self):
        expr = self.__equality__()
        while self.match(TokenType.AND):
            operator = self.previous()
            right = self.__equality__()
            expr = Expr.Logical(expr, operator, right)
        return expr


#等值运算
    def __equality__(self) -> Expr:
        expr = self.__comparison__()
        while self.match(TokenType.BANG_EQUAL, TokenType.EQUAL_EQUAL):
            operator = self.previous()
            right = self.__comparison__()
            expr = Expr.Binary(expr, operator.lexeme, operator, right)
        return expr

#比较运算
    def __comparison__(self) -> Expr:
        expr = self.__term__()
        while self.match(TokenType.GREATER, TokenType.GREATER_EQUAL, TokenType.LESS, TokenType.LESS_EQUAL):
             operator = self.previous()
             right = self.__term__()
             expr = Expr.Binary(expr, operator.lexeme, operator,right)
        return expr

#加减运算
    def __term__(self) -> Expr:
        expr = self.__factor__()
        while self.match(TokenType.MINUS, TokenType.PLUS):
            operator = self.previous()
            right = self.__factor__()
            expr = Expr.Binary(expr, operator.lexeme, operator, right)
        return expr

#乘除运算
    def __factor__(self) -> Expr:
        expr = self.__unary__()
        while self.match(TokenType.SLASH, TokenType.STAR):
            operator = self.previous()
            right = self.__factor__()
            expr = Expr.Binary(expr, operator.lexeme, operator, right)
        return expr

#二元运算
    def __unary__(self) -> Expr:
        if self.match(TokenType.BANG, TokenType.MINUS):
            operator = self.previous()
            right = self.__unary__()
            return Expr.Unary(operator, right)
        return self.__predicate__()

    def __predicate__(self)-> Expr.Expr:
        predicate = self.__primary__()
        args = []
        if self.match(TokenType.LEFT_PAREN):
            if not self.check(TokenType.RIGHT_PAREN):
                while (True):
                    if len(args) >= 255:
                        self.error(self.peek(), "Can't have more than 255 parameters.")
                    args.append(self.__predicate__())
                    if not self.match(TokenType.COMMA):
                        break
            self.consume(TokenType.RIGHT_PAREN, "Expect ')' after parameters.")

            return Expr.Predicate(predicate.token.lexeme,predicate.token,*tuple(args))
        return  predicate

#最顶级“原子”
    def __primary__(self):
        if self.match(TokenType.FALSE):
            return  Expr.Constant('False',self.previous())
        if self.match(TokenType.TRUE):
            return Expr.Constant('True',self.previous())
        if self.match(TokenType.NIL):
            return Expr.Constant('Nil',self.previous())
        if self.match(TokenType.NUMBER, TokenType.STRING):
            return Expr.Constant(self.previous().literal,self.previous())
        if self.match(TokenType.IDENTIFIER):
            if self.previous().lexeme[0].islower() or self.previous().lexeme=="_":
                return Expr.Variable(self.previous().lexeme,self.previous())
            elif self.previous().lexeme[0].isupper():
                return Expr.Constant(self.previous().lexeme,self.previous())

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

    def consume(self, type: TokenType, message: str) -> Token:
        if self.check(type):
            return self.advance()
        raise  self.error(self.peek(), message)

    def synchronize(self):
        self.advance()
        while not self.is_at_end :
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
