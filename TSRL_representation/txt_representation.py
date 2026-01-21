import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入原始模块
from Scanner import Scanner
from Parser import Parser
import Tokentype
from Tokentype import Token, TokenType
import Expr
import Stmt
import errorHanding


# 语法树可视化类（适配原始Expr/Stmt结构，增加对ASK等语句的支持）
class AstPrinter:
    def print(self, expr):
        if isinstance(expr, Expr.Implication):
            return self.visit_Implication_expr(expr)
        elif isinstance(expr, Expr.Logical):
            return self.visit_Logical_expr(expr)
        elif isinstance(expr, Expr.Predicate):
            return self.visit_Predicate_expr(expr)
        elif isinstance(expr, Expr.Constant):
            return self.visit_Constant_expr(expr)
        elif isinstance(expr, Expr.Variable):
            return self.visit_Variable_expr(expr)
        elif isinstance(expr, Expr.Binary):
            return self.visit_Binary_expr(expr)
        elif isinstance(expr, Expr.Unary):  # 新增对Unary表达式的支持
            return self.visit_Unary_expr(expr)
        return ""

    def visit_Constant_expr(self, expr):
        return f"└── [Cons] {expr.op}"

    def visit_Variable_expr(self, expr):
        return f"└── [Var] {expr.op}"

    def visit_Binary_expr(self, expr):
        parts = [f"└── [Binary:{expr.operator.lexeme}]"]
        parts.append(self.indent(self.print(expr.left)))
        parts.append(self.indent(self.print(expr.right)))
        return '\n'.join(parts)

    def visit_Unary_expr(self, expr):  # 新增Unary表达式处理
        parts = [f"└── [Unary:{expr.operator.lexeme}]"]
        parts.append(self.indent(self.print(expr.right)))
        return '\n'.join(parts)

    def visit_Logical_expr(self, expr):
        parts = [f"└── [Logical] {expr.operator.lexeme}"]
        parts.append(self.indent(self.print(expr.left)))
        parts.append(self.indent(self.print(expr.right)))
        return '\n'.join(parts)

    def visit_Predicate_expr(self, expr):
        parts = [f"└── [Pred] {expr.op}"]
        for arg in expr.args:
            parts.append(self.indent(self.print(arg)))
        return '\n'.join(parts)

    def visit_Implication_expr(self, expr):
        parts = [
            f"└── [Implicate] {expr.token.lexeme}",
            self.indent(self.print(expr.args[0])),
            self.indent(self.print(expr.args[1]))
        ]
        return '\n'.join(parts)

    def indent(self, text, level=1):
        indent_str = '\t' * level
        lines = text.split('\n')
        return '\n'.join([indent_str + line if line else line for line in lines])


# 核心功能函数（增加对ASK等语句的处理）
def process_instruction():
    # 重置错误标志
    errorHanding.hadError = False

    # 1. 定义文件路径
    file_path = "instruction_input.txt"
    if not os.path.exists(file_path):
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write("")
        print(f"已创建文件: {file_path}")

    # 2. 预设测试输入（增加ASK语句示例，可切换为手动输入）
    # 支持Let和ASK两种语句测试
    #test_input = "ASK IsEmergency(Car_0) ∨ HasWarning(Lane_1);"  # ASK语句示例
    #test_input = "Let CheckChangeLane(Car_1):-IsSelfVehicle(Car_1) ∧ HasSituation(Car_0,EmergencyStop);"  # Let语句示例
    #print(f"\n使用测试输入: {test_input}")
    #user_input = test_input
   #如需手动输入，注释上面两行，启用下面两行
    print("\n请输入指令（示例1：Let CheckChangeLane(Car_1):-IsSelfVehicle(Car_1) ∧ HasSituation(Car_0,EmergencyStop); 示例2：ASK IsEmergency(Car_0) ∨ HasWarning(Lane_1);）：")
    user_input = input("> ")

    # 3. 词法分析（生成词元流）
    scanner = Scanner(user_input)
    tokens = scanner.scan_tokens()

    # 4. 收集词元流内容（含格式化处理）
    token_output = ["===== 词元流 ====="]
    for token in tokens:
        literal_str = token.literal if token.literal is not None else ""
        # 修正逻辑运算符的字面量展示
        if token.type == TokenType.AND:
            literal_str = "∧"
        if token.type == TokenType.OR:
            literal_str = "∨"
        token_line = f"Token({token.type.name}, {token.lexeme}, {literal_str}, {token.line})"
        token_output.append(token_line)

    # 5. 语法分析（生成语法树，兼容ASK和Let语句）
    tree_output = ["\n===== 语法树 ====="]
    if not errorHanding.hadError:
        parser = Parser(tokens)
        statements = parser.parse()
        printer = AstPrinter()
        if statements and len(statements) > 0:
            # 根据语句类型处理不同根节点
            stmt = statements[0]
            if isinstance(stmt, Stmt.Expression):
                # Let语句（Expression类型包装）
                root_expr = stmt.expression
                if isinstance(root_expr, Expr.Implication):
                    tree_str = f"└── [Let] Let\n{printer.indent(printer.print(root_expr))}"
                    tree_output.append(tree_str)
            elif isinstance(stmt, Stmt.Ask):
                # ASK语句处理
                ask_expr = stmt.expression
                tree_str = f"└── [Ask] ASK\n{printer.indent(printer.print(ask_expr))}"
                tree_output.append(tree_str)
            elif isinstance(stmt, Stmt.Print):
                # PRINT语句处理
                print_expr = stmt.expression
                tree_str = f"└── [Print] PRINT\n{printer.indent(printer.print(print_expr))}"
                tree_output.append(tree_str)
            elif isinstance(stmt, Stmt.Print):
                # PRINT语句处理
                print_expr = stmt.expression
                tree_str = f"└── [Print] PRINT\n{printer.indent(printer.print(print_expr))}"
                tree_output.append(tree_str)
            elif isinstance(stmt, Stmt.Tell):
                # PRINT语句处理
                print_expr = stmt.expression
                tree_str = f"└── [Tell] Tell\n{printer.indent(printer.print(print_expr))}"
                tree_output.append(tree_str)
            else:
                # 通用表达式处理
                tree_str = printer.print(stmt.expression)
                tree_output.append(tree_str)
    else:
        tree_output.append("词法分析出错，跳过语法分析")

    # 6. 整合内容写入文件（用户输入 + 词元流 + 语法树）
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write("===== 用户输入 =====\n")
        f.write(user_input + "\n\n")
        f.write('\n'.join(token_output) + "\n")
        f.write('\n'.join(tree_output) + "\n")

    # 7. 控制台输出（保持原有显示）
    print("\n===== 词元流 =====")
    print('\n'.join(token_output[1:]))  # 跳过标题行
    print("\n===== 语法树 =====")
    print('\n'.join(tree_output[1:]))   # 跳过标题行


if __name__ == "__main__":
    process_instruction()