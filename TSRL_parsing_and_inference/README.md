# TSRL解析器模块使用说明
本模块提供了一个基于交通语义表示语言(Traffic Semantic Representation Language,TSRL)的解析器，可用于交通场景和知识的语义表达和解析推理。

## 使用方法

### 基本用法
TSRL.py 是TSRL语义解析器的主程序入口，可以通过以下方式运行：
```python
from TSRL import TSRL
# 设置输入文件和输出文件路径
input_file = "TSRL_representation\Infer_input\input_1.txt"
output_file = "TSRL_representation\Infer_output\output.txt"
# 运行TSRL解释器
TSRL.main(input_file, output_file)
```
### 参数说明
- `input_file`: TSRL解析器的输入文件路径
例：`Infer_input\input_1.txt`，文件内容如下所示
    ```
    SelfVehicle(0);
    IsJunction(7);
    EmergencyStation(3);
    HasNextJunction(0,7);
    LetStop(1);
    LetStop(2);
    EmergencyStation(2);
    EmergencyStation(2);
    StopAt(2,9);
    EmergencyStation(1);
    StopAt(2,7);
    Congestion(7);

    LetStopBeforeJunction(x):-HasNextJunction(x,y),Congestion(y);//y拥堵，x车前方是交叉口y

    ASK LetStopBeforeJunction(x);
    ```

    - `input_file` 输入文件的内容分为3个部分，分别为：**交通事实谓词语句，匹配的语义规则，询问语句**。
        1. 交通事实谓词语句：用于描述交通场景中的事实，例如`SelfVehicle(0);`表示车辆0是自车，`IsJunction(7);`表示交通主体7是一个交叉口。
        2. 匹配的语义规则：用于定义交通场景中匹配到的语义规则，例如`LetStopBeforeJunction(x):-HasNextJunction(x,y),Congestion(y);//y拥堵，x车前方是交叉口y`表示在交叉口y拥堵时，车辆x需要停止。
        3. 询问语句：用于查询交通场景中的信息，例如`ASK LetStopBeforeJunction(x);`表示查询到基于事实的匹配规则中，哪些车辆x需要停止。

- `output_file`: 输出结果的文件路径（可选，如果不提供，默认输出到`TSRL_representation\Infer_output\output.txt`）
例：`Infer_output\output.txt`，文件内容如下所示
    ```
    x = 0
    ```
    对应输入文件中的询问语句`ASK LetStopBeforeJunction(x);`，表示查询到基于事实的匹配规则中，车辆x=0需要停止。
## TSRL.py 函数说明
### TSRL 类
TSRL是程序的主类，包含以下静态方法：

#### `main(input_file, output_file=None)`
主函数，接收输入文件路径和可选的输出文件路径，执行TSRL代码的解释过程。

#### `__run_file(file_path)`
从文件中读取TSRL代码并执行。

#### `__run_prompt()`
交互式运行模式，允许用户逐行输入TSRL代码并执行。

#### `__run(source)`
核心执行函数，对TSRL源代码进行词法分析、语法分析和解释执行。

## 模块说明

### Scanner.py
词法分析器，将TSRL源代码转换为标记(token)流。
- 负责识别源代码中的关键字、标识符、运算符等
- 生成标记流供语法分析器使用

#### Tokentype-文本字符 对应表

| TokenType | 对应字符/符号 | 说明 |
|-----------|--------------|------|
| **单字符标记** | | |
| LEFT_PAREN | `(` | 左括号 |
| RIGHT_PAREN | `)` | 右括号 |
| LEFT_BRACE | `{` | 左花括号 |
| RIGHT_BRACE | `}` | 右花括号 |
| COMMA | `,` | 逗号 |
| DOT | `.` | 点号 |
| MINUS | `-` | 减号 |
| PLUS | `+` | 加号 |
| SEMICOLON | `;` | 分号 |
| SLASH | `/` | 斜杠 |
| STAR | `*` | 星号 |
| **逻辑运算符** | | |
| OR | `∨` | 析取（或） |
| AND | `∧` | 合取（与） |
| **比较运算符** | | |
| BANG | `!` | 感叹号 |
| BANG_EQUAL | `!=` | 不等于 |
| EQUAL | `=` | 等号 |
| EQUAL_EQUAL | `==` | 等于 |
| LESS | `<` | 小于 |
| LESS_EQUAL | `<=` | 小于等于 |
| GREATER | `>` | 大于 |
| GREATER_EQUAL | `>=` | 大于等于 |
| **TSRL 特殊运算符** | | |
| IMPLICATE | `:-` | 蕴含符 |
| ASK | `?-` | 提问符 |
| **字面量** | | |
| STRING | `"..."` | 字符串（双引号包围） |
| NUMBER | 数字 | 整数或浮点数 |
| IDENTIFIER | 标识符 | 以字母或下划线开头 |
| **关键字** | | |
| PRINT | `PRINT` | 打印关键字 |
| AND | `and` | 逻辑与关键字 |
| OR | `or` | 逻辑或关键字 |
| FALSE | `false` | 假值 |
| TRUE | `true` | 真值 |
| FOR | `for` | 循环关键字 |
| NIL | `nil` | 空值 |
| LET | `Let` | Let语句关键字 |
| TELL | `Tell` | Tell语句关键字 |
| **其他** | | |
| EOF | 文件结束 | 文件结束标记 |
| PREDICATE | 谓词 | 谓词声明 |
| VAR | 变量 | 变量声明 |
| FUN | 函数 | 函数声明 |
| RULE | 规则 | 规则声明 |

**注释：**
- 单行注释：`//` 后的内容直到行末
- 空白字符（空格、制表符、换行符）会被忽略

### Parser.py
语法分析器，将标记流转换为抽象语法树(AST)。
- 实现了TSRL语言的语法规则
- 生成表达式和语句的抽象语法树

### Interpreter.py
解释器，执行抽象语法树并与推理引擎交互。
- 实现了表达式和语句的访问者模式
- 维护知识库(KB)和置换表
- 处理ASK和PRINT等语句
- 设置输出文件路径

### Inference_engine.py
推理引擎，处理知识库和查询。
- 实现了一阶逻辑知识库(KB)的存储和推理
- 提供了推理算法和置换机制

### errorHanding.py
错误处理模块，管理词法、语法和运行时错误。
- 记录错误标志
- 提供错误报告功能
- 处理运行时异常

### Expr.py
表达式类定义，包含各种表达式的抽象表示。

### Stmt.py
语句类定义，包含各种语句的抽象表示。

### Tokentype.py
标记类型定义，包含TSRL语言中所有标记的类型枚举。

### txt_representation.py
某一语句的词法分析及语法分析可视化。
- 将某一语句词法解析对应的标记(token)流和语法解析对应的抽象语法树(AST)的结果输出在instruction_input.txt

## 工作流程

1. 从输入文件读取TSRL源代码
2. Scanner将源代码转换为标记流
3. Parser将标记流转换为抽象语法树
4. Interpreter执行抽象语法树，将合法谓词表达式存储在知识库中
5. 当遇到ASK语句时，通过推理引擎进行推理
6. 推理结果输出到指定的输出文件

## 输入输出示例

### 输入文件 (input_1.txt)
```
# TSRL语言代码示例
# 定义交通规则
SelfVehicle(0);
IsJunction(7);
EmergencyStation(3);
HasNextJunction(0,7);
LetStop(1);
LetStop(2);
EmergencyStation(2);
EmergencyStation(2);
StopAt(2,9);
EmergencyStation(1);
StopAt(2,7);
Congestion(7);

LetStopBeforeJunction(x):-HasNextJunction(x,y),Congestion(y);//y拥堵，x车前方是交叉口y

# 询问查询
ASK LetStopBeforeJunction(x);
```

### 输出文件 (output.txt)
```
x = 0
```

## 注意事项

1. 确保输入文件路径正确，程序将从指定的输入文件读取TSRL代码
2. 如果不指定输出文件，默认输出到`TSRL_representation\Infer_output\output.txt`
3. 程序支持交互式运行模式，可以通过`__run_prompt()`方法启动
4. 所有错误信息将被记录并报告，包括词法错误、语法错误和运行时错误
