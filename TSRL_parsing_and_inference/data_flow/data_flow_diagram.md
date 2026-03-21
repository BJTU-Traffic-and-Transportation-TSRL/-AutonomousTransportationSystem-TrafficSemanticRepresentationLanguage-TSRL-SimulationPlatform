# TSRL 系统数据流导向图

## 数据流导向图

```mermaid
---
config:
  theme: default
  layout: dagre
  look: classic
---
flowchart TB
 subgraph s1["输入层"]
        A["xxx.txt 输入文件"]
  end
 subgraph s2["入口层"]
        B["TSRL.py 入口文件"]
        B1["TSRL.main 主函数"]
        B2["TSRL.__run_file 读取文件"]
        B3["TSRL.__run 执行处理"]
  end
 subgraph s3["词法分析层"]
        C["Scanner.py 词法分析器"]
        C1["Scanner.scan_tokens 生成输入文件所有语句的tokens"]
        C2["Scanner.scan_token 针对单个字符生成token对象" ]
  end
 subgraph s4["语法分析层"]
        D["Parser.py 语法分析器"]
        D1["Parser.parse 生成语法树"]
        D2["Parser.\_\_declaration\_\_() 语句预声明"]
        D3["Parser.\_\_statement\_\_() 解析语句类型"]
        
        subgraph s4_1["语句类型解析"]
                  D4["Parser.\_\_LetStatement\_\_() 解析Let语句"]
                  D5["Parser.\_\_ASKStatement\_\_() 解析Ask语句"]
                  D6["Parser.\_\_printStatement\_\_() 解析Print语句"]
                  D7["Parser.\_\_TellStatement\_\_() 解析Tell语句"]
                  D8["Parser.\_\_expressionStatement\_\_() 解析表达式语句"]
        end
        
        subgraph s4_2["表达式解析层次"]
                  D9["Parser.\_\_expression\_\_() 表达式入口"]
                  D10["Parser.\_\_implicate\_\_() 蕴含表达式"]
                  D11["Parser.\_\_Or\_\_() 析取表达式"]
                  D12["Parser.\_\_And\_\_() 合取表达式"]
                  D13["Parser.\_\_equality\_\_() 等值表达式"]
                  D14["Parser.\_\_comparison\_\_() 比较表达式"]
                  D15["Parser.\_\_term\_\_() 项表达式"]
                  D16["Parser.\_\_factor\_\_() 因子表达式"]
                  D17["Parser.\_\_unary\_\_() 一元表达式"]
                  D18["Parser.\_\_predicate\_\_() 谓词表达式"]
                  D19["Parser.\_\_primary\_\_() 基本表达式"]
        end
        
        subgraph s4_3["基本表达式类型"]
                  D20["常量(FALSE/TRUE/NIL)"]
                  D21["字面量(NUMBER/STRING)"]
                  D22["变量(小写开头)"]
                  D23["常量(大写开头)"]
        end
  end
 subgraph s5["解释执行层"]
        E["Interpreter.py 解释器"]
        E1["Interpreter.interpret 执行语句"]
        E2["Interpreter.\_\_execute\_\_ 执行单个语句"]
        E3["Interpreter.visitAskStmt 处理ASK语句"]
        E4["Interpreter.visitExpressionStmt 处理表达式语句"]
        E5["Interpreter.visitPrintStmt 处理Print语句"]
        E6["Interpreter.visitTellStmt 处理Tell语句"]
  end
 subgraph s6["推理引擎层"]
        F["Inference_engine.py 推理引擎"]
        F1["FolKB.tell 添加知识"]
        F2["FolKB.ask 查询推理"]
        F3["fol_bc_ask 反向链接推理"]
        F4["unify_mm 合一算法"]
  end
 subgraph s7["基础层"]
        G["Expr.py 表达式定义"]
        H["Stmt.py 语句定义"]
        I["Tokentype.py 词法单元类型"]
  end
 subgraph s8["输出层"]
        J["output.txt 输出文件"]
  end
    A -- 读取 --> B2
    B1 -- 调用 --> B2
    B2 -- 调用 --> B3
    B3 -- 创建 --> C
    C -- 调用 --> C1
    C1 -- 遍历每个字符 --> C2
    C2 -- 返回单个字符的token --> C1
    C1 -- 返回List[Token] --> D
    D -- 调用 --> D1
    D1 -- 循环解析 --> D2
    D2 -- 调用 --> D3
    D3 -- 匹配LET --> D4
    D3 -- 匹配ASK --> D5
    D3 -- 匹配PRINT --> D6
    D3 -- 匹配TELL --> D7
    D3 -- 其他 --> D8
    D4 -- 解析表达式 --> D9
    D5 -- 解析表达式 --> D9
    D6 -- 解析表达式 --> D9
    D7 -- 解析表达式 --> D9
    D8 -- 解析表达式 --> D9
    D9 -- 调用 --> D10
    D10 -- 调用 --> D11
    D11 -- 调用 --> D12
    D12 -- 调用 --> D13
    D13 -- 调用 --> D14
    D14 -- 调用 --> D15
    D15 -- 调用 --> D16
    D16 -- 调用 --> D17
    D17 -- 调用 --> D18
    D18 -- 调用 --> D19
    D19 -- 匹配FALSE/TRUE/NIL --> D20
    D19 -- 匹配NUMBER/STRING --> D21
    D19 -- 匹配小写IDENTIFIER --> D22
    D19 -- 匹配大写IDENTIFIER --> D23
    D1 -- 返回语法树 --> E
    E -- 调用 --> E1
    E1 -- 循环调用 --> E2
    E2 -- 处理表达式 --> E4
    E2 -- 处理ASK --> E3
    E2 -- 处理PRINT --> E5
    E2 -- 处理TELL --> E6
    E4 -- 添加知识 --> F1
    E3 -- 查询推理 --> F2
    F2 -- 调用 --> F3
    F3 -- 调用 --> F4
    F1 -- 存储 --> F
    F2 -- 返回结果 --> E3
    E3 -- 写入 --> J
    E5 -- 写入 --> J
    C -- 使用 --> I
    D -- 使用 --> G & H
    E -- 使用 --> G & H
    F -- 使用 --> G
```

## 关键函数和数据流向说明

### 1. 输入流程
- **TSRL.main()**：接收输入文件路径和输出文件路径，是系统的入口点
- **TSRL.__run_file()**：读取输入文件内容，将其转换为字符串
- **TSRL.__run()**：开始处理流程，创建词法分析器

### 2. 词法分析
- **Scanner.scan_tokens()**：将输入文本转换为词法单元（tokens）
- 处理关键字、运算符、常量等，生成token列表

### 3. 语法分析
- **Parser.parse()**：将tokens转换为抽象语法树（AST）
- 支持Let、Ask、Print、Tell等语句类型
- 构建表达式和语句的层次结构

### 4. 解释执行
- **Interpreter.interpret()**：遍历执行语法树中的所有语句
- **Interpreter.__execute__()**：执行单个语句，根据语句类型调用相应的访问方法
- **Interpreter.visitExpressionStmt()**：处理表达式语句，将表达式添加到知识库
- **Interpreter.visitAskStmt()**：处理ASK语句，执行查询并返回结果

### 5. 推理引擎
- **FolKB.tell()**：将表达式添加到知识库中
- **FolKB.ask()**：执行推理查询，返回满足条件的置换
- **fol_bc_ask()**：使用反向链接算法进行推理，查找满足查询的解

### 6. 输出流程
- 推理结果写入指定的输出文件
- 支持JSON格式输出置换结果
- 对于ASK语句，将推理结果写入output.txt

### 数据流向
- **输入文件 → 词法分析 → 语法分析 → 解释执行 → 推理 → 输出文件**
- 各模块通过函数调用和数据传递形成完整的处理链
- 基础模块（Expr、Stmt、Tokentype）被其他模块广泛使用

### 核心数据结构
- **Token**：词法单元，包含类型、值、位置等信息
- **Expr**：表达式基类，包括Predicate、Implication、Binary等子类
- **Stmt**：语句基类，包括Expression、Ask、Print、Tell等子类
- **FolKB**：一阶逻辑知识库，存储知识和执行推理

## 查看方式

1. **使用支持 Mermaid 的编辑器**：如 VS Code（安装 Mermaid 插件）、GitHub、GitLab 等
2. **使用在线 Mermaid 编辑器**：访问 https://mermaid.live/，复制上述 Mermaid 代码进行渲染
3. **使用 Markdown 查看器**：许多 Markdown 查看器都支持 Mermaid 图表渲染

## 注意事项

- 图表展示了 TSRL 系统的完整数据流路径
- 各模块之间的依赖关系和数据流向清晰可见
- 关键函数和处理步骤已在图表中标注
- 此图表可帮助理解 TSRL 系统的工作原理和处理流程