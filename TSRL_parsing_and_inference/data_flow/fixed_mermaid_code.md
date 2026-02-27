# 修正后的 Mermaid 代码

## 修正说明

原始 Mermaid 代码可能存在格式问题或特殊字符，导致在线编辑器无法正确解析。以下是修正后的代码，确保格式正确且兼容 Mermaid 编辑器。

## 修正后的 Mermaid 代码

```mermaid
flowchart TD
    subgraph 输入层
        A[input_1.txt\n输入文件]
    end

    subgraph 入口层
        B[TSRL.py\n入口文件]
        B1[TSRL.main\n主函数]
        B2[TSRL.__run_file\n读取文件]
        B3[TSRL.__run\n执行处理]
    end

    subgraph 词法分析层
        C[Scanner.py\n词法分析器]
        C1[Scanner.scan_tokens\n生成tokens]
    end

    subgraph 语法分析层
        D[Parser.py\n语法分析器]
        D1[Parser.parse\n生成语法树]
    end

    subgraph 解释执行层
        E[Interpreter.py\n解释器]
        E1[Interpreter.interpret\n执行语句]
        E2[Interpreter.__execute__\n执行单个语句]
        E3[Interpreter.visitAskStmt\n处理ASK语句]
        E4[Interpreter.visitExpressionStmt\n处理表达式语句]
    end

    subgraph 推理引擎层
        F[Inference_engine.py\n推理引擎]
        F1[FolKB.tell\n添加知识]
        F2[FolKB.ask\n查询推理]
        F3[fol_bc_ask\n反向链接推理]
    end

    subgraph 基础层
        G[Expr.py\n表达式定义]
        H[Stmt.py\n语句定义]
        I[Tokentype.py\n词法单元类型]
    end

    subgraph 输出层
        J[output.txt\n输出文件]
    end

    A -->|读取| B2
    B1 -->|调用| B2
    B2 -->|调用| B3
    B3 -->|创建| C
    C -->|调用| C1
    C1 -->|返回tokens| D
    D -->|调用| D1
    D1 -->|返回语法树| E
    E -->|调用| E1
    E1 -->|循环调用| E2
    E2 -->|处理表达式| E4
    E2 -->|处理ASK| E3
    E4 -->|添加知识| F1
    E3 -->|查询推理| F2
    F2 -->|调用| F3
    F1 -->|存储| F
    F2 -->|返回结果| E3
    E3 -->|写入| J
    C -->|使用| I
    D -->|使用| G
    D -->|使用| H
    E -->|使用| G
    E -->|使用| H
    F -->|使用| G
```

## 使用说明

1. 复制上面的 Mermaid 代码（从 ```mermaid 到 ```）
2. 打开 https://mermaid.live/
3. 在左侧编辑器中粘贴代码
4. 右侧会自动生成可视化的数据流导向图
5. 可以使用编辑器右上角的下载按钮将图表保存为 PNG 文件

## 修正的内容

1. 移除了节点名称中的双引号，使用方括号 [] 替代
2. 简化了函数名，移除了括号 ()
3. 确保缩进和格式一致
4. 移除了可能导致解析问题的特殊字符

这样应该可以解决在线 Mermaid 编辑器的解析错误问题。