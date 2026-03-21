# `ASK Congestion(y);` 调用链顺序图

下面的顺序图描述了在知识库已包含以下事实与规则时：

```txt
IsJunction(2);
StopAt(0,2);
Congestion(y):-StopAt(x,y),IsJunction(y);
ASK Congestion(y);
```

执行 `ASK Congestion(y);` 时，`Interpreter.visitAskStmt` 及其下游方法之间的主要数据传递过程。

```mermaid
sequenceDiagram
    autonumber
    participant U as 输入/AST
    participant I as Interpreter
    participant P as visitPredicateExpr
    participant KB as FolKB
    participant BCA as fol_bc_ask
    participant OR1 as fol_bc_or
    participant SV as standardize_variables
    participant PDC as parse_definite_clause
    participant UM as unify_mm
    participant AND1 as fol_bc_and
    participant OR2 as fol_bc_or(StopAt)
    participant AND2 as fol_bc_and(rest)
    participant OR3 as fol_bc_or(IsJunction)

    Note over U: stmt = Ask(expression = Congestion(y))

    U->>I: __execute__(stmt)
    I->>I: visitAskStmt(stmt)
    I->>I: __evaluate__(stmt.expression)
    I->>P: visitPredicateExpr(Congestion(y))
    P-->>I: Congestion(y)

    I->>KB: ask(Congestion(y))
    KB->>KB: ask_generator(Congestion(y))
    KB->>BCA: fol_bc_ask(kb, Congestion(y))
    BCA->>OR1: fol_bc_or(kb, goal=Congestion(y), theta={})

    Note over OR1: 遍历知识库子句\n1. IsJunction(2)\n2. StopAt(0,2)\n3. Congestion(y):-StopAt(x,y),IsJunction(y)

    OR1->>SV: standardize_variables(rule3)
    SV-->>OR1: Congestion(v_y):-StopAt(v_x,v_y),IsJunction(v_y)

    OR1->>PDC: parse_definite_clause(标准化后的 rule3)
    PDC-->>OR1: lhs=[StopAt(v_x,v_y), IsJunction(v_y)]\nrhs=Congestion(v_y)

    OR1->>UM: unify_mm(Congestion(v_y), Congestion(y), {})
    UM-->>OR1: {v_y: y}

    OR1->>AND1: fol_bc_and(goals=[StopAt(v_x,v_y), IsJunction(v_y)], theta={v_y:y})

    AND1->>AND1: subst(theta, StopAt(v_x,v_y))
    AND1-->>OR2: fol_bc_or(kb, goal=StopAt(v_x,y), theta={v_y:y})

    Note over OR2: 匹配到事实 StopAt(0,2)

    OR2->>PDC: parse_definite_clause(StopAt(0,2))
    PDC-->>OR2: lhs=[]\nrhs=StopAt(0,2)

    OR2->>UM: unify_mm(StopAt(0,2), StopAt(v_x,y), {v_y:y})
    UM-->>OR2: {v_y:2, v_x:0, y:2}
    OR2-->>AND1: theta1 = {v_y:2, v_x:0, y:2}

    AND1->>AND2: fol_bc_and(goals=[IsJunction(v_y)], theta={v_y:2, v_x:0, y:2})
    AND2->>AND2: subst(theta, IsJunction(v_y))
    AND2-->>OR3: fol_bc_or(kb, goal=IsJunction(2), theta={v_y:2, v_x:0, y:2})

    Note over OR3: 匹配到事实 IsJunction(2)

    OR3->>PDC: parse_definite_clause(IsJunction(2))
    PDC-->>OR3: lhs=[]\nrhs=IsJunction(2)

    OR3->>UM: unify_mm(IsJunction(2), IsJunction(2), {v_y:2, v_x:0, y:2})
    UM-->>OR3: {v_y:2, v_x:0, y:2}
    OR3-->>AND2: theta2 = {v_y:2, v_x:0, y:2}
    AND2-->>AND1: yield {v_y:2, v_x:0, y:2}
    AND1-->>OR1: yield {v_y:2, v_x:0, y:2}
    OR1-->>BCA: yield {v_y:2, v_x:0, y:2}
    BCA-->>KB: generator yields first answer
    KB-->>I: Dict = {v_y:2, v_x:0, y:2}

    Note over I: 过滤内部标准化变量 v_y / v_x\n保留用户查询变量

    I->>I: d = {"y": "2"}
    I->>I: output_file.write(json.dumps(d))
    I->>I: print({'y': '2'})
    I-->>U: return None
```

## 结果说明

- 推理器内部拿到的第一个置换结果是：`{v_y:2, v_x:0, y:2}`
- `visitAskStmt` 过滤内部变量后写出的结果是：`{"y": "2"}`
- `visitAskStmt` 方法自身没有显式 `return`，因此返回值为 `None`
