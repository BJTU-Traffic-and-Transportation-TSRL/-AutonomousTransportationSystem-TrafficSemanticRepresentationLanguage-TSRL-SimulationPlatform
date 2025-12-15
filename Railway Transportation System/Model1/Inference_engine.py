import itertools
import random

from Expr import Expr,Predicate,Variable,Constant
from Tokentype import Token

#工具函数
def first(iterable, default=None):
    """Return the first element of an iterable; or default."""
    return next(iter(iterable), default)

#扩展字典集合
def extend(s, var, val):
    """Copy dict s and extend it by setting var to val; return copy."""
    return {**s, var: val}

#生成长整数
class UniqueEightDigitGenerator:
    def __init__(self):
        self.generated = set()
        # 计算总共可能的组合数：9 * 10^7
        self.total_possible = 90000000

    def generate(self):
        """生成一个不重复的8位随机整数"""
        if len(self.generated) >= self.total_possible:
            raise RuntimeError("已生成所有可能的8位整数")

        while True:
            # 生成10000000到99999999之间的随机整数
            num = random.randint(10000000, 99999999)

            if num not in self.generated:
                self.generated.add(num)
                return num


class KB:

    def __init__(self, sentence=None):
        if sentence:
            self.tell(sentence)

    def tell(self, sentence):
        """Add the sentence to the KB."""
        raise NotImplementedError

    def ask(self, query):
        """Return a substitution that makes the query true, or, failing that, return False."""
        return first(self.ask_generator(query), default=False)


    def ask_generator(self, query):
        """Yield all the substitutions that make query true."""
        raise NotImplementedError

    def retract(self, sentence):
        """Remove sentence from the KB."""
        raise NotImplementedError

#工具函数
#判断符号
def is_symbol(s):
    """A string s is a symbol if it starts with an alphabetic char.
    判断输入是否是以字母开头的符号
    >>> is_symbol('R2D2')
    True
    """
    return isinstance(s, str) and s[:1].isalpha()

def is_prop_symbol(s):
    """A proposition logic symbol is an initial-uppercase string.
    判断输入是否是以大写字母开头的命题词（或常量）
    >>> is_prop_symbol('exe')
    False
    """
    return is_symbol(s) and s[0].isupper()
    #return isinstance(s,Predicate) or isinstance(s,Constant)

def is_var_symbol(s):
    """A logic variable symbol is an initial-lowercase string.
    判断输入是否是以小写字母开头的变量
    >>> is_var_symbol('EXE')
    False
    """
    return is_symbol(s) and s[0].islower()
    #return isinstance(s,Variable)

#将合取式或析取式拆分为列表
def dissociate(op, args):
    """Given an associative op, return a flattened list result such
    that Expr(op, *result) means the same as Expr(op, *args).
    >>> dissociate('&', [Expr('&',Expr("A"),Expr("B"))])
    [A, B]
    """
    result = []

    def collect(subargs):
        for arg in subargs:
            if arg.op == op:
                collect(arg.args)
            else:
                result.append(arg)

    collect(args)
    return result
#将合取式拆分为列表
def conjuncts(s):
    """
    输入合取式，输出合取式的各子式的列表.
    """
    return dissociate('&', [s])

#判断是否为限定子句
def is_definite_clause(s):
    """
    判断是否为限定子句，若是，则返回True
    """
    if is_symbol(s.op):
        return True
    elif s.op == '==>' or s.op == ':-':
        antecedent, consequent = s.args
        return is_symbol(consequent.op) and all(is_symbol(arg.op) for arg in conjuncts(antecedent))
    else:
        return False

def parse_definite_clause(s):
    """解析限定子句，输出为体和头的各子式的列表"""
    assert is_definite_clause(s)
    if is_symbol(s.op):
        return [], s
    else:
        antecedent, consequent = s.args
        return conjuncts(antecedent), consequent

#输入子句，输出集合，集合中包含子句中的所有常量（非谓词）
def constant_symbols(x):
    """Return the set of all constant symbols in x."""
    if not isinstance(x, Expr):
        return set()
    elif is_prop_symbol(x.op) and not x.args:
        return {x}
    else:
        return {symbol for arg in x.args for symbol in constant_symbols(arg)}

#生成子表达式（包括它自身）
def subexpressions(x):
    """Yield the subexpressions of an Expression (including x itself)."""
    yield x
    if isinstance(x, Expr):
        for arg in x.args:
            yield from subexpressions(arg)
#判断表达式是否为变量
def is_variable(x):
    """A variable is an Expr with no args and a lowercase symbol as the op."""
    # return isinstance(x, Expr) and not x.args and x.op[0].islower()
    return isinstance(x, Variable)

#输入子句，输出集合，集合中包含子句中的所有变量
def variables(s):
    """Return a set of the variables in expression s.
    >>> variables(Expr('==>',Expr('&',Expr('Rabbit',Expr('r')),Expr('Farmer',Expr('f'))),Expr('Hates',Expr('f'),Expr('r')))) == {Expr('r'),Expr('f')}
    True
    """
    return {x for x in subexpressions(s) if is_variable(x)}

#判断表达式是否为常量
def is_constant(x):
    return isinstance(x, Constant)

def vars_elimination(x, s):
    """Apply variable elimination to x: if x is a variable and occurs in s, return
    the term mapped by x, else if x is a function recursively applies variable
    elimination to each term of the function.
    对x应用变量消除：如果x是一个变量且出现在s中，则返回由x映射的项；否则，如果x是一个函数，则对该函数的每个项递归地应用变量消除。"""
    if not isinstance(x, Expr):
        return x
    if is_variable(x):
        return s.get(x, x)
    return Expr(x.op, *[vars_elimination(arg, s) for arg in x.args])

#变量标准化
def standardize_variables(sentence:Expr, dic=None):
    """Replace all the variables in sentence with new variables.
    >>> from Tokentype import TokenType
    >>> standardize_variables(Predicate('male',None,Constant('Di',Token(TokenType.IDENTIFIER,'Di',None,0))))
    male(Di)
    """
    if dic is None:
        dic = {}
    if not isinstance(sentence, Expr):
        return sentence
    # elif is_var_symbol(sentence.op):
    elif is_variable(sentence):
        if sentence in dic:
            return dic[sentence]
        else:
            # v = Variable('v_{}'.format(next(standardize_variables.counter)),
            #              Token(sentence.token.type,'v_{}'.format(next(standardize_variables.counter)),sentence.token.literal,sentence.token.line))
            v_iden = generator.generate()
            v = Variable('v_{}'.format(v_iden),
                         Token(sentence.token.type,'v_{}'.format(v_iden),sentence.token.literal,sentence.token.line))
            dic[sentence] = v
            return v
    elif is_constant(sentence):
        return sentence
    else:
        return Predicate(sentence.op, sentence.token, *[standardize_variables(a, dic) for a in sentence.args])

# standardize_variables.counter = itertools.count()
generator = UniqueEightDigitGenerator()

def term_reduction(x, y, s):
    """Apply term reduction to x and y if both are functions and the two root function
    symbols are equals (e.g. F(x1, x2, ..., xn) and F(x1', x2', ..., xn')) by returning
    a new mapping obtained by replacing x: y with {x1: x1', x2: x2', ..., xn: xn'}
    如果 x 和 y 均为函数且两个根函数符号相等（例如 F(x1, x2, ..., xn) 和 F(x1', x2', ..., xn')），
    则对 x 和 y 应用项缩减，通过将 x: y 替换为 {x1: x1', x2: x2', ..., xn: xn'} 来返回一个新的映射。
    """
    for i in range(len(x.args)):
        if x.args[i] in s:
            s[s.get(x.args[i])] = y.args[i]
        else:
            s[x.args[i]] = y.args[i]

def occur_check(var, x, s):
    """Return true if variable var occurs anywhere in x
    (or in subst(s, x), if s has a binding for x).
    如果变量 var 出现在 x 的任何位置（或者如果 s 中有对 x 的绑定，则出现在对 x 应用替换 s 后的结果中），则返回 true。"""
    if var == x:
        return True
    elif is_variable(x) and x in s:
        return occur_check(var, s[x], s)
    elif isinstance(x, Expr):
        return (occur_check(var, x.op, s) or
                occur_check(var, x.args, s))
    elif isinstance(x, (list, tuple)):
        return first(e for e in x if occur_check(var, e, s))
    else:
        return False

def subst(s, x):
    """置换替换生成新子句.
    >>> subst({Expr("x"): 42, Expr("y"):0}, Expr("+",Expr('F',Expr('x')),Expr('y')))
    (F(42) + 0)
    """
    if isinstance(x, list):
        return [subst(s, xi) for xi in x]
    elif isinstance(x, tuple):
        return tuple([subst(s, xi) for xi in x])
    elif not isinstance(x, Expr):
        return x
    elif is_var_symbol(x.op):
        return s.get(x, x)
    else:
        return Expr(x.op, None, *[subst(s, arg) for arg in x.args])

#合一算法，生成置换字典
def unify_mm(x, y, s={}):
    """
    合一算法，生成置换字典      注意is表达式
    Unify expressions x,y with substitution s using an efficient rule-based
    unification algorithm by Martelli & Montanari; return a substitution that
    would make x,y equal, or None if x,y can not unify. x and y can be
    variables (e.g. Expr('x')), constants, lists, or Exprs.
    >>> unify_mm(Expr("x"), 3, {})
    {x: 3}
    """

    set_eq = extend(s, x, y)
    s = set_eq.copy()
    while True:
        trans = 0
        for x, y in set_eq.items():
            if x == y:
                # 如果 x = y 删除这个映射（rule b）
                del s[x]
            elif not is_variable(x) and is_variable(y):
                # 如果 x不是变量，y是变量，在s中重写为 y=x（rule a）
                if s.get(y, None) is None:
                    s[y] = x
                    del s[x]
                else:
                    # 如果在s中存在y的某个映射，则应用变量消去vars_elimination（有可能应用规则 d）
                    s[x] = vars_elimination(y, s)
            elif not is_variable(x) and not is_variable(y):
                #在这种情况下，x 和 y 不是变量，如果两个根函数符号不同(谓词名或参数数量不同)，则以失败告终，否则应用 项约简term_reduction（规则 c）
                if x.op == y.op and len(x.args) == len(y.args):
                    term_reduction(x, y, s)
                    del s[x]
                else:
                    return None
            elif isinstance(y, Expr):
                # 在这种情况下，x 是一个变量，而 y 是一个函数或变量（例如 F(z) 或 y），
                # 如果 y 是一个函数，我们必须检查 x 是否出现在 y 中，如果出现则停止并失败，
                # 否则尝试对 y 应用变量消去variable elimination（规则 d）。
                if occur_check(x, y, s):
                    return None
                s[x] = vars_elimination(y, s)
                if y == s.get(x):
                    trans += 1
            else:
                trans += 1
        if trans == len(set_eq):
            # if no transformation has been applied, stop with success
            return s
        set_eq = s.copy()


class FolKB(KB):
    """A knowledge base consisting of first-order definite clauses.
    >>> kb0 = FolKB([Expr('Farmer',Expr('Mac')),Expr('Rabbit',Expr('Pete')),
    ...             Expr('==>',Expr('&',Expr('Rabbit',Expr('r')),Expr('Farmer',Expr('f'))),Expr('Hates',Expr('f'),Expr('r')))])
    >>> kb0.tell(Expr('Rabbit',Expr('Flopsie')))
    >>> kb0.retract(Expr('Rabbit',Expr('Pete')))
    >>> kb0.ask(Expr('Hates',Expr('Mac'),Expr('x')))[Expr('x')]
    Flopsie
    >>> kb0.ask(Expr('Wife',Expr('Pete'),Expr('x')))
    False
    """

    def __init__(self, clauses=None):
        super().__init__()
        self.clauses = []  # inefficient: no indexing
        if clauses:
            for clause in clauses:
                self.tell(clause)

    #只接受一阶确定子句
    def tell(self, sentence):
        if is_definite_clause(sentence):
            self.clauses.append(sentence)
        else:
            raise Exception('Not a definite clause: {}'.format(sentence))

    def ask_generator(self, query):
        return fol_bc_ask(self, query)

    def retract(self, sentence):
        self.clauses.remove(sentence)

    def fetch_rules_for_goal(self, goal):
        return self.clauses

#前向链接
def fol_fc_ask(kb, alpha):
    """
    [Figure 9.3]
    A simple forward-chaining algorithm.
    """
    # TODO: improve efficiency
    kb_consts = list({c for clause in kb.clauses for c in constant_symbols(clause)}) # 返回不重复的常量列表

    def enum_subst(p):
        """
        枚举所有可能的置换
        """
        query_vars = list({v for clause in p for v in variables(clause)}) # 返回不重复的变量列表
        for assignment_list in itertools.product(kb_consts, repeat=len(query_vars)): #产生常量列表依变量个数的笛卡尔积列表
            theta = {x: y for x, y in zip(query_vars, assignment_list)} #生成所有可能的置换
            yield theta

    # 检查我们是否不推理就能得到回答
    for q in kb.clauses:
        phi = unify_mm(q, alpha)
        if phi is not None:
            yield phi

    while True:
        new = []
        for rule in kb.clauses:
            p, q = parse_definite_clause(rule) #p为前提，q为结论
            for theta in enum_subst(p):
                if set(subst(theta, p)).issubset(set(kb.clauses)):
                    q_ = subst(theta, q)
                    if all([unify_mm(x, q_) is None for x in kb.clauses + new]):
                        new.append(q_)
                        phi = unify_mm(q_, alpha)
                        if phi is not None:
                            yield phi
        if not new:
            break
        for clause in new:
            kb.tell(clause)
    return None


#反向链接
def fol_bc_ask(kb, query):
    """
    [Figure 9.6]
    A simple backward-chaining algorithm for first-order logic.
    KB should be an instance of FolKB, and query an atomic sentence.
    """
    return fol_bc_or(kb, query, {})

#或搜索
def fol_bc_or(kb, goal, theta):
    for rule in kb.fetch_rules_for_goal(goal):
        lhs, rhs = parse_definite_clause(standardize_variables(rule))
        for theta1 in fol_bc_and(kb, lhs, unify_mm(rhs, goal, theta)):
            yield theta1

#与搜索
def fol_bc_and(kb, goals, theta):
    if theta is None:
        pass
    elif not goals:
        yield theta
    else:
        first, rest = goals[0], goals[1:]
        for theta1 in fol_bc_or(kb, subst(theta, first), theta):
            for theta2 in fol_bc_and(kb, rest, theta1):
                yield theta2


# kb = FolKB([Expr('Male',Expr('Di')),Expr('Male',Expr('Jianbo')),
#              Expr('Female',Expr('Xin')),Expr('Female',Expr('Yuan')),Expr('Female',Expr('YuQing')),
#              Expr('Father',Expr('Jianbo'),Expr('Di')),Expr('Father',Expr('Di'),Expr('YuQing')),
#              Expr('Mother',Expr('Xin'),Expr('Di')),Expr('Mother',Expr('Yuan'),Expr('YuQing'))])
#
# kb.tell(Expr('==>',Expr('&',Expr('Father',Expr('x'),Expr('z')),Expr('Father',Expr('z'),Expr('y'))),
#              Expr('Grandfather',Expr('x'),Expr('z'))))
# kb.tell(Expr('==>',Expr('&',Expr('Mather',Expr('x'),Expr('z')),Expr('Father',Expr('z'),Expr('y'))),
#              Expr('Grandmother',Expr('x'),Expr('z'))))
# kb.tell(Expr('==>',Expr('&',Expr('Father',Expr('x'),Expr('z')),Expr('Female',Expr('z'))),
#              Expr('Daughter',Expr('z'),Expr('x'))))
#
# print(kb.ask(Expr('Daughter',Expr('YuQing'),Expr('x')))[Expr('x')])

