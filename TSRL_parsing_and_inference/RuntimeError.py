
import sys
import os
# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import Tokentype
class CustomRuntimeError(RuntimeError) :
    def __init__(self, token, message):
        self.token = token
        self.message = message
        RuntimeError.__init__(self,message)

    # def getMessage(self):
    #     return self.message
    # def runtimeError(self):
    #     return f"RuntimeError at {self.token}: {super().__str__()}"


