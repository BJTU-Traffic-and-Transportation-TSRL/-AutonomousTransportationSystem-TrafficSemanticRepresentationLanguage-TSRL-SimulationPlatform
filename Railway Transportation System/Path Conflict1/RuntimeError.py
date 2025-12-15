
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


