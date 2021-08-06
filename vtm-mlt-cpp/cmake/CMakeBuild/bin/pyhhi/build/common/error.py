

class BaseError(Exception):
    """Base exception with a hint whether to list traceback information or not.

    Attributes:
        msg -- explanation of the error
        list_traceback -- hint to an application level exception handler whether to list traceback information or not.
    """

    def __init__(self, msg, list_traceback=True):
        self.msg = msg
        self.list_traceback = list_traceback

    def __str__(self):
        return self.msg


class InvalidInputParameterError(BaseError):
    """Exception raised for invalid input parameters."""

    def __init__(self, msg, list_traceback=False):
        BaseError.__init__(self, msg, list_traceback)


class InvalidCommandLineArgumentError(BaseError):
    """Exception raised for invalid command line arguments."""

    def __init__(self, msg, list_traceback=False):
        BaseError.__init__(self, msg, list_traceback)
