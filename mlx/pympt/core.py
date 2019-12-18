"""Core definitions of MPT framework"""


class MptException(Exception):
    pass


class BadCrcException(MptException):
    pass


class CommandTooLongException(MptException):
    def __init__(self):
        MptException.__init__(self)
        self.message = "Command is limited to 253 bytes!"


class NotConnectedException(MptException):
    pass


class I2CAcknowledgeError(MptException):
    pass
