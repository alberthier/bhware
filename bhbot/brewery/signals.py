import logger
import sys
import traceback


class Signal:
    def __init__(self, name=None):
        self.callbacks = []
        self.errbacks = []
        self.name = "(anonymous signal)"

    def connect(self, fn):
        self.callbacks.append(fn)

    def send(self, *args, **kwargs):
        for cb in self.callbacks :
            cb(*args, **kwargs)

    def send_safe(self, *args, **kwargs):
        for cb in self.callbacks :
            try :
                cb(*args, **kwargs)
            except Exception as e :
                exc_type, exc_value, exc_traceback = sys.exc_info()
                content = "\n".join(traceback.format_exception(exc_type, exc_value,
                                          exc_traceback))
                logger.log('Exception while handling signal {} by {} : {}'.format(self.name,
                                                                                           cb.__name__, content))

    def failure(self, exc, *args, **kwargs):
        for cb in self.errbacks :
            try :
                cb(exc, *args, **kwargs)
            except Exception as e :
                exc_type, exc_value, exc_traceback = sys.exc_info()
                content = "\n".join(traceback.format_exception(exc_type, exc_value,
                                          exc_traceback))
                logger.log('Exception while handling error in signal {} by {} : {}'.format(self.name,
                                                                                           cb.__name__, content))

class SafeSignal(Signal):
    def send(self, *args, **kwargs):
        return self.send_safe(*args, **kwargs)
