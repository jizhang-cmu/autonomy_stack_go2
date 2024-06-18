import threading

class ThreadPauser:
    def __init__(self):
        self.condition = threading.Condition()
        self.result = None
	
    def sleep_until_resumed(self):
        with self.condition:
            self.condition.wait()

    def resume_with_result(self, result):
        self.result = result
        with self.condition:
            self.condition.notify()
