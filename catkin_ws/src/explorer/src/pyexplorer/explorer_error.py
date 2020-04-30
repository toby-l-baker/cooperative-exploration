#
# explorer_error.py
#

class ExplorerError(Exception):
    def __init__(self, message):
        self.message = message
