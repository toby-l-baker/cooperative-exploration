#
# explorer_server.py
#
# Implements the explorer server class
#

from explorer import ExplorerError

class ExplorerServer():
    """
    ExplorerServer

    Organizes multi-robot duties for exploring a map.
    Communicates with an ExplorerClient.
    """
    def __init__(self):
        self.initialized = False
        # TODO Get ros parameters and construct objects here

    def setup(self):
        if self.initialized:
            raise ExplorerError("ExplorerServer already initialized!")
        # TODO finalize any setup needed
        self.initialized = True

    def loop(self):
        if not self.initialized:
            raise ExplorerError("ExplorerServer not initialized!")
        # TODO execute main functionality here

    ###############################
    # Helper functions here
    ###############################



    ###############################
    # Callback functions here
    ###############################



