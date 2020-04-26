#
# explorer_client.py
#
# Implements the explorer client class
#

# from explorer import ExplorerError

class ExplorerClient():
    """
    ExplorerClient

    Handles robot duties for exploring a map.
    Communicates with an ExplorerServer.
    """
    def __init__(self):
        self.initialized = False
        # TODO Get ros parameters and construct objects here

    def setup(self):
        if self.initialized:
            # raise ExplorerError("ExplorerClient already initialized!")
            pass
        # TODO finalize any setup needed
        self.initialized = True

    def loop(self):
        if not self.initialized:
            # raise ExplorerError("ExplorerClient not initialized!")
            pass
        # TODO execute main functionality here

    ###############################
    # Helper functions here
    ###############################



    ###############################
    # Callback functions here
    ###############################



