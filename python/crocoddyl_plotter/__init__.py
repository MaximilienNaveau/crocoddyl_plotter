from crocoddyl_plotter.crocoddyl_pb2_grpc import CrocoddylPlotterStub
from crocoddyl_plotter.crocoddyl_pb2 import OCPd
from google.protobuf.empty_pb2 import Empty

import grpc

from .crocoddyl_plotter_pywrap import *

class Client:
    def __init__(self):
        self._channel = grpc.insecure_channel("localhost:1234")
        self._stub = CrocoddylPlotterStub(self._channel)

    def getOCPData(self):
        return self._stub.GetOCPData(Empty())