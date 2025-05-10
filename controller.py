# controller.py
import signal, sys, time
from contextlib import AbstractContextManager
from dynamixel_sdk import PortHandler, PacketHandler
from config import load_config

ADDR = { "CW_LIMIT":6, "CCW_LIMIT":8, "TORQUE":24, "LED":25, "GOAL_POS":30, "SPEED":32 }

class MotorGroup:
    def __init__(self, ids, port, pkt, invert):
        self.ids = ids
        self.pkt = pkt
        self.port = port
        self.invert = dict(zip(ids, invert))
    # ... (аналогично предыдущему)

class DXController(AbstractContextManager):
    def __init__(self):
        cfg = load_config()
        self.ids    = cfg["ids"]
        self.invert = cfg["invert"]
        self.scale  = cfg["speed_scale"]

        self.port = PortHandler(cfg["device"])
        self.pkt  = PacketHandler(cfg["protocol"])
        if not (self.port.openPort() and self.port.setBaudRate(cfg["baud"])):
            raise RuntimeError(f"Can't open {cfg['device']}@{cfg['baud']}")

        self.motors = MotorGroup(self.ids, self.port, self.pkt, self.invert)
        signal.signal(signal.SIGINT, self._on_sig)
        self.motors.set_mode(wheel=True)
    # ... остальной код как есть
