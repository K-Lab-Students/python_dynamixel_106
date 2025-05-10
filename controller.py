"""
controller.py — Высокоуровневый модуль управления шасси на MX-106 (wheel-mode).
"""

import signal, sys, time
from contextlib import AbstractContextManager
from typing import Dict, List

import yaml
from dynamixel_sdk import PortHandler, PacketHandler

ADDR = {
    "CW_LIMIT": 6,
    "CCW_LIMIT": 8,
    "TORQUE":   24,
    "LED":      25,
    "GOAL_POS": 30,
    "SPEED":    32,
}


def load_config(path: str = "config.yaml") -> Dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


class MotorGroup:
    def __init__(self, ids: List[int], port: PortHandler, pkt: PacketHandler, invert: List[int]):
        self.ids = ids
        self.port = port
        self.pkt  = pkt
        self.invert = dict(zip(ids, invert))

    def _w1(self, mid: int, addr: int, val: int):
        self.pkt.write1ByteTxRx(self.port, mid, addr, val)

    def _w2(self, mid: int, addr: int, val: int):
        self.pkt.write2ByteTxRx(self.port, mid, addr, val)

    def torque(self, enable: bool):
        for i in self.ids:
            self._w1(i, ADDR["TORQUE"], int(enable))

    def set_mode(self, wheel: bool = True):
        cw, ccw = (0, 0) if wheel else (0, 4095)
        for i in self.ids:
            self._w1(i, ADDR["TORQUE"], 0)
            self._w2(i, ADDR["CW_LIMIT"], cw)
            self._w2(i, ADDR["CCW_LIMIT"], ccw)
            self._w1(i, ADDR["TORQUE"], 1)

    def set_speed(self, raw_speeds: Dict[int, int]):
        for mid, raw in raw_speeds.items():
            val = self.invert[mid] * raw
            if val >= 0:
                packet = val & 0x3FF
            else:
                packet = 0x400 | ((-val) & 0x3FF)
            self._w2(mid, ADDR["SPEED"], packet)


class DXController(AbstractContextManager):
    def __init__(self, config_path: str = "config.yaml"):
        cfg = load_config(config_path)
        self.ids    = cfg["ids"]
        self.invert = cfg["invert"]
        self.scale  = cfg["speed_scale"]

        self.port = PortHandler(cfg["device"])
        self.pkt  = PacketHandler(cfg["protocol"])
        if not (self.port.openPort() and self.port.setBaudRate(cfg["baud"])):
            raise RuntimeError(f"Cannot open {cfg['device']} @ {cfg['baud']}")

        self.motors = MotorGroup(self.ids, self.port, self.pkt, self.invert)
        signal.signal(signal.SIGINT, self._on_sig)

        # сразу в wheel-mode
        self.motors.set_mode(wheel=True)

    def _on_sig(self, *args):
        self.motors.torque(False)
        self.port.closePort()
        print("\n⏹ Torque OFF, port closed")
        sys.exit(0)

    def __exit__(self, exc_type, exc, tb):
        self.motors.torque(False)
        self.port.closePort()

    def drive(self, v: float, omega: float):
        """
        Простейшее дифф-подобное управление:
        v     — линейная скорость (м/с)
        omega — угловая скорость (рад/с)
        """
        TRACK_WIDTH = 0.13
        half = TRACK_WIDTH / 2
        v_l = v - omega * half
        v_r = v + omega * half

        to_raw = lambda w: max(-1023, min(1023, int(w * self.scale)))
        speeds = {self.ids[i]: to_raw(val)
                  for i, val in enumerate([v_l, v_r, v_r, v_l])}

        self.motors.set_speed(speeds)
