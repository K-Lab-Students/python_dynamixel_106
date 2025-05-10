#!/usr/bin/env python3
"""
controller.py — Высокоуровневый модуль управления шасси на MX-106 (wheel-mode)
Добавлен метод drive_vector для векторного (strafe) управления.
"""
import signal, sys, time
from contextlib import AbstractContextManager

from dynamixel_sdk import PortHandler, PacketHandler
from config import load_config

# адреса регистров MX-106 Protocol 1.0
ADDR = {
    "CW_LIMIT": 6,
    "CCW_LIMIT": 8,
    "TORQUE":   24,
    "LED":      25,
    "GOAL_POS": 30,
    "SPEED":    32,
}

class MotorGroup:
    def __init__(self, ids, port, pkt, invert):
        self.ids    = ids
        self.port   = port
        self.pkt    = pkt
        self.invert = dict(zip(ids, invert))  # карта инверсий {id: ±1}

    def _w1(self, mid, addr, val):
        self.pkt.write1ByteTxRx(self.port, mid, addr, val)

    def _w2(self, mid, addr, val):
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

    def set_speed(self, raw_speeds):
        for mid, raw in raw_speeds.items():
            val = self.invert[mid] * raw
            packet = (val & 0x3FF) if val >= 0 else (0x400 | ((-val) & 0x3FF))
            self._w2(mid, ADDR["SPEED"], packet)

class DXController(AbstractContextManager):
    def __init__(self):
        cfg = load_config()
        self.ids    = cfg["ids"]
        self.invert = cfg["invert"]
        self.scale  = cfg["speed_scale"]

        # открываем порт и протокол
        self.port = PortHandler(cfg["device"])
        self.pkt  = PacketHandler(cfg["protocol"])
        if not (self.port.openPort() and self.port.setBaudRate(cfg["baud"])):
            raise RuntimeError(f"Cannot open {cfg['device']} @ {cfg['baud']}")

        # инициализируем группу моторов и wheel-mode
        self.motors = MotorGroup(self.ids, self.port, self.pkt, self.invert)
        signal.signal(signal.SIGINT, self._on_sig)
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
        Дифференциальное движение:
          v     — линейная скорость (м/с)
          omega — угловая скорость (рад/с, + против часовой)
        """
        TRACK = 0.13  # ширина между левым и правым блоками, м
        half = TRACK / 2
        vl = v - omega * half
        vr = v + omega * half

        to_raw = lambda w: max(-1023, min(1023, int(w * self.scale)))
        speeds = {
            self.ids[0]: to_raw(vl),  # FL
            self.ids[1]: to_raw(vr),  # FR
            self.ids[2]: to_raw(vr),  # RR
            self.ids[3]: to_raw(vl),  # RL
        }
        self.motors.set_speed(speeds)

    def drive_vector(self, vx: float, vy: float, omega: float):
        """
        Векторное управление для mecanum-колёс:
          vx    — вперед (м/с)
          vy    — вправо (м/с)
          omega — угловая скорость (рад/с)
        """
        # параметры шасси
        r = 0.025    # радиус колеса, м
        Lx = 0.195   # межцентровое расстояние вперед/назад, м
        Ly = 0.13    # межцентровое расстояние влево/вправо, м
        L = Lx + Ly

        # расчет угловых скоростей колес
        w_fl = (vx - vy - omega * L) / r
        w_fr = (vx + vy + omega * L) / r
        w_rr = (vx - vy + omega * L) / r
        w_rl = (vx + vy - omega * L) / r

        to_raw = lambda w: max(-1023, min(1023, int(w * self.scale)))
        raw = {
            self.ids[0]: to_raw(w_fl),  # FL
            self.ids[1]: to_raw(w_fr),  # FR
            self.ids[2]: to_raw(w_rr),  # RR
            self.ids[3]: to_raw(w_rl),  # RL
        }
        self.motors.set_speed(raw)
