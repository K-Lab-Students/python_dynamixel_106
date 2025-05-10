# controller.py
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
        # карта инверсий {id: ±1}
        self.invert = dict(zip(ids, invert))

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
        # порядок IDs: [FL, FR, RR, RL]
        speeds = {
            self.ids[0]: to_raw(vl),  # переднее левое
            self.ids[1]: to_raw(vr),  # переднее правое
            self.ids[2]: to_raw(vr),  # заднее правое
            self.ids[3]: to_raw(vl),  # заднее левое
        }
        self.motors.set_speed(speeds)
