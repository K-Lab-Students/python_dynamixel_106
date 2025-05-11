#!/usr/bin/env python3
"""
controller.py — Высокоуровневый модуль управления шасси на MX-106 (wheel- и vector-mode)
Добавлены методы для прямого управления: вперед/назад, поворот, страйф.
Теперь с логированием raw-скоростей и событий.
"""
import signal
import sys
import time
import logging
from contextlib import AbstractContextManager

from dynamixel_sdk import PortHandler, PacketHandler
from config import load_config

# Настройка логирования для модуля
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)

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
        self.invert = dict(zip(ids, invert))  # {id: ±1}

    def _w1(self, mid, addr, val):
        self.pkt.write1ByteTxRx(self.port, mid, addr, val)

    def _w2(self, mid, addr, val):
        self.pkt.write2ByteTxRx(self.port, mid, addr, val)

    def torque(self, enable: bool):
        for i in self.ids:
            self._w1(i, ADDR["TORQUE"], int(enable))
        logger.info(f"Torque {'ON' if enable else 'OFF'} for motors {self.ids}")

    def set_mode(self, wheel: bool = True):
        cw, ccw = (0, 0) if wheel else (0, 4095)
        for i in self.ids:
            self._w1(i, ADDR["TORQUE"], 0)
            self._w2(i, ADDR["CW_LIMIT"], cw)
            self._w2(i, ADDR["CCW_LIMIT"], ccw)
            self._w1(i, ADDR["TORQUE"], 1)
        mode = 'wheel' if wheel else 'joint'
        logger.info(f"Set mode {mode} for motors {self.ids}")

    def set_speed(self, raw_speeds):
        for mid, raw in raw_speeds.items():
            val = self.invert.get(mid, 1) * raw
            packet = (val & 0x3FF) if val >= 0 else (0x400 | ((-val) & 0x3FF))
            self._w2(mid, ADDR["SPEED"], packet)
        logger.info(f"Set speeds (raw) -> {raw_speeds}")

# Расположение колёс (вид сверху):
#   Front ↑
#   FL(ID2)      FR(ID7)
#    X-конфигурация роликов 45°
#   RL(ID9)      RR(ID8)
class DXController(AbstractContextManager):
    """
    Управление: drive, drive_vector, а также удобные команды:
    forward, backward, turn_left, turn_right, strafe_left, strafe_right, stop
    С логированием всех raw-значений и ключевых событий.
    """
    # геометрия шасси
    R_WHEEL = 0.025   # м
    L_X     = 0.195   # межцентровая длина
    L_Y     = 0.13    # межцентровая ширина

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
        logger.info(f"Opened port {cfg['device']} @ {cfg['baud']}")

        # инициализируем моторную группу, wheel-mode по умолчанию
        self.motors = MotorGroup(self.ids, self.port, self.pkt, self.invert)
        signal.signal(signal.SIGINT, self._on_sig)
        self.motors.set_mode(wheel=True)

    def _on_sig(self, *args):
        logger.info("SIGINT received, shutting down robot")
        self.stop()
        self.shutdown()
        sys.exit(0)

    def __exit__(self, exc_type, exc, tb):
        # On context exit, stop motion and shutdown resources
        self.stop()
        self.shutdown()

    def stop(self):
        """Stop all motion (zero speeds) without disabling torque or closing port."""
        zero = {mid: 0 for mid in self.ids}
        self.motors.set_speed(zero)
        logger.info("Robot stopped")

    def shutdown(self):
        """Disable torque and close the port."""
        # ensure motors are stopped before disabling torque
        zero = {mid: 0 for mid in self.ids}
        self.motors.set_speed(zero)
        self.motors.torque(False)
        self.port.closePort()
        logger.info("Torque OFF and port closed")

    def to_raw(self, w: float) -> int:
        """Перевод м/с или рад/с в raw ±1023"""
        return max(-1023, min(1023, int(w * self.scale)))

    def drive(self, v: float, omega: float):
        """
        Дифф-упление (две стороны): вперед/назад + поворот
        """
        half = self.L_Y / 2
        v_l = v - omega * half
        v_r = v + omega * half
        raw = {
            self.ids[0]: self.to_raw(v_l),  # FL
            self.ids[1]: self.to_raw(v_r),  # FR
            self.ids[2]: self.to_raw(v_r),  # RR
            self.ids[3]: self.to_raw(v_l),  # RL
        }
        logger.info(f"drive -> v={v}, omega={omega}, raw={raw}")
        self.motors.set_speed(raw)

    def drive_vector(self, vx: float, vy: float, omega: float):
        """
        Векторное управление для Mecanum (X-конфигурация)
        """
        L = self.L_X + self.L_Y
        w_fl = ( vx - vy - omega * L) / self.R_WHEEL
        w_fr = ( vx + vy + omega * L) / self.R_WHEEL
        w_rr = ( vx - vy + omega * L) / self.R_WHEEL
        w_rl = ( vx + vy - omega * L) / self.R_WHEEL
        raw = {
            self.ids[0]: self.to_raw(w_fl),
            self.ids[1]: self.to_raw(w_fr),
            self.ids[2]: self.to_raw(w_rr),
            self.ids[3]: self.to_raw(w_rl),
        }
        logger.info(f"drive_vector -> vx={vx}, vy={vy}, omega={omega}, raw={raw}")
        self.motors.set_speed(raw)

    # удобные методы
    def forward(self, speed: float):
        """Езда вперед"""
        logger.info(f"Command: forward speed={speed}")
        self.drive(v=speed, omega=0.0)

    def backward(self, speed: float):
        """Езда назад"""
        logger.info(f"Command: backward speed={speed}")
        self.drive(v=-speed, omega=0.0)

    def turn_left(self, omega: float):
        """Поворот против часовой"""
        logger.info(f"Command: turn_left omega={omega}")
        self.drive(v=0.0, omega=omega)

    def turn_right(self, omega: float):
        """Поворот по часовой"""
        logger.info(f"Command: turn_right omega={omega}")
        self.drive(v=0.0, omega=-omega)

    def strafe_right(self, speed: float):
        """Страйф вправо"""
        logger.info(f"Command: strafe_right speed={speed}")
        self.drive_vector(vx=0.0, vy=speed, omega=0.0)

    def strafe_left(self, speed: float):
        """Страйф влево"""
        logger.info(f"Command: strafe_left speed={speed}")
        self.drive_vector(vx=0.0, vy=-speed, omega=0.0)
