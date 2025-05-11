#!/usr/bin/env python3
"""
controller.py — Высокоуровневый модуль управления шасси на MX-106 с точными логами
В логах теперь показывается:
  - задаваемое v, omega
  - рассчитанный raw для каждой пары
  - эквивалентная линейная скорость на колесе v_m_s = raw/speed_scale
"""
import signal
import sys
import time
import logging
from contextlib import AbstractContextManager

from dynamixel_sdk import PortHandler, PacketHandler
from config import load_config

# Logging setup
dlogging = logging.getLogger("dynamixel")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

# регистры MX-106 Protocol 1.0
ADDR = {"CW_LIMIT":6, "CCW_LIMIT":8, "TORQUE":24, "LED":25, "GOAL_POS":30, "SPEED":32}

class MotorGroup:
    def __init__(self, ids, port, pkt, invert):
        self.ids = ids
        self.port = port
        self.pkt = pkt
        self.invert = dict(zip(ids, invert))

    def _w1(self, mid, addr, val):
        self.pkt.write1ByteTxRx(self.port, mid, addr, val)

    def _w2(self, mid, addr, val):
        self.pkt.write2ByteTxRx(self.port, mid, addr, val)

    def torque(self, enable: bool):
        for i in self.ids:
            self._w1(i, ADDR["TORQUE"], int(enable))
        dlogging.info(f"Torque {'ON' if enable else 'OFF'} for IDs={self.ids}")

    def set_mode(self, wheel: bool = True):
        cw, ccw = (0,0) if wheel else (0,4095)
        for i in self.ids:
            self._w1(i, ADDR["TORQUE"], 0)
            self._w2(i, ADDR["CW_LIMIT"], cw)
            self._w2(i, ADDR["CCW_LIMIT"], ccw)
            self._w1(i, ADDR["TORQUE"], 1)
        dlogging.info(f"Mode set to {'wheel' if wheel else 'joint'} for IDs={self.ids}")

    def set_speed(self, raw_speeds):
        for mid, raw in raw_speeds.items():
            val = self.invert[mid] * raw
            packet = (val & 0x3FF) if val >= 0 else (0x400 | ((-val)&0x3FF))
            self._w2(mid, ADDR["SPEED"], packet)
        dlogging.info(f"Packet SPEED -> {raw_speeds}")

class DXController(AbstractContextManager):
    R_WHEEL = 0.025
    L_X = 0.195
    L_Y = 0.13

    def __init__(self):
        cfg = load_config()
        self.ids = cfg['ids']
        self.invert = cfg['invert']
        self.scale = cfg['speed_scale']

        self.port = PortHandler(cfg['device'])
        self.pkt = PacketHandler(cfg['protocol'])
        if not (self.port.openPort() and self.port.setBaudRate(cfg['baud'])):
            raise RuntimeError(f"Cannot open {cfg['device']} @ {cfg['baud']}")
        dlogging.info(f"Opened {cfg['device']} @ {cfg['baud']}")

        self.motors = MotorGroup(self.ids, self.port, self.pkt, self.invert)
        signal.signal(signal.SIGINT, self._on_sig)
        self.motors.set_mode(wheel=True)

    def _on_sig(self, *args):
        dlogging.info("SIGINT received, stopping")
        self.stop()
        sys.exit(0)

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        self.port.closePort()
        dlogging.info("Port closed")

    def stop(self):
        zero = {mid:0 for mid in self.ids}
        self.motors.set_speed(zero)
        self.motors.torque(False)
        dlogging.info("Motors stopped and torque off")

    def to_raw(self, w: float) -> int:
        return max(-1023, min(1023, int(w*self.scale)))

    def drive(self, v: float, omega: float):
        half = self.L_Y/2
        v_l = v - omega*half
        v_r = v + omega*half
        raw = {self.ids[0]:self.to_raw(v_l),
               self.ids[1]:self.to_raw(v_r),
               self.ids[2]:self.to_raw(v_r),
               self.ids[3]:self.to_raw(v_l)}
        # логируем и raw и эквивалент m/s
        speeds_m_s = {mid: raw[mid]/self.scale for mid in raw}
                # логируем команду и рассчитанную физическую скорость
        # cmd_v — запрошенное значение, v_m_savg — средняя фактическая линейная скорость
        v_m_savg = (speeds_m_s[self.ids[0]] + speeds_m_s[self.ids[1]] + speeds_m_s[self.ids[2]] + speeds_m_s[self.ids[3]]) / 4
        dlogging.info(f"drive cmd_v={v} (unit), omega={omega} rad/s -> raw={raw}, v_phys_avg={v_m_savg:.3f} m/s, v_m_s={speeds_m_s}")


    def drive_vector(self, vx: float, vy: float, omega: float):
        """
        Векторное управление для Mecanum (X-конфигурация)
        vx    — вперед (м/с), vy — вправо (м/с), omega — угловая скорость (рад/с)
        """
        # Расчёт геометрической суммы половин баз
        L = self.L_X + self.L_Y

        # Угловые скорости каждого колеса (в рад/с):
        w_fl = ( vx - vy - omega * L) / self.R_WHEEL
        w_fr = ( vx + vy + omega * L) / self.R_WHEEL
        w_rr = ( vx - vy + omega * L) / self.R_WHEEL
        w_rl = ( vx + vy - omega * L) / self.R_WHEEL

        # Перевод в «сырые» единицы
        raw = {
            self.ids[0]: self.to_raw(w_fl),  # FL
            self.ids[1]: self.to_raw(w_fr),  # FR
            self.ids[2]: self.to_raw(w_rr),  # RR
            self.ids[3]: self.to_raw(w_rl),  # RL
        }

        # Логирование для отладки
        speeds_m_s = {mid: raw[mid] / self.scale for mid in raw}
        v_m_savg = sum(speeds_m_s.values()) / 4
        dlogging.info(
            f"drive_vector vx={vx}, vy={vy}, omega={omega} "
            f"-> raw={raw}, v_phys_avg={v_m_savg:.3f} m/s, v_m_s={speeds_m_s}"
        )

        # Отправка на моторы
        self.motors.set_speed(raw)

    def _linear(self, speed: float):
        """Helper: set linear speed for all wheels."""
        raw = {mid: self.to_raw(speed) for mid in self.ids}
        self.motors.set_speed(raw)

    def _side_speed(self, side_ids: list, speed: float):
        """Helper: set speed for specified side wheels only, others zero."""
        raw = {mid: self.to_raw(speed) if mid in side_ids else 0 for mid in self.ids}
        self.motors.set_speed(raw)

    def forward(self, speed: float):
        """Езда вперед: set same speed on all wheels."""
        dlogging.info(f"Command: forward speed={speed}")
        # move forward on all wheels
        self._linear(speed)

    def backward(self, speed: float):
        """Езда назад: set same negative speed on all wheels."""
        dlogging.info(f"Command: backward speed={speed}")
        # move backward on all wheels
        self._linear(-speed)

    def strafe_right(self, speed: float):
        """Страйф вправо: mecanum vector strafe, ближние колеса внутрь, дальние наружу."""
        dlogging.info(f"Command: strafe_right speed={speed}")
        self.drive_vector(vx=0.0, vy=speed, omega=0.0)

    def strafe_left(self, speed: float):
        """Страйф влево: mecanum vector strafe, ближние колеса внутрь, дальние наружу."""
        dlogging.info(f"Command: strafe_left speed={speed}")
        self.drive_vector(vx=0.0, vy=-speed, omega=0.0)
