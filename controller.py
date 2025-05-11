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
import math

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
        # send speed commands to motors
        self.motors.set_speed(raw)

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

    def strafe_right(self, speed: float, angle_offset: float = 0.0):
        """Страйф вправо с возможностью коррекции угла: ближние колеса внутрь, дальние наружу."""
        dlogging.info(f"Command: strafe_right speed={speed}, angle_offset={angle_offset}")
        # Base strafe vector (y positive = right)
        vx, vy = 0.0, speed
        # Apply rotation offset if provided
        if angle_offset:
            cos_o, sin_o = math.cos(angle_offset), math.sin(angle_offset)
            vx, vy = vx * cos_o - vy * sin_o, vx * sin_o + vy * cos_o
        # Execute mecanum vector drive
        self.drive_vector(vx=vx, vy=vy, omega=0.0)

    def strafe_left(self, speed: float, angle_offset: float = 0.0):
        """Страйф влево с возможностью коррекции угла: ближние колеса внутрь, дальние наружу."""
        dlogging.info(f"Command: strafe_left speed={speed}, angle_offset={angle_offset}")
        # Base strafe vector (y negative = left)
        vx, vy = 0.0, -speed
        # Apply rotation offset if provided
        if angle_offset:
            cos_o, sin_o = math.cos(angle_offset), math.sin(angle_offset)
            vx, vy = vx * cos_o - vy * sin_o, vx * sin_o + vy * cos_o
        # Execute mecanum vector drive
        self.drive_vector(vx=vx, vy=vy, omega=0.0)

    def pivot_around_corner(self, omega: float, corner: str = 'rr'):
        """Pivot around a specified corner (fl, fr, rr, rl)."""
        idx_map = {'fl': 0, 'fr': 1, 'rr': 2, 'rl': 3}
        corner_l = corner.lower()
        if corner_l not in idx_map:
            raise ValueError(f"Unknown corner '{corner}'. Must be one of {list(idx_map.keys())}")
        pivot_idx = idx_map[corner_l]
        half_x = self.L_X / 2
        half_y = self.L_Y / 2
        positions = {
            0: ( half_x, -half_y),  # FL
            1: ( half_x,  half_y),  # FR
            2: (-half_x,  half_y),  # RR
            3: (-half_x, -half_y),  # RL
        }
        pivot_pos = positions[pivot_idx]
        raw = {}
        for i, mid in enumerate(self.ids):
            if i == pivot_idx:
                raw[mid] = 0
            else:
                dx = positions[i][0] - pivot_pos[0]
                dy = positions[i][1] - pivot_pos[1]
                dist = math.hypot(dx, dy)
                linear = abs(omega) * dist
                val = self.to_raw(linear)
                # positive omega => clockwise rotation
                direction = 1 if omega > 0 else -1
                # front-right wheel spins opposite direction
                if i == 1:
                    direction *= -1
                raw[mid] = direction * val
        dlogging.info(f"Pivot around {corner_l} -> omega={omega}, raw={raw}")
        self.motors.set_speed(raw)

    def pivot_around_side(self, omega: float, side: str = 'right'):
        """Pivot around a specified side (left or right)."""
        side_l = side.lower()
        if side_l not in ('left', 'right'):
            raise ValueError(f"Unknown side '{side}'. Must be 'left' or 'right'")
        # pivot wheels indices: right side => FR,RR; left side => FL,RL
        pivot_idxs = [1, 2] if side_l == 'right' else [0, 3]
        # distance from pivot axis to opposite wheels
        dist = self.L_Y
        linear = abs(omega) * dist
        val = self.to_raw(linear)
        direction = 1 if omega > 0 else -1
        raw = {}
        for i, mid in enumerate(self.ids):
            if i in pivot_idxs:
                raw[mid] = 0
            else:
                raw[mid] = direction * val
        dlogging.info(f"Pivot around side {side_l} -> omega={omega}, raw={raw}")
        self.motors.set_speed(raw)

    def turn_left(self, omega: float):
        """In-place rotation: counter-clockwise turn by angular speed omega (rad/s)."""
        dlogging.info(f"Command: turn_left omega={omega}")
        self.drive(v=0.0, omega=omega)

    def turn_right(self, omega: float):
        """In-place rotation: clockwise turn by angular speed omega (rad/s)."""
        dlogging.info(f"Command: turn_right omega={omega}")
        self.drive(v=0.0, omega=-omega)
