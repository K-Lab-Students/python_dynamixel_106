#!/usr/bin/env python3
import time
import argparse
import logging
import math
from controller import DXController

# Настройка логирования
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="Общий тестовый сценарий всех команд")
    parser.add_argument("--speed", type=float, default=0.3,
                        help="Линейная скорость м/с (по умолчанию 0.3)")
    parser.add_argument("--omega", type=float, default=1.0,
                        help="Угловая скорость рад/с (по умолчанию 1.0)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Длительность каждой команды в секундах (по умолчанию 10.0)")
    args = parser.parse_args()

    speed = args.speed
    omega = args.omega
    duration = args.duration
    diag = speed / math.sqrt(2)

    # Список команд: (метод, args, описание)
    commands = [
        ("forward", (speed,), "вперед"),
        ("backward", (speed,), "назад"),
        ("strafe_right", (speed,), "страйф вправо"),
        ("strafe_left", (speed,), "страйф влево"),
        ("drive_vector", (diag, diag, 0.0), "диагональ вперед-вправо"),
        ("drive_vector", (diag, -diag, 0.0), "диагональ вперед-влево"),
        ("drive_vector", (-diag, diag, 0.0), "диагональ назад-вправо"),
        ("drive_vector", (-diag, -diag, 0.0), "диагональ назад-влево"),
        ("turn_left", (omega,), "поворот против часовой"),
        ("turn_right", (omega,), "поворот по часовой"),
        ("pivot_around_corner", (omega, "rr"), "поворот вокруг заднего правого угла"),
        ("pivot_around_side", (omega, "right"), "поворот вокруг правой стороны"),
        ("drive", (0.0, omega), "танковый разворот против часовой"),
        ("drive", (0.0, -omega), "танковый разворот по часовой"),
    ]

    with DXController() as robot:
        for idx, (method, margs, desc) in enumerate(commands, start=1):
            logger.info(f"{idx}) {desc}, speed/omega={margs}, duration={duration}s")
            func = getattr(robot, method)
            func(*margs)
            time.sleep(duration)
            robot.stop()
            time.sleep(1)

    logger.info("Общий тестовый сценарий завершён")


if __name__ == '__main__':
    main() 