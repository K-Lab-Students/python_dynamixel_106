#!/usr/bin/env python3
import time
import argparse
import logging
from controller import DXController

# Настройка логирования
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="Тест страйфов вправо и влево")
    parser.add_argument("--speed", type=float, default=0.3,
                        help="Скорость м/с (по умолчанию 0.3)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Время движения в секундах (по умолчанию 3.0)")
    args = parser.parse_args()

    # Список тестов: (метод, описание)
    tests = [("strafe_right", "вправо"), ("strafe_left", "влево")]

    with DXController() as robot:
        for idx, (method, direction) in enumerate(tests, start=1):
            logger.info(f"{idx}) страйф {direction} speed={args.speed} m/s, duration={args.duration}s")
            getattr(robot, method)(args.speed)
            time.sleep(args.duration)
            robot.stop()
            time.sleep(1)

    logger.info("Тест страйфов завершён")


if __name__ == '__main__':
    main() 