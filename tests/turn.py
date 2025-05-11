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
    parser = argparse.ArgumentParser(description="Тест поворотов по и против часовой стрелки")
    parser.add_argument("--omega", type=float, default=1.0,
                        help="Угловая скорость рад/с (по умолчанию 1.0)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Время поворота в секундах (по умолчанию 3.0)")
    args = parser.parse_args()

    # Список тестов: (метод, описание)
    tests = [("turn_left", "против часовой"), ("turn_right", "по часовой")]

    with DXController() as robot:
        for idx, (method, direction) in enumerate(tests, start=1):
            logger.info(f"{idx}) поворот {direction} omega={args.omega} rad/s, duration={args.duration}s")
            getattr(robot, method)(args.omega)
            time.sleep(args.duration)
            robot.stop()
            time.sleep(1)

    logger.info("Тест поворотов завершён")


if __name__ == '__main__':
    main() 