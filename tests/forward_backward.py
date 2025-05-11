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
    parser = argparse.ArgumentParser(description="Тест движения вперед/назад с высокой скоростью")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Линейная скорость м/с (по умолчанию 1.0)")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Время движения в секундах (по умолчанию 2.0)")
    args = parser.parse_args()

    # Список тестов: (метод, описание)
    tests = [("forward", "вперед"), ("backward", "назад")]

    with DXController() as robot:
        for idx, (method, direction) in enumerate(tests, start=1):
            logger.info(f"{idx}) движение {direction} speed={args.speed} m/s, duration={args.duration}s")
            getattr(robot, method)(args.speed)
            time.sleep(args.duration)
            robot.stop()
            time.sleep(1)

    logger.info("Тест движения вперед/назад завершён")


if __name__ == '__main__':
    main() 