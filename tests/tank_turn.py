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
    parser = argparse.ArgumentParser(description="Тест танкового разворота (drive v=0, omega)")
    parser.add_argument("--omega", type=float, default=1.0,
                        help="Угловая скорость рад/с (по умолчанию 1.0)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Время поворота в секундах (по умолчанию 3.0)")
    args = parser.parse_args()

    with DXController() as robot:
        logger.info(f"Tank pivot: counterclockwise omega={args.omega}, duration={args.duration}")
        robot.drive(0.0, args.omega)
        time.sleep(args.duration)
        robot.stop()
        time.sleep(1)

        logger.info(f"Tank pivot: clockwise omega={args.omega}, duration={args.duration}")
        robot.drive(0.0, -args.omega)
        time.sleep(args.duration)
        robot.stop()

    logger.info("Тест танкового разворота завершён")


if __name__ == '__main__':
    main() 