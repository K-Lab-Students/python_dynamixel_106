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
    parser = argparse.ArgumentParser(description="Тест диагонального перемещения")
    parser.add_argument("--speed", type=float, default=0.3,
                        help="Скорость движения м/с (по умолчанию 0.3)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Время движения в секундах (по умолчанию 3.0)")
    args = parser.parse_args()

    diag = args.speed / math.sqrt(2)
    # Список тестов: (описание, vx, vy)
    tests = [
        ("вперед-вправо",  diag,  diag),
        ("вперед-влево",   diag, -diag),
        ("назад-вправо",  -diag,  diag),
        ("назад-влево",   -diag, -diag),
    ]

    with DXController() as robot:
        for idx, (desc, vx, vy) in enumerate(tests, start=1):
            logger.info(f"{idx}) диагональ {desc} vx={vx:.3f}, vy={vy:.3f}, duration={args.duration}s")
            robot.drive_vector(vx=vx, vy=vy, omega=0.0)
            time.sleep(args.duration)
            robot.stop()
            time.sleep(1)

    logger.info("Тест диагонального перемещения завершён")


if __name__ == '__main__':
    main() 