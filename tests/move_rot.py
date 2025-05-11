#!/usr/bin/env python3
"""
test_vector.py — расширенный тест бокового движения, диагоналей и разворотов
6 шагов с параметрами:
  --speed   (m/s) скорость страйфа/диагоналей
  --omega   (rad/s) скорость разворотов
  --duration (s) длительность каждого движения
Пауза 2 секунды между шагами.
"""
import time
import argparse
import logging
from controller import DXController

# Логирование
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="Расширенный тест движения и разворотов")
    parser.add_argument("--speed", type=float, default=1,
                        help="Скорость м/с для страйфа и диагоналей (default 1.0)")
    parser.add_argument("--omega", type=float, default=2.0,
                        help="Угловая скорость рад/с для разворотов (default 2.0)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Длительность каждого движения в секундах (default 3.0)")
    args = parser.parse_args()

    logger.info("Запуск расширенного теста движения и разворотов")
    with DXController() as robot:
        # Страйф вправо
        logger.info(f"1) страйф вправо speed={args.speed} m/s, duration={args.duration}s")
        robot.strafe_right(args.speed)
        time.sleep(args.duration)
        robot.stop()
        time.sleep(2)

        # Страйф влево
        logger.info(f"2) страйф влево speed={args.speed} m/s, duration={args.duration}s")
        robot.strafe_left(args.speed)
        time.sleep(args.duration)
        robot.stop()
        time.sleep(2)

        # Диагональ вперед-вправо
        diag = args.speed / 2**0.5
        logger.info(f"3) диагональ вперед-вправо speed={args.speed} m/s, duration={args.duration}s")
        robot.drive_vector(vx=diag, vy=diag, omega=0.0)
        time.sleep(args.duration)
        robot.stop()
        time.sleep(2)

        # Диагональ назад-влево
        logger.info(f"4) диагональ назад-влево speed={args.speed} m/s, duration={args.duration}s")
        robot.drive_vector(vx=-diag, vy=-diag, omega=0.0)
        time.sleep(args.duration)
        robot.stop()
        time.sleep(2)

        # Поворот против часовой
        logger.info(f"5) поворот против часовой omega={args.omega} rad/s, duration={args.duration}s")
        robot.turn_left(args.omega)
        time.sleep(args.duration)
        robot.stop()
        time.sleep(2)

        # Поворот по часовой
        logger.info(f"6) поворот по часовой omega={args.omega} rad/s, duration={args.duration}s")
        robot.turn_right(args.omega)
        time.sleep(args.duration)
        robot.stop()

    logger.info("Расширенный тест завершён")


if __name__ == '__main__':
    main()
