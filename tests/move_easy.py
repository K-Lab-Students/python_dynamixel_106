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
    parser = argparse.ArgumentParser(description="Тест движения вперед/назад")
    parser.add_argument("--speed", type=float, default=0.2,
                        help="Линейная скорость м/с (по умолчанию 0.2)")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Время движения в секундах (по умолчанию 2.0)")
    args = parser.parse_args()

    logger.info(f"Тест: движение вперед speed={args.speed} м/с, duration={args.duration} с")
    with DXController() as robot:
        robot.forward(args.speed)
        time.sleep(args.duration)
        robot.stop()

        time.sleep(1)

        logger.info(f"Тест: движение назад speed={args.speed} м/с, duration={args.duration} с")
        robot.backward(args.speed)
        time.sleep(args.duration)
        robot.stop()

    logger.info("Тест завершен")


if __name__ == '__main__':
    main()