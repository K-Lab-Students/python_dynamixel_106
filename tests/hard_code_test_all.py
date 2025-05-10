import time
import logging
from controller import DXController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)


def run_scenario():
    logger.info("Запуск жестко заданного сценария")
    with DXController() as robot:
        # 1) Движение вперед
        logger.info("Шаг 1: движение 0.2 м/с вперед, 3 секунды")
        robot.drive(v=0.2, omega=0.0)
        time.sleep(3)

        # 2) Поворот на месте
        logger.info("Шаг 2: поворот против часовой, 0.5 рад/с, 2 секунды")
        robot.drive(v=0.0, omega=0.5)
        time.sleep(2)

        # 3) Движение назад
        logger.info("Шаг 3: движение -0.1 м/с назад, 2 секунды")
        robot.drive(v=-0.1, omega=0.0)
        time.sleep(2)

        # 4) Стоп
        logger.info("Шаг 4: остановка")
        robot.drive(v=0.0, omega=0.0)
        time.sleep(1)

    logger.info("Сценарий завершен")


if __name__ == '__main__':
    run_scenario()
