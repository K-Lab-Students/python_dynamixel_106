import time
import logging
from math import pi, sqrt
from controller import DXController

# Настройка логирования
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)


def run_scenario():
    logger.info("Запуск жестко заданного сценария (6 шагов по 1 сек)")
    with DXController() as robot:
        # Шаг a) движение вперед 0.3 м/с, 1 сек → 0.3 м
        logger.info("Шаг a): вперед 0.3 м/с, 1 с")
        robot.drive(v=0.3, omega=0.0)
        time.sleep(1)

        # Шаг b) движение вправо (strafe) 0.3 м/с, 1 сек → 0.3 м
        logger.info("Шаг b): вправо 0.3 м/с, 1 с")
        robot.drive_vector(vx=0.0, vy=0.3, omega=0.0)
        time.sleep(1)

        # Шаг c) движение по диагонали вперед-вправо
        diag = 0.3 / sqrt(2)
        logger.info("Шаг c): диагональ вперед-вправо 0.3 m/s, 1 с")
        robot.drive_vector(vx=diag, vy=diag, omega=0.0)
        time.sleep(1)

        # Шаг d) движение по диагонали назад-вправо
        logger.info("Шаг d): диагональ назад-вправо 0.3 m/s, 1 с")
        robot.drive_vector(vx=-diag, vy=diag, omega=0.0)
        time.sleep(1)

        # Шаг e) поворот против часовой 90° за 1 сек
        omega_90 = (pi/2) / 1.0  # rad/s
        logger.info("Шаг e): поворот против часовой 90°, 1 с")
        robot.drive(v=0.0, omega=omega_90)
        time.sleep(1)

        # Шаг f) поворот по часовой 90° за 1 сек
        logger.info("Шаг f): поворот по часовой 90°, 1 с")
        robot.drive(v=0.0, omega=-omega_90)
        time.sleep(1)

        # Завершение: стоп
        logger.info("Стоп: остановка")
        robot.drive(v=0.0, omega=0.0)
        time.sleep(1)

    logger.info("Сценарий завершен")


if __name__ == '__main__':
    run_scenario()
