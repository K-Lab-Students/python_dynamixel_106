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
    logger.info("Запуск жестко заданного сценария: 6 шагов по 3 с, пауза 5 с между этапами")
    with DXController() as robot:
        # Шаг a) вперед
        logger.info("Шаг a): вперед 0.3 м/с, 3 с")
        robot.drive(v=0.3, omega=0.0)
        time.sleep(3)
        robot.drive(0.0, 0.0)
        time.sleep(5)

        # Шаг b) вправо (strafe)
        logger.info("Шаг b): вправо 0.3 м/с, 3 с")
        robot.drive_vector(vx=0.0, vy=0.3, omega=0.0)
        time.sleep(3)
        robot.drive(v=0.0, omega=0.0)
        time.sleep(5)

        # Шаг c) диагональ вперед-вправо
        diag = 0.3 / sqrt(2)
        logger.info("Шаг c): диагональ вперед-вправо 0.3 м/с, 3 с")
        robot.drive_vector(vx=diag, vy=diag, omega=0.0)
        time.sleep(3)
        robot.drive(v=0.0, omega=0.0)
        time.sleep(5)

        # Шаг d) диагональ назад-вправо
        logger.info("Шаг d): диагональ назад-вправо 0.3 м/с, 3 с")
        robot.drive_vector(vx=-diag, vy=diag, omega=0.0)
        time.sleep(3)
        robot.drive(v=0.0, omega=0.0)
        time.sleep(5)

        # Шаг e) поворот против часовой 90° за 3 с
        omega_90 = (pi/2) / 3.0  # рад/с
        logger.info("Шаг e): поворот против часовой 90°, 3 с")
        robot.drive(v=0.0, omega=omega_90)
        time.sleep(3)
        robot.drive(v=0.0, omega=0.0)
        time.sleep(5)

        # Шаг f) поворот по часовой 90° за 3 с
        logger.info("Шаг f): поворот по часовой 90°, 3 с")
        robot.drive(v=0.0, omega=-omega_90)
        time.sleep(3)
        robot.drive(v=0.0, omega=0.0)
        time.sleep(5)

    logger.info("Сценарий завершен")


if __name__ == '__main__':
    run_scenario()