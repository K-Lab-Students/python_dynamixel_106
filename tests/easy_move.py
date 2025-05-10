# easy_move.py
import time
from controller import DXController

if __name__ == "__main__":
    with DXController() as robot:
        robot.drive(0.2, 0.0)  # вперёд
        time.sleep(2)
        robot.drive(0.0, 0.5)  # поворот против часовой
        time.sleep(1)
        robot.drive(0.0, 0.0)  # стоп
