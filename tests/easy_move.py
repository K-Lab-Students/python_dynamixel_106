from controller import DXController
import time
with DXController("config.yaml") as robot:
    # вперёд 0.2 м/с
    robot.drive(0.2, 0.0)
    time.sleep(2)
    # поворот против часовой 0.5 рад/с
    robot.drive(0.0, 0.5)
    time.sleep(1)
    # стоп
    robot.drive(0.0, 0.0)
