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
    parser = argparse.ArgumentParser(description="In-Place Rotation test")
    parser.add_argument("--omega", type=float, default=1.0,
                        help="Angular speed rad/s (positive = clockwise)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Duration seconds")
    args = parser.parse_args()

    with DXController() as robot:
        logger.info(f"In-place rotation clockwise omega={args.omega}, duration={args.duration}")
        robot.turn_right(args.omega)
        time.sleep(args.duration)
        robot.stop()
        time.sleep(1)

        logger.info(f"In-place rotation counterclockwise omega={args.omega}, duration={args.duration}")
        robot.turn_left(args.omega)
        time.sleep(args.duration)
        robot.stop()

    logger.info("Test in-place rotation completed")

if __name__ == '__main__':
    main() 