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
    parser = argparse.ArgumentParser(description="Pivot Around Side test")
    parser.add_argument("--side", choices=["left", "right"], default="right",
                        help="Side to pivot around (default: right)")
    parser.add_argument("--omega", type=float, default=1.0,
                        help="Angular speed rad/s (positive = clockwise)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Duration seconds")
    args = parser.parse_args()

    with DXController() as robot:
        logger.info(f"Pivot around side {args.side} omega={args.omega}, duration={args.duration}")
        robot.pivot_around_side(args.omega, args.side)
        time.sleep(args.duration)
        robot.stop()
    logger.info("Test pivot around side completed")

if __name__ == '__main__':
    main() 