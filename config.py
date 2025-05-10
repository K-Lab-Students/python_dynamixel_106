import os
from dotenv import load_dotenv

def load_config():
    # подгружаем .env, если есть
    load_dotenv()

    return {
        "device":      os.getenv("DX_DEVICE", "/dev/ttyUSB0"),
        "baud":        int(os.getenv("DX_BAUD", "57600")),
        "protocol":    float(os.getenv("DX_PROTOCOL", "1.0")),
        "ids":         list(map(int, os.getenv("DX_IDS",  "2,7,8,9").split(","))),
        "invert":      list(map(int, os.getenv("DX_INVERT", "1,-1,-1,1").split(","))),
        "speed_scale": float(os.getenv("DX_SPEED_SCALE", "500")),
    }
