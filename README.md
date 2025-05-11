# Управление Mecanum-шасси на базе Dynamixel MX-106

Проект предоставляет единый Python API и набор CLI/скриптов для управления четырёхколёсным шасси с mecanum-колёсами на сервоприводах Dynamixel MX-106 в режиме колёс (wheel-mode).

## Возможности

- Инициализация и настройка порта (device, baudrate, protocol)
- Управление линейным движением (вперёд/назад)
- Векторное управление (страйф, диагонали)
- Повороты: на месте, вокруг углового колеса, вокруг боковой оси
- Набор тестовых сценариев для отладки и калибровки

## Установка

```bash
# Клонировать репозиторий
git clone https://github.com/ваш_путь/python_dynamixel_106.git
cd python_dynamixel_106

# Создать виртуальное окружение (рекомендуется)
python3 -m venv venv
source venv/bin/activate

# Установить зависимости
pip install dynamixel_sdk pyyaml
```  

## Конфигурация

Создайте файл `config.yaml` или `.env` в корне проекта со следующими параметрами:

```yaml
# config.yaml
device: "/dev/ttyUSB0"
baud: 57600
protocol: 1   # версия протокола Dynamixel (1 или 2)
ids: [2,7,8,9]
invert: [1,-1,-1,1]    # карта инверсии направления для каждого ID
speed_scale: 500       # масштаб скорости (м/с → raw)
```

## Использование API

```python
from controller import DXController

with DXController() as robot:
    robot.forward(0.2)       # движение вперёд 0.2 м/с
    robot.stop()             # остановка
    robot.strafe_right(0.3)  # страйф вправо 0.3 м/с
    robot.stop()
    robot.turn_left(1.0)     # поворот против часовой 1 рад/с
    robot.stop()
```

## CLI (тестовые скрипты)

Все тесты находятся в папке `tests/`. Запуск каждого теста через модульный запуск:

```bash
python -m tests.forward_backward --speed 1.0 --duration 2.0
python -m tests.strafe           --speed 0.3 --duration 3.0 --angle-offset 0.1
python -m tests.diagonal         --speed 0.3 --duration 3.0
python -m tests.turn             --omega 1.0 --duration 3.0
python -m tests.in_place         --omega 1.0 --duration 3.0
python -m tests.pivot_corner     --corner rr --omega 1.0 --duration 3.0
python -m tests.pivot_side       --side right --omega 1.0 --duration 3.0
```

### Описание тестов

1. **forward_backward.py**  
   Движение вперёд/назад с задаваемой высокой скоростью.

2. **strafe.py**  
   Страйф вправо и влево с опцией `--angle-offset` (коррекция угла).

3. **diagonal.py**  
   Диагональные перемещения: вперёд-вправо, вперёд-влево, назад-вправо, назад-влево.

4. **turn.py**  
   Повороты на месте: по и против часовой стрелки.

5. **in_place.py**  
   Ещё один сценарий in-place вращения с явным разделением направлений.

6. **pivot_corner.py**  
   Поворот вокруг указанного углового колеса (`fl`, `fr`, `rr`, `rl`).

7. **pivot_side.py**  
   Поворот вокруг боковой оси: левая или правая сторона.

## Лицензия

Проект распространяется под лицензией MIT. См. файл `LICENSE`.