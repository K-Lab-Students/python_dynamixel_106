#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# install_ros2_humble.sh
#
# Полный скрипт сборки ROS 2 Humble Hawksbill на Raspberry Pi OS (Debian 12 “Bookworm”)
# ==============================================================================

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Печать сообщений
info()    { echo -e "${GREEN}[INFO]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[WARNING]${NC} $*"; }
error()   { echo -e "${RED}[ERROR]${NC}   $*"; exit 1; }

# Проверка прав
if [[ $EUID -ne 0 ]]; then
  error "Скрипт нужно запускать от root: sudo $0"
fi

# Проверка Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
  error "Этот скрипт предназначен для Raspberry Pi OS"
fi

info "Начало установки ROS 2 Humble на Raspberry Pi OS (Bookworm)"

# ================================
# 1. Установка локали и базовых пакетов
# ================================
info "1. Установка локали и системных утилит"
apt update
apt install -y locales curl wget git build-essential \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    python3-rosdep python3-rosinstall-generator python3-argcomplete \
    pkg-config libpython3-dev cmake

info "Генерация и настройка локали UTF-8"
locale-gen en_US.UTF-8
update-locale LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8

# ================================
# 2. Инициализация rosdep
# ================================
info "2. Инициализация rosdep"
if ! rosdep init 2>/dev/null; then
  warn "rosdep уже инициализирован"
fi
rosdep update

# ================================
# 3. Создание рабочей области
# ================================
WORKSPACE=~/ros2_humble
info "3. Создание рабочей области в $WORKSPACE"
rm -rf "$WORKSPACE"
mkdir -p "$WORKSPACE/src"
cd "$WORKSPACE"

# ================================
# 4. Загрузка исходников ROS 2 Humble
# ================================
info "4. Загрузка .repos и импорт исходников"
wget -q https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos

# ================================
# 5. Установка зависимостей через rosdep
# ================================
info "5. Установка внешних зависимостей via rosdep"
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "rti-connext-dds-6.1.1 fastcdr urdfdom_headers python3-vcstool"

# ================================
# 6. Сборка colcon
# ================================
info "6. Сборка ROS 2 Humble (это может занять 2–4 часа)"
colcon build --merge-install

# ================================
# 7. Настройка окружения
# ================================
info "7. Настройка окружения в ~/.bashrc"
# Удаляем старые строки, если они есть
sed -i '/ros2_humble\/install\/setup.bash/d' ~/.bashrc || true
# Добавляем новые
echo ""                             >> ~/.bashrc
echo "# ROS 2 Humble setup"        >> ~/.bashrc
echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

info "Сборка окончена. Перезапустите терминал или выполните:"
echo "  source ~/.bashrc"
echo "Проверка: ros2 --version"
