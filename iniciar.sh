#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# 1. Carrega o ROS 2
ROS_DISTRO="${ROS_DISTRO:-lyrical}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
export MANIPULADOR_DATA_DIR="${MANIPULADOR_DATA_DIR:-$WS_DIR}"

# 2. Recompila alterações (rápido se nada mudou em C++, completo se algo foi editado)
echo "[Manipulador ROS 2] Verificando e compilando pacotes..."
colcon build --symlink-install

# 3. Carrega o workspace compilado e o ambiente Python local (.venv), se existir
source "$WS_DIR/install/setup.bash"
if [ -f "$WS_DIR/.venv/bin/activate" ]; then
    source "$WS_DIR/.venv/bin/activate"
    VENV_PATHS="$(python3 -c 'import sys; print(":".join(p for p in sys.path if p))')"
    export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$VENV_PATHS"
fi

echo "[Manipulador ROS 2] Iniciando sistema..."
exec ros2 launch manipulador_control sistema.launch.py "$@"
