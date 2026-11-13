# Manipulador Robótico 6-DOF (Arctos / GRBL) — ROS 2

Workspace ROS 2 para controle, planejamento de trajetória por curva de Bézier, cinemática inversa (Craig) e visão computacional (YOLO + ArUco) do manipulador **Arctos 6-DOF** com firmware **GRBL** (ou simulador `MockSerialGRBL` automático quando o hardware estiver desconectado).

---

## Estrutura do Projeto

```text
manipulador_ros2_ws/
├── iniciar.sh                   # Compila (colcon) e inicia o sistema
├── requirements.txt             # Dependências Python (YOLO, OpenCV, pyserial, roboticstoolbox)
├── Dockerfile                   # Imagem Ubuntu 26.04 + ROS 2 Lyrical (amd64 / arm64)
├── docker-compose.yml           # Perfis para rodar com ou sem GUI
└── src/
    ├── arctos_urdf_description/ # URDF/meshes do robô + plugin de painéis RViz2 (C++/Qt6)
    └── manipulador_control/     # Nó principal ROS 2, IK, Bézier, driver GRBL, câmera/YOLO e launch
```

---

## 1. Como Baixar os Arquivos

Em qualquer computador ou no Raspberry Pi:

```bash
git clone https://github.com/<seu-usuario>/manipulador_ros2_ws.git
cd manipulador_ros2_ws
```

---

## 2. Uso Nativo (Ubuntu 26.04 + ROS 2 Lyrical)

### Pré-requisitos
- **ROS 2 Lyrical** (variante `ros-lyrical-desktop` + `ros-dev-tools`) instalado em `/opt/ros/lyrical` — siga o [guia oficial do ROS 2 Lyrical](https://docs.ros.org/en/lyrical/)
- `colcon`, `rosdep` e Python 3 com `venv`

### Instalação e Execução

```bash
# 1. Dependências ROS 2 do workspace
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 2. Ambiente virtual Python local (enxergando o rclpy do sistema)
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt

# 3. Compilar e iniciar o sistema completo (RViz2 + controle + câmera)
./iniciar.sh
```

Para passar parâmetros de câmera/serial ou iniciar sem interface gráfica:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch manipulador_control sistema.launch.py use_gui:=true camera_index:=0 serial_port:=/dev/ttyACM0
```

---

## 3. Uso com Docker no Raspberry Pi 5 (Raspberry Pi OS)

O **Raspberry Pi OS** (Debian) não possui pacotes `.deb` oficiais do ROS 2 no `apt`. Por isso, no Pi 5 o sistema roda dentro de um container **Ubuntu 26.04 (ARM64) com ROS 2 Lyrical**.

### 3.1. Preparação do Raspberry Pi OS (fazer uma única vez)

1. **Instale o Docker e o Docker Compose:**
   ```bash
   curl -fsSL https://get.docker.com | sh
   sudo usermod -aG docker "$USER"
   newgrp docker
   ```
   *(Confirme se `docker compose version` responde normalmente; se necessário, instale `docker-compose-plugin` via `apt`.)*

2. **Conecte os periféricos USB (opcional para simulação):**
   - **Arduino/GRBL:** verifique se apareceu em `/dev/ttyACM0` (ou `/dev/ttyUSB0`). Se não estiver conectado, o sistema entra automaticamente no modo simulado (`MockSerialGRBL`).
   - **Webcam USB:** verifique em `/dev/video0` (`ls /dev/video*`).
   > **Nota:** Para câmera **CSI** (cabo flat) do Raspberry Pi, prefira uma **webcam USB** padrão (UVC/V4L2), pois `cv2.CAP_V4L2` não lê diretamente o fluxo `libcamera` sem ponte adicional.
   > O `docker-compose.yml` já mapeia `/dev:/dev` em modo `privileged: true`, então os containers sobem mesmo se a webcam ou o Arduino estiverem desconectados.

3. **Compile a imagem Docker no Raspberry Pi:**
   ```bash
   cd manipulador_ros2_ws
   docker compose build
   ```
   *(Na primeira vez no Pi 5 pode levar alguns minutos para baixar o ROS 2 Lyrical, dependências Python e compilar o workspace).*

4. **Para atualizar o código depois de um `git push`:**
   ```bash
   git pull
   docker compose build
   ```

---

### 3.2. Rodando SEM GUI no Raspberry Pi (`use_gui:=false`) — Recomendado

Neste modo, o Raspberry Pi 5 executa **apenas os nós de controle, GRBL, câmera e YOLO** (sem abrir o RViz2 localmente), economizando CPU/memória. Você pode controlar o robô **tanto diretamente pelo próprio Raspberry Pi** (via CLI `ros2 topic pub` / `ros2 service call` ou scripts ROS 2) **quanto visualizando/controlando pelo RViz2 em outro computador** na mesma rede local.

#### Como definir o `ROS_DOMAIN_ID` (obrigatório ser igual nas duas máquinas)

O `ROS_DOMAIN_ID` (inteiro de `0` a `101`, padrão `0`) isola os nós na rede local. Para que o outro computador enxergue o Raspberry Pi, os dois precisam usar o **mesmo número**:

- **Na sessão atual do terminal (PC ou Raspberry Pi):**
  ```bash
  export ROS_DOMAIN_ID=10
  ```
- **De forma permanente no terminal (`~/.bashrc`):**
  ```bash
  echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
  source ~/.bashrc
  ```
- **Ao subir o Docker Compose no Raspberry Pi:**
  ```bash
  ROS_DOMAIN_ID=10 docker compose up manipulador-headless
  ```
  *(Ou passe `-e ROS_DOMAIN_ID=10` no comando `docker run` abaixo.)*

1. **No Raspberry Pi 5:**
   ```bash
   docker compose up manipulador-headless
   ```
   Para trocar porta serial, câmera ou domínio ROS 2 sem editar o `docker-compose.yml`:
   ```bash
   ROS_DOMAIN_ID=10 CAMERA_INDEX=0 SERIAL_PORT=/dev/ttyUSB0 docker compose up manipulador-headless
   ```
   *(Para encerrar, pressione `Ctrl+C` ou rode `docker compose down` em outro terminal. Equivalente ao `docker run` passando rede host e dispositivos USB:)*
   ```bash
   docker run --rm -it \
     --network=host \
     --ipc=host \
     -e ROS_DOMAIN_ID=0 \
     --device=/dev/video0:/dev/video0 \
     --device=/dev/ttyACM0:/dev/ttyACM0 \
     manipulador_ros2_ws:latest \
     ros2 launch manipulador_control sistema.launch.py use_gui:=false camera_index:=0 serial_port:=/dev/ttyACM0
   ```
   > **Dica:** se estiver apenas testando sem câmera ou sem Arduino conectados, remova a flag `--device=...` do dispositivo ausente (ou passe `--group-add video --group-add dialout` se houver erro de permissão).

2. **Para visualizar e controlar pelo RViz2 em outro computador (na mesma rede Wi-Fi/Ethernet, com o mesmo `ROS_DOMAIN_ID`):**
   No outro PC (com o repositório clonado e compilado via `colcon build`, necessário para carregar o URDF e o painel lateral do RViz2), abra apenas o RViz2 para enxergar o robô, a câmera (`/camera/image_raw`), o Gizmo 6-DOF e o painel de controle:
   ```bash
   source /opt/ros/lyrical/setup.bash
   source install/setup.bash
   export ROS_DOMAIN_ID=0
   rviz2 -d src/manipulador_control/config/interface.rviz
   ```

3. **Para controlar pelo próprio Raspberry Pi via CLI (sem outro PC):**
   Com o container rodando, veja o nome dele com `docker ps` e abra um shell dentro do container (o `.bashrc` já carrega ROS 2, workspace e `.venv`):
   ```bash
   docker exec -it <nome_container> bash
   ros2 service call /planejar_home std_srvs/srv/Trigger
   ros2 service call /executar std_srvs/srv/Trigger
   ```

---

### 3.3. Rodando COM GUI direto no Raspberry Pi (`use_gui:=true`)

Use este modo se o Raspberry Pi 5 estiver ligado a um monitor (sessão gráfica X11 ou XWayland ativa) e você quiser abrir a janela completa do RViz2 no próprio Pi.

1. **Libere o acesso local ao servidor gráfico X11 no Raspberry Pi OS:**
   ```bash
   xhost +local:docker
   ```
   *(Se o Raspberry Pi OS estiver usando Wayland/labwc sem X11 exportado, mude a sessão para X11 em `sudo raspi-config` → **Advanced Options** → **Wayland** → **X11**, ou garanta que a variável `$DISPLAY` esteja ativa).*

2. **Suba o container com GUI:**
   ```bash
   docker compose up manipulador-gui
   ```
   *(Equivalente ao `docker run` completo com X11 e GPU/DRI:)*
   ```bash
   docker run --rm -it \
     --network=host \
     --ipc=host \
     -e DISPLAY="$DISPLAY" \
     -e QT_X11_NO_MITSHM=1 \
     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
     --device=/dev/dri:/dev/dri \
     --device=/dev/video0:/dev/video0 \
     --device=/dev/ttyACM0:/dev/ttyACM0 \
     manipulador_ros2_ws:latest \
     ros2 launch manipulador_control sistema.launch.py use_gui:=true camera_index:=0 serial_port:=/dev/ttyACM0
   ```

3. **Ao encerrar:** fechar a janela do RViz2 encerra automaticamente todos os nós dentro do container. Por segurança, se quiser revogar a permissão X11 depois:
   ```bash
   xhost -local:docker
   ```

---

## 4. Referência Rápida dos Parâmetros de Launch

| Parâmetro | Padrão | Descrição |
| :--- | :--- | :--- |
| `use_gui` | `true` | Se `true`, abre o RViz2 com os painéis integrados; se `false`, roda headless |
| `camera_index` | `0` | Índice da câmera V4L2 (`/dev/video<N>`); no `docker compose`, variável `CAMERA_INDEX` |
| `serial_port` | `/dev/ttyACM0` | Porta serial do GRBL (usa mock se indisponível); no `docker compose`, variável `SERIAL_PORT` |
| `MANIPULADOR_DATA_DIR` *(env)* | `~/.manipulador_ros2` (ou raiz no `iniciar.sh`) | Pasta gravável onde ficam `gcode_log.txt` e `capturas/` |
