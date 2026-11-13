FROM ubuntu:26.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=lyrical \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl ca-certificates gnupg \
        python3-pip python3-venv \
        qt6-base-dev libgl1 libglib2.0-0 \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-dev-tools

RUN python3 -m venv /opt/venv --system-site-packages
COPY requirements.txt /tmp/requirements.txt
RUN /opt/venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
    && rm -f /tmp/requirements.txt

WORKDIR /opt/ws
COPY src ./src

RUN rosdep init || true \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

RUN printf '%s\n' \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "source /opt/ws/install/setup.bash" \
    "source /opt/venv/bin/activate" \
    'export PYTHONPATH="${PYTHONPATH}:$(echo /opt/venv/lib/python*/site-packages)"' \
    >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/ws/install/setup.bash && source /opt/venv/bin/activate && export PYTHONPATH=\"${PYTHONPATH}:$(echo /opt/venv/lib/python*/site-packages)\" && exec \"$@\"", "--"]
CMD ["ros2", "launch", "manipulador_control", "sistema.launch.py", "use_gui:=false", "camera_index:=0", "serial_port:=/dev/ttyACM0"]
