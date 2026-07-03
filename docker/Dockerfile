FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Istanbul
ENV HSA_OVERRIDE_GFX_VERSION=10.3.0
ENV PYTORCH_ROCM_ARCH=gfx1030
ENV HIP_VISIBLE_DEVICES=0

# 1. Hazırlık: Temel Araçlar ve Repolar
RUN apt-get update && apt-get install -y \
    software-properties-common \
    lsb-release \
    sudo \
    git \
    python3-pip \
    python3-venv \
    python3-dev \
    wget \
    curl \
    nano \
    tzdata \
    locales \
    cmake \
    build-essential \
    gnupg \
    && add-apt-repository universe \
    && add-apt-repository multiverse \
    && apt-get update \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install \
    numpy \
    scipy \
    scikit-learn \
    matplotlib \
    pandas \
    jupyterlab \
    seaborn \
    opencv-python

# 2. ROS 2 Humble Kurulumu
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# 3. Gazebo Harmonic Kurulumu
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic

# 4. Ardupilot Kurulumu (Ana Dizine)
WORKDIR /
RUN git clone --recursive https://github.com/ArduPilot/ardupilot.git

# Ardupilot Bağımlılıkları
WORKDIR /ardupilot
RUN apt-get update && apt-get install -y \
    python3-matplotlib \
    python3-serial \
    python3-wxgtk4.0 \
    python3-lxml \
    python3-scipy \
    python3-opencv \
    ccache \
    gawk \
    python3-pexpect \
    libgz-sim8-dev \
    rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libdebuginfod1

# Python Paketleri
# Büyük wheel indirmelerinde timeout yaşamamak için pip timeout/retry artırıldı.
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir --upgrade pip setuptools wheel \
    && /opt/venv/bin/pip install --no-cache-dir --retries 10 --timeout 300 --prefer-binary \
    MAVProxy \
    dronekit \
    pymavlink \
    pexpect \
    future \
    empy==3.3.4 \
    filterpy \
    "numpy<2.0" \
    scikit-learn \
    ultralytics \
    && /opt/venv/bin/pip install --no-cache-dir --retries 10 --timeout 300 --prefer-binary \
    tensorflow-cpu==2.15.1

# 5. Ardupilot Gazebo Plugin Kurulumu (Geçici clone - daha sonra host'tan senkronize edilecek)
WORKDIR /gz_ws/src
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo

WORKDIR /gz_ws/src/ardupilot_gazebo
RUN mkdir -p build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j8 \
    && make install

# Çevre Değişkenleri (Herkes için geçerli olması için global tanımlıyoruz)
ENV PATH=$PATH:/ardupilot/Tools/autotest
ENV PATH=$PATH:/root/.local/bin
ENV PATH=/opt/venv/bin:$PATH
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=""
ENV GZ_SIM_RESOURCE_PATH=""
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
ENV GZ_SIM_RESOURCE_PATH=/gz_ws/src/ardupilot_gazebo/models:/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
ENV GZ_SIM_RENDER_ENGINE_BACKEND=ogre2
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 6. Kullanıcı Oluşturma
ARG USERNAME=droneuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN chown -R $USERNAME:$USER_GID /ardupilot /gz_ws

# Entrypoint script'i kopyala ve çalıştırılabilir yap (root olarak çalışması için)
COPY docker/entrypoint.sh /home/droneuser/entrypoint.sh
RUN chmod +x /home/droneuser/entrypoint.sh

USER $USERNAME
WORKDIR /home/$USERNAME
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
