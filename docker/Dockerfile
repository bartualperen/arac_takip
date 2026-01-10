FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Istanbul

# 1. Hazırlık: Temel Araçlar ve Repolar
RUN apt-get update && apt-get install -y \
    software-properties-common \
    lsb-release \
    sudo \
    git \
    python3-pip \
    python3-dev \
    wget \
    curl \
    nano \
    tzdata \
    cmake \
    build-essential \
    gnupg \
    && add-apt-repository universe \
    && add-apt-repository multiverse \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*

# 2. Gazebo Harmonic Kurulumu
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic

# 3. Ardupilot Kurulumu (Ana Dizine)
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
    gstreamer1.0-gl

# Python Paketleri
RUN pip3 install MAVProxy pymavlink future empy==3.3.4

# 4. Ardupilot Gazebo Plugin Kurulumu
WORKDIR /gz_ws/src
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo

WORKDIR /gz_ws/src/ardupilot_gazebo
RUN mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j4

# Çevre Değişkenleri (Herkes için geçerli olması için global tanımlıyoruz)
ENV PATH=$PATH:/ardupilot/Tools/autotest
ENV PATH=$PATH:/root/.local/bin
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
ENV GZ_SIM_RESOURCE_PATH=/gz_ws/src/ardupilot_gazebo/models:/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
ENV GZ_SIM_RENDER_ENGINE_BACKEND=ogre2

# 5. Kullanıcı Oluşturma
ARG USERNAME=droneuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN chown -R $USERNAME:$USER_GID /ardupilot /gz_ws

USER $USERNAME
WORKDIR /home/$USERNAME