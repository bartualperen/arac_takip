FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Istanbul

# 1. Hazırlık: Repolar
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && add-apt-repository multiverse \
    && apt-get update

# 2. Temel Araçlar ve Gazebo
# 'cmake' ve 'build-essential' ekledik
RUN apt-get install -y \
    lsb-release \
    sudo \
    git \
    python3-pip \
    python3-dev \
    wget \
    curl \
    nano \
    tzdata \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps \
    gazebo \
    libgazebo-dev \
    libopencv-dev \
    cmake \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# 3. Python Bağımlılıkları (NumPy Fix Dahil)
RUN pip3 install --no-cache-dir \
    MAVProxy \
    pexpect \
    future \
    pymavlink \
    empy==3.3.4 \
    "numpy<2.0.0"

# 4. Eksik GUI Paketleri (Console/Map için)
RUN apt-get update && apt-get install -y \
    python3-wxgtk4.0 \
    python3-matplotlib \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# 5. Kullanıcı Oluşturma
ARG USERNAME=droneuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# 6. ArduPilot Kaynak Kodunu İndir
RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

# 7. Gazebo Plugin Kurulumu (OTOMATİK)
RUN git clone https://github.com/khancyr/ardupilot_gazebo.git \
    && cd ardupilot_gazebo \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && sudo make install

# 8. Ortam Değişkenleri (Kalıcı)
ENV PATH="/home/$USERNAME/ardupilot/Tools/autotest:${PATH}"
ENV PATH="/usr/lib/ccache:${PATH}"
ENV GAZEBO_MODEL_PATH="/home/$USERNAME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}"
ENV GAZEBO_RESOURCE_PATH="/home/$USERNAME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}"
ENV OGRE_RESOURCE_PATH="/usr/lib/x86_64-linux-gnu/OGRE-1.9.0/"

CMD ["/bin/bash"]