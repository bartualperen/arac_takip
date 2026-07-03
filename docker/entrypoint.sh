#!/bin/bash

# Çevre Değişkenlerini Ayarla
export PATH=$PATH:/ardupilot/Tools/autotest
export PATH=$PATH:/root/.local/bin
export GZ_SIM_SYSTEM_PLUGIN_PATH=/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/gz_ws/src/ardupilot_gazebo/models:/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RENDER_ENGINE_BACKEND=ogre2

# Gazebo/ROS Python binding'leri sistem paketlerinde, ML paketleri venv'de.
# PYTHONPATH sistem paketlerini en one alirsa Torch eski Ubuntu sympy'sine takilir.
export PYTHONPATH=/opt/venv/lib/python3.10/site-packages:/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}

echo "Checking ROCm GPU..."

rocminfo | grep gfx || echo "ROCm GPU not detected"

python3 - <<EOF
import torch
print("PyTorch GPU:", torch.cuda.is_available())
EOF

# Ardupilot Gazebo Plugin'ini rebuild et
echo "Building ArduPilot Gazebo Plugin..."
cd /gz_ws/src/ardupilot_gazebo
if [ -d "build" ]; then
    rm -rf build
fi
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
make install

echo "ArduPilot Gazebo Plugin built successfully!"
echo "GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"

# droneuser .bashrc'sine PYTHONPATH ekle (wx ve sistem paketleri için)
BASHRC=/home/droneuser/.bashrc
if [ -f "$BASHRC" ]; then
    sed -i '/export PYTHONPATH=\/usr\/lib\/python3\/dist-packages:\$PYTHONPATH/d' "$BASHRC"
    sed -i '/export PYTHONPATH=\/opt\/venv\/lib\/python3.10\/site-packages:\/usr\/lib\/python3\/dist-packages/d' "$BASHRC"
fi
echo 'export PYTHONPATH=/opt/venv/lib/python3.10/site-packages:/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}' >> "$BASHRC"

# droneuser kullanıcısına geç, env değişkenlerini aktar (-E)
exec sudo -E -u droneuser bash
