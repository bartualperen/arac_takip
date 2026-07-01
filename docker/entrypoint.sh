#!/bin/bash

# Çevre Değişkenlerini Ayarla
export PATH=$PATH:/ardupilot/Tools/autotest
export PATH=$PATH:/root/.local/bin
export GZ_SIM_SYSTEM_PLUGIN_PATH=/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/gz_ws/src/ardupilot_gazebo/models:/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RENDER_ENGINE_BACKEND=ogre2
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH

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
if ! grep -q "usr/lib/python3/dist-packages" "$BASHRC" 2>/dev/null; then
    echo 'export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH' >> "$BASHRC"
fi

# droneuser kullanıcısına geç, env değişkenlerini aktar (-E)
exec sudo -E -u droneuser bash
