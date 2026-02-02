#!/bin/bash

# Çevre Değişkenlerini Ayarla
export PATH=$PATH:/ardupilot/Tools/autotest
export PATH=$PATH:/root/.local/bin
export GZ_SIM_SYSTEM_PLUGIN_PATH=/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/gz_ws/src/ardupilot_gazebo/models:/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RENDER_ENGINE_BACKEND=ogre2

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

# droneuser kullanıcısına geç ve shell başlat
exec sudo -u droneuser bash
