# CMake Project
Example CMake Project to integrate Phase C++ library.

## Dependencies
Phase library is required to be installed for use in the build process.  
### Linux
Download debian package from latest release.  
Install debian package using apt package manager:
```
sudo apt install -f ./phase_vx.x.x-amd64.deb
```
This should install to `/opt/i3dr/phase`
### Windows
Download Windows installer from the latest release.  
Install using the installer GUI, this should install to `C:\Program Files\i3DR\Phase`

## Build
Build CMake project:
```bash
mkdir build
cd build
# [windows]
cmake -G "Visual Studio 16 2019" -A x64 -DPhase_DIR="C:\Program Files\i3DR\Phase\lib\cmake" ..
cmake --build . --config Release
# [linux]
cmake -DPhase_DIR="/opt/i3dr/phase/lib/cmake" ..
make -j$(nproc)
```
*Note: Make sure to run this from the repository root directory*

### Run
To run the sample application, use the following commands:
```bash
# [windows]
PATH=/c/Program\ Files/i3DR/Phase/install/bin:$PATH
./build/bin/phase_sample
# [linux]
INIT_LD_LIBRARY_PATH=$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/i3dr/phase/lib:/opt/i3dr/phase/lib/i3drsgm:$LD_LIBRARY_PATH
./build/bin/phase_sample
export LD_LIBRARY_PATH=$INIT_LD_LIBRARY_PATH
```

*Note: Make sure to run this from the repository root directory*