# Installation

```
sudo apt-get update
sudo apt-get upgrade

sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libxcb-cursor0
sudo apt-get install python3-dev
sudo apt install cython3
sudo apt install build-essential
sudo apt install libc6-dev
sudo apt install libgl1-mesa-dev
sudo apt-get install make
sudo apt-get install cmake 
sudo apt-get install python3-opencv 
pip install "numpy<1.24"
pip install --upgrade cython


git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build
cd build

cmake .. -DBUILD_PYTHON3=ON -DBUILD_EXAMPLES=OFF \
    -DPYTHON_INCLUDE_DIR=$(python3 -c "import sysconfig; print(sysconfig.get_path('include'))")  \
    -DPYTHON_LIBRARY=$(python3 -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")

sudo make install
# Si se instala en otro directorio añade al path (mirar los últimos prints del make install):
export PYTHONPATH=/usr/local/lib/python3/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```