# source:  https://www.youtube.com/watch?v=xqroBkpf3lY
sudo apt update
pip install --upgrade pip

# Install YOLOv8 + PyTorch on Jetson Orin Nano
pip3 install -U python-dateutil
pip3 install ultralytics
pip3 uninstall torch torchvision -y

# Installing PyTorch for Jetson Platform:
# https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
sudo apt install -y libopenblas-dev

export TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

pip install aiohttp
echo "export LD_LIBRARY_PATH=/usr/lib/llvm-8/lib:$LD_LIBRARY_PATH" >> ~/.bashrc

pip install --upgrade protobuf
pip install --no-cache $TORCH_INSTALL


# install torchvision
# https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

sudo apt install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch v0.16.0 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.16.0

export PYTHONPATH=/home/jetson/.local/lib/python3.8/site-packages/:$PYTHONPATH
python3 setup.py install --user --install-dir=/usr/local/lib/python3.8/dist-packages/ --build-type=Release
cd ../
