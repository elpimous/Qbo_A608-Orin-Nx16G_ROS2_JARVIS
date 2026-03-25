pour la wheel / binaire
-----------------------

git clone https://github.com/k2-fsa/sherpa-onnx
cd sherpa-onnx
mkdir build
cd build

cmake \
  -DSHERPA_ONNX_LINUX_ARM64_GPU_ONNXRUNTIME_VERSION=1.18.1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DSHERPA_ONNX_ENABLE_GPU=ON \
  ..

make


for developper
---------------

git clone https://github.com/k2-fsa/sherpa-onnx
cd sherpa-onnx

wget https://github.com/csukuangfj/onnxruntime-libs/releases/download/v1.22.0/onnxruntime-linux-x64-gpu-1.22.0-patched.zip
unzip  onnxruntime-linux-x64-gpu-1.22.0-patched.zip

export SHERPA_ONNXRUNTIME_LIB_DIR=$PWD/onnxruntime-linux-x64-gpu-1.22.0-patched/lib
export SHERPA_ONNXRUNTIME_INCLUDE_DIR=$PWD/onnxruntime-linux-x64-gpu-1.22.0-patched/include

mkdir build
cd build

cmake \
  -DSHERPA_ONNX_ENABLE_PYTHON=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DSHERPA_ONNX_ENABLE_CHECK=OFF \
  -DSHERPA_ONNX_ENABLE_PORTAUDIO=OFF \
  -DSHERPA_ONNX_ENABLE_C_API=OFF \
  -DSHERPA_ONNX_ENABLE_WEBSOCKET=OFF \
  -DSHERPA_ONNX_ENABLE_GPU=ON \
  ..

make -j 4
export PYTHONPATH=$PWD/../sherpa-onnx/python/:$PWD/lib:$PYTHONPATH
mkdir -p ../sherpa-onnx/python/sherpa_onnx/lib/
cp -v lib/_sherpa_onnx* ../sherpa-onnx/python/sherpa_onnx/lib/