#!/bin/bash

set -e

# 检查CUDA基础环境
echo "===== CUDA 基础环境验证 ====="
nvidia-smi
NV_STATUS=$?
if [ $NV_STATUS -eq 0 ]; then
  echo "[√] NVIDIA驱动检测 验证通过"
else
  echo "[×] NVIDIA驱动检测 验证失败"
fi

nvcc --version
NVCC_STATUS=$?
if [[ $NVCC_STATUS -eq 0 ]]; then
  echo "[√] CUDA 11.8编译器检测 验证通过"
else
  echo "[×] CUDA 11.8编译器检测 验证失败"
fi

# 检查PyTorch环境
echo "\n===== PyTorch环境验证 ====="
PYTORCH_VERSION=$(python3 -c "import torch; print(torch.__version__)")
echo "PyTorch版本: $PYTORCH_VERSION"

CUDA_AVAIL=$(python3 -c "import torch; print(torch.cuda.is_available())")
echo "CUDA可用性: $CUDA_AVAIL"

CUDA_VERSION=$(python3 -c "import torch; print(torch.version.cuda)")
echo "CUDA版本: $CUDA_VERSION"

if [[ "$CUDA_AVAIL" == "True" ]]; then
  echo "[√] PyTorch环境 验证通过"
else
  echo "[×] PyTorch环境 验证失败"
fi

# 检查3D视觉库
echo "\n===== 3D视觉库验证 ====="
OPENCV_VERSION=$(python3 -c "import cv2; print(cv2.__version__)")
PYPOSE_VERSION=$(python3 -c "import pypose; print(pypose.__version__)")

echo "OpenCV版本: $OPENCV_VERSION"
echo "PyPose版本: $PYPOSE_VERSION"

if [[ ! -z "$OPENCV_VERSION" && ! -z "$PYPOSE_VERSION" ]]; then
  echo "[√] OpenCV/PyPose 验证通过"
else
  echo "[×] OpenCV/PyPose 验证失败"
fi

# 检查NeRF相关库
echo "\n===== NeRF相关库验证 ====="
# NeRFStudio
command -v ns-train >/dev/null 2>&1 || { echo "[×] NeRFStudio安装 验证失败"; return 1; }
echo "[√] NeRFStudio安装 验证通过"

# tiny-cuda-nn
TCNN_STATUS=$(python3 -c "import tinycudann as tcnn; print(tcnn.__version__)" 2>/dev/null)
if [[ ! -z "$TCNN_STATUS" ]]; then
  echo "[√] tiny-cuda-nn 验证通过"
else
  echo "[×] tiny-cuda-nn 验证失败"
fi

# 检查ROS环境
echo "\n===== ROS环境验证 ====="
# 检查ROS是否能正确初始化
ros2 doctor
ROS_STATUS=$?

if [[ $ROS_STATUS -eq 0 ]]; then
  echo "[√] ROS2基础环境 验证通过"
else
  echo "[×] ROS2基础环境 验证失败"
fi

echo "\n测试完成！请根据上述结果检查异常项"
