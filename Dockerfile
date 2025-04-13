# 使用带cudnn的官方CUDA基础镜像（指定SHA256校验）
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04@sha256:8f0d09d3ad3900357a1cf7f887888b5b74056636cd6ef03c160c3cd4b1d95

# 统一环境配置
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    # CUDA环境    CUDA_HOME=/usr/local/cuda-11.8 \
    PATH=/usr/local/cuda-11.8/bin:$PATH \
    LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:/usr/local/cuda-11.8/extras/CUPTI/lib64:/usr/local/cuda-11.8/lib64/stubs:$LD_LIBRARY_PATH \
    # 虚拟环境配置
    VENV_PATH=/workspace/nerf-env \
    # 设置默认GPU架构（根据实际设备调整）
    TCNN_CUDA_ARCHITECTURES=61

# 系统层依赖安装（带缓存清理）
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    git cmake ninja-build \
    libboost-dev libeigen3-dev libopenmpi-dev \
    python3-pip python3-venv \
    curl wget gnupg2 lsb-release \
    gcc-10 g++-10 \
    # 图形库依赖
    libgl1-mesa-glx libglfw3 libosmesa6 \
    # CUDA开发工具链
    cuda-toolkit-11-8 cuda-libraries-dev-11-8 \
    && rm -rf /var/lib/apt/lists/*

# 创建Python虚拟环境（带依赖清理）
RUN python3 -m venv $VENV_PATH && \
    $VENV_PATH/bin/pip install --no-cache-dir --upgrade \
    pip==24.0 setuptools==70.0 wheel==0.43.0 && \
    find $VENV_PATH -type d $ -name 'tests' -o -name 'test' $ -exec rm -rf {} +

# 精确安装PyTorch套件
RUN $VENV_PATH/bin/pip install --no-cache-dir \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118 \
    --index-url https://download.pytorch.org/whl/cu118

# 增强版CUDA验证（带设备信息检查）
RUN $VENV_PATH/bin/python -c "\
import torch; \
assert torch.cuda.is_available(), f'''CUDA不可用! 可能原因:
1. 容器运行时未添加--gpus参数
2. 宿主机NVIDIA驱动版本过低（要求≥525.60.13）
3. nvidia-container-toolkit未正确安装
当前环境检查:
- CUDA路径: {torch.utils.cpp_extension.CUDA_HOME}
- 库目录: {torch.utils.cpp_extension.LIBRARY_DIRS}'''; \
print(f'[√] PyTorch {torch.__version__} 验证通过'); \
print(f'|__ CUDA版本: {torch.version.cuda}'); \
print(f'|__ 可用设备: {torch.cuda.device_count()}x {torch.cuda.get_device_name(0)}'); \
print(f'|__ 设备内存: {torch.cuda.get_device_properties(0).total_memory/1024​**​3:.1f}GB')"

# 编译优化tiny-cuda-nn（带架构白名单）
WORKDIR /workspace
RUN git clone --depth 1 --branch v1.7 https://github.com/NVlabs/tiny-cuda-nn.git && \
    cd tiny-cuda-nn && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_CUDA_ARCHITECTURES="$TCNN_CUDA_ARCHITECTURES" \
    -DCMAKE_CXX_COMPILER=g++-10 \
    -DCMAKE_CUDA_COMPILER=$CUDA_HOME/bin/nvcc && \
    cmake --build . --parallel $(nproc) && \
    cd ../bindings && \
    $VENV_PATH/bin/pip install .

# 安装科学计算套件（带版本锁定）
RUN $VENV_PATH/bin/pip install --no-cache-dir \
    nerfstudio==0.3.4 \
    pypose==0.7.3 \
    opencv-python-headless==4.11.0 \
    scipy==1.13.0 \
    numpy==1.26.4 \
    jupyterlab==4.1.6

# 最终系统诊断
RUN $VENV_PATH/bin/python -c "\
import torch, tinycudann, cv2; \
print('\n===== 系统诊断 ====='); \
print(f'PyTorch CUDA: {torch.cuda.is_available()}'); \
print(f'OpenCV CUDA: {cv2.cuda.getCudaEnabledDeviceCount()>0}'); \
print(f'tinycudann: {tinycudann.__version__}'); \
print(f'CUDA内存: {torch.cuda.get_device_properties(0).total_memory/1024​**​3:.1f}GB')"

# 启动配置
COPY nerfstudio_test.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/nerfstudio_test.sh && \
    $VENV_PATH/bin/jupyter labextension install @jupyter-widgets/jupyterlab-manager

CMD ["bash", "-c", "source $VENV_PATH/bin/activate && /usr/local/bin/nerfstudio_test.sh && jupyter-lab --ip=0.0.0.0 --allow-root --no-browser --NotebookApp.token='''"]
