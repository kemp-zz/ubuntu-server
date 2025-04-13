# 使用精确版本的基础镜像（包含完整CUDA工具链）
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04@sha256:8f9dd0d09d3ad3900357a1cf7f887888b5b74056636cd6ef03c160c3cd4b1d95

# 统一环境变量配置
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    # CUDA环境配置
    CUDA_HOME=/usr/local/cuda-11.8 \
    PATH=/usr/local/cuda-11.8/bin:$PATH \
    LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:/usr/local/cuda-11.8/extras/CUPTI/lib64:/usr/local/cuda-11.8/lib64/stubs:$LD_LIBRARY_PATH \
    # Python虚拟环境路径
    VENV_PATH=/workspace/nerf-env

# 系统层依赖安装（优化缓存机制）
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    git \
    cmake \
    ninja-build \
    libboost-dev \
    libeigen3-dev \
    libopenmpi-dev \
    python3-pip \
    python3-venv \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    gcc-10 g++-10 \
    # 补充缺失的图形库依赖
    libgl1-mesa-glx \
    libglfw3 \
    libosmesa6 \
    # 确保CUDA工具链完整
    cuda-toolkit-11-8 \
    cuda-libraries-dev-11-8 \
    && rm -rf /var/lib/apt/lists/*

# 创建Python虚拟环境（增加清理步骤）
RUN python3 -m venv $VENV_PATH && \
    $VENV_PATH/bin/pip install --no-cache-dir --upgrade pip setuptools wheel && \
    find $VENV_PATH -type d -name 'tests' -exec rm -rf {} + && \
    find $VENV_PATH -type d -name 'test' -exec rm -rf {} +

# 精确安装PyTorch及其依赖（使用官方推荐安装方式）
RUN $VENV_PATH/bin/pip install --no-cache-dir \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118 \
    --index-url https://download.pytorch.org/whl/cu118

# 增强版CUDA验证（增加设备信息检查）
RUN $VENV_PATH/bin/python -c "\
import torch; \
try: \
    assert torch.cuda.is_available(), 'CUDA不可用! 检查项:\n1. 容器运行时是否添加--gpus参数\n2. 宿主机驱动版本是否≥525.60.13\n3. nvidia-container-toolkit是否正确安装'; \
    print(f'[√] PyTorch {torch.__version__} 验证通过'); \
    print(f'|__ CUDA版本: {torch.version.cuda}'); \
    print(f'|__ 设备数量: {torch.cuda.device_count()}'); \
    print(f'|__ 当前设备: {torch.cuda.get_device_name(0)}'); \
except Exception as e: \
    print(f'[×] 验证失败: {str(e)}'); \
    print('调试信息:'); \
    print(f'CUDA路径: {torch.utils.cpp_extension.CUDA_HOME}'); \
    print(f'LD路径: {torch.utils.cpp_extension.LIBRARY_DIRS}'); \
    raise SystemExit(1)"

# 编译优化tiny-cuda-nn（增加架构白名单）
WORKDIR /workspace
RUN git clone --depth 1 --branch v1.7 https://github.com/NVlabs/tiny-cuda-nn.git && \
    cd tiny-cuda-nn && \
    mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_CUDA_ARCHITECTURES="61" \ 
    -DCMAKE_CXX_COMPILER=g++-10 \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc && \
    cmake --build . --parallel $(nproc) && \
    cd ../bindings && \
    $VENV_PATH/bin/pip install .

# 安装其他科学计算库（增加版本锁定）
RUN $VENV_PATH/bin/pip install --no-cache-dir \
    jupyterlab==4.1.6 \
    nerfstudio==0.3.4 \
    pypose==0.7.3 \
    opencv-python-headless==4.11.0 \
    # 补充数学库
    scipy==1.13.0 \
    numpy==1.26.4

# 最终系统检查（增加CUDA功能验证）
RUN $VENV_PATH/bin/python -c "\
import torch, tinycudann, cv2; \
print('\n===== 系统诊断 ====='); \
print(f'PyTorch CUDA支持: {torch.cuda.is_available()}'); \
print(f'OpenCV CUDA支持: {cv2.cuda.getCudaEnabledDeviceCount() > 0}'); \
print(f'tinycudann版本: {tinycudann.__version__}'); \
print(f'CUDA设备内存: {torch.cuda.get_device_properties(0).total_memory/1024​**​3:.2f}GB')"

# 启动配置优化（增加Jupyter Lab扩展）
COPY nerfstudio_test.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/nerfstudio_test.sh && \
    $VENV_PATH/bin/jupyter labextension install @jupyter-widgets/jupyterlab-manager

CMD ["bash", "-c", "source $VENV_PATH/bin/activate && /usr/local/bin/nerfstudio_test.sh && jupyter-lab --ip=0.0.0.0 --allow-root --no-browser --NotebookApp.token=''"]
