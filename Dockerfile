# 使用多阶段构建
FROM nvidia/cuda:12.8.0-runtime-ubuntu22.04 AS builder

# 设置环境变量
ENV PATH="/usr/local/cuda-12.8/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH}"
ENV NVIDIA_DRIVER_CAPABILITIES="compute,utility"

# 最终阶段
FROM builder

# 添加您的应用程序代码和配置
# ...

# 运行您的应用程序
# CMD ["your_app"]
