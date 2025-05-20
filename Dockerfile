# 使用官方 Python 3.11 slim Bullseye 镜像，支持多架构包括 ARM64
FROM python:3.11-slim-bullseye

ENV MODEL_NAME=tiny-int8
ENV DEBIAN_FRONTEND=noninteractive
ENV TMPDIR=/run/whisper-temp

# 安装必要依赖（包括音频处理库和编译工具）
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

# 安装 faster-whisper 及依赖
RUN pip install --no-cache-dir --upgrade pip wheel
RUN pip install --no-cache-dir faster-whisper

# （可选）提前下载 tiny-int8 模型，避免运行时首次下载卡顿
RUN python3 -c "\
from faster_whisper import WhisperModel; \
model = WhisperModel('tiny-int8', device='cpu', compute_type='int8'); \
print('模型下载并缓存完成')"

# 暴露配置目录，方便挂载配置和模型
VOLUME /config

# 暴露端口（根据需要修改）
EXPOSE 10300

# 默认启动 bash，方便调试和运行
CMD ["/bin/bash"]
