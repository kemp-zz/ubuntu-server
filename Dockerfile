# 使用官方 Python 3.11 slim Bullseye 镜像，支持多架构包括 ARM64
FROM python:3.11-slim-bullseye

ENV DEBIAN_FRONTEND=noninteractive
ENV TMPDIR=/run/whisper-temp

# 安装必要依赖（包括音频处理库和编译工具）
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

# 升级 pip、wheel 并安装 faster-whisper
RUN pip install --no-cache-dir --upgrade pip wheel
RUN pip install --no-cache-dir faster-whisper

# 预先下载并缓存 tiny 模型，启用 int8 量化
RUN python3 -c "\
from faster_whisper import WhisperModel; \
model = WhisperModel('tiny', device='cpu', compute_type='int8'); \
print('模型下载并缓存完成')"

# 暴露配置目录，方便挂载配置和模型
VOLUME /config

# 暴露端口（根据需要修改）
EXPOSE 10300

# Docker 健康检查（前提是容器内有监听 10300 端口的服务响应 /health）
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:10300/health || exit 1

# 默认启动 bash，方便调试和运行
CMD ["/bin/bash"]
