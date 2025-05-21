FROM debian:bookworm-slim

ARG HF_TOKEN
ENV HF_TOKEN=${HF_TOKEN}

ENV LANG=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    WYOMING_PIPER_VERSION=1.5.4 \
    BINARY_PIPER_VERSION=1.2.0

WORKDIR /usr/src

# 安装系统依赖并安装 Python 包
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash curl ca-certificates python3 python3-pip \
        xz-utils jq netcat-traditional && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir \
        huggingface_hub \
        -U setuptools wheel "wyoming-piper @ https://github.com/rhasspy/wyoming-piper/archive/refs/tags/v${WYOMING_PIPER_VERSION}.tar.gz"

# 下载模型并解压 Piper 二进制
RUN mkdir -p /data/zh_CN-huayan-medium && \
    python3 -c "from huggingface_hub import snapshot_download; \
    snapshot_download(repo_id='rhasspy/huayan', \
        local_dir='/data/zh_CN-huayan-medium', \
        local_dir_use_symlinks=False, \
        resume_download=True)" && \
    mkdir -p /usr/share && \
    curl -L -sS "https://github.com/rhasspy/piper/releases/download/v${BINARY_PIPER_VERSION}/piper_arm64.tar.gz" | \
    tar -zxvf - -C /usr/share

# 暴露端口和健康检查
EXPOSE 10200
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:10200/health || exit 1

# 运行时设置离线环境变量，防止联网（根据需要开启或关闭）
ENV HF_HUB_OFFLINE=1

WORKDIR /

CMD ["python3", "-m", "wyoming_piper", \
     "--model-dir", "/data/zh_CN-huayan-medium", \
     "--speaker", "0", \
     "--noise-scale", "0.667", \
     "--length-scale", "1.0", \
     "--noise-w", "0.333", \
     "--auto-punctuation", ".?!", \
     "--samples-per-chunk", "1024", \
     "--max-piper-procs", "1", \
     "--update-voices", "True", \
     "--debug", "True", \
     "--log-format", "%(levelname)s:%(name)s:%(message)s"]
