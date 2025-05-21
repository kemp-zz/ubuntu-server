FROM debian:bookworm-slim

ENV LANG=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    WYOMING_PIPER_VERSION=1.5.4 \
    BINARY_PIPER_VERSION=1.2.0 \
    MODEL_VERSION=0.0.2

WORKDIR /usr/src

# 安装系统依赖（合并下载层）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash curl ca-certificates python3 python3-pip \
        xz-utils jq netcat-traditional && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir \
        huggingface_hub \
        -U setuptools wheel "wyoming-piper @ https://github.com/rhasspy/wyoming-piper/archive/refs/tags/v${WYOMING_PIPER_VERSION}.tar.gz"

# 模型与二进制下载（使用GitHub镜像源）
RUN mkdir -p /data/zh_CN-huayan-medium && \
    curl -L -sS "https://github.com/rhasspy/piper/releases/download/v${MODEL_VERSION}/voice-zh_CN-huayan-medium.tar.gz" | \
        tar -zxvf - -C /data/zh_CN-huayan-medium --strip-components=1 && \
    curl -L -sS "https://github.com/rhasspy/piper/releases/download/v${BINARY_PIPER_VERSION}/piper_arm64.tar.gz" | \
        tar -zxvf - -C /usr/share

EXPOSE 10200
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:10200/health || exit 1

ENV HF_HUB_OFFLINE=1

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
