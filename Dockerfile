FROM debian:bookworm-slim

ENV LANG=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    WYOMING_PIPER_VERSION=1.5.4 \
    BINARY_PIPER_VERSION=1.2.0 \
    MODEL_VERSION=0.0.2 \
    HF_HUB_OFFLINE=1

WORKDIR /usr/src

# 系统依赖安装（合并层优化）
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash curl ca-certificates python3 python3-pip \
        xz-utils jq netcat-traditional && \
    rm -rf /var/lib/apt/lists/* && \
    useradd -ms /bin/bash appuser

# Python依赖安装
RUN pip3 install --no-cache-dir \
        huggingface_hub \
        -U setuptools wheel "wyoming-piper @ https://github.com/rhasspy/wyoming-piper/archive/refs/tags/v${WYOMING_PIPER_VERSION}.tar.gz"

# 模型与二进制部署
RUN mkdir -p /data/zh_CN-huayan-medium && \
    curl -L -sS "https://github.com/rhasspy/piper/releases/download/v${MODEL_VERSION}/voice-zh_CN-huayan-medium.tar.gz" | \
        tar -zxvf - -C /data/zh_CN-huayan-medium --strip-components=1 && \
    curl -L -sS "https://github.com/rhasspy/piper/releases/download/v${BINARY_PIPER_VERSION}/piper_arm64.tar.gz" | \
        tar -zxvf - -C /usr/share && \
    chmod +x /usr/share/piper/piper && \
    chown -R appuser:appuser /usr/share/piper /data

EXPOSE 10200
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:10200/health || exit 1

USER appuser 

CMD ["python3", "-m", "wyoming_piper", \
  "--piper", "/usr/share/piper/piper", \
  "--voice", "zh_CN-huayan-medium", \
  "--data-dir", "/data/zh_CN-huayan-medium", \
  "--speaker", "0", \
  "--noise-scale", "0.667", \
  "--length-scale", "1.0", \
  "--noise-w", "0.333", \
  "--auto-punctuation", ".?!", \
  "--samples-per-chunk", "1024", \
  "--max-piper-procs", "1", \  
  "--debug", \
  "--log-format", "%(levelname)s:%(name)s:%(message)s"]
