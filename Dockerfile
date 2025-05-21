FROM debian:bookworm-slim

ARG HF_TOKEN
ENV HF_TOKEN=${HF_TOKEN}

ENV LANG=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    WYOMING_PIPER_VERSION=1.5.4 \
    BINARY_PIPER_VERSION=1.2.0

WORKDIR /usr/src

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash curl ca-certificates python3 python3-pip xz-utils jq netcat-traditional \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir huggingface_hub && \
    pip3 install --no-cache-dir -U setuptools wheel && \
    pip3 install --no-cache-dir "wyoming-piper @ https://github.com/rhasspy/wyoming-piper/archive/refs/tags/v${WYOMING_PIPER_VERSION}.tar.gz"

RUN mkdir -p /data/zh_CN-huayan-medium

RUN python3 -c "\
from huggingface_hub import snapshot_download; \
snapshot_download(repo_id='rhasspy/huayan', cache_dir='/data/zh_CN-huayan-medium', local_dir_use_symlinks=False)"

RUN mkdir -p /usr/share

RUN curl -L -s "https://github.com/rhasspy/piper/releases/download/v${BINARY_PIPER_VERSION}/piper_arm64.tar.gz" | tar -zxvf - -C /usr/share

EXPOSE 10200

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
