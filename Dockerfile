# syntax=docker/dockerfile:1

# 阶段1：构建阶段 (Builder)
FROM ghcr.io/linuxserver/baseimage-ubuntu:arm64v8-noble AS builder

ARG BUILD_DATE
ARG VERSION
ARG WHISPER_VERSION
ENV HOME=/config \
    MODEL_NAME=tiny-int8 \
    TMPDIR="/run/whisper-temp"

RUN echo "**** 安装构建依赖 ****" && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        git \
        git-lfs \
        python3-dev \
        python3-venv && \
    # 动态获取WHISPER版本
    if [ -z ${WHISPER_VERSION+x} ]; then \
        WHISPER_VERSION=$(curl -sX GET "https://api.github.com/repos/rhasspy/wyoming-faster-whisper/releases/latest" \
        | awk '/tag_name/{print $4;exit}' FS='[""]'); \
    fi && \
    # 创建虚拟环境
    python3 -m venv /lsiopy && \
    /lsiopy/bin/pip install -U --no-cache-dir \
        pip \
        wheel && \
    # 安装项目依赖
    /lsiopy/bin/pip install -U --no-cache-dir --find-links https://wheel-index.linuxserver.io/ubuntu/ \
        git+https://github.com/rhasspy/wyoming-faster-whisper@${WHISPER_VERSION} && \
    # 下载语音模型
    /lsiopy/bin/huggingface-cli download --resume-download --local-dir /config/rhasspy/${MODEL_NAME} \
        rhasspy/faster-whisper-${MODEL_NAME}

# 阶段2：运行阶段 (Runtime)
FROM ghcr.io/linuxserver/baseimage-ubuntu:arm64v8-noble

LABEL maintainer="thespad"

# 从构建阶段复制产物
COPY --from=builder /lsiopy /lsiopy
COPY --from=builder /config/rhasspy /config/rhasspy

# 设置环境变量
ENV PATH="/lsiopy/bin:$PATH" \
    HOME=/config \
    TMPDIR="/run/whisper-temp"

# 声明端口和存储卷
EXPOSE 10300
VOLUME /config

# 复制启动脚本
COPY root/ /
