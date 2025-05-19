# syntax=docker/dockerfile:1

FROM ghcr.io/linuxserver/baseimage-ubuntu:arm64v8-noble

# 新增：设置Hugging Face镜像源（国内加速）
ENV     MODEL_NAME=tiny-int8

# 原有配置保持不变
ARG BUILD_DATE
ARG VERSION
ARG WHISPER_VERSION
LABEL build_version="Linuxserver.io version:- ${VERSION} Build-date:- ${BUILD_DATE}"
LABEL maintainer="thespad"

ENV HOME=/config \
    DEBIAN_FRONTEND="noninteractive" \
    TMPDIR="/run/whisper-temp"


RUN \

    echo "**** install packages ****" && \
    echo "**** install packages ****" && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        git \
        git-lfs \
        python3-dev \
        python3-venv \
        && \
    if [ -z ${WHISPER_VERSION+x} ]; then \
        WHISPER_VERSION=$(curl -sX GET "https://api.github.com/repos/rhasspy/wyoming-faster-whisper/releases/latest" \
        | awk '/tag_name/{print $4;exit}' FS='[""]'); \
    fi && \
    python3 -m venv /lsiopy && \
    pip install -U --no-cache-dir \
        pip \
        wheel && \
    pip install -U --no-cache-dir --find-links https://wheel-index.linuxserver.io/ubuntu/ \
        git+https://github.com/rhasspy/wyoming-faster-whisper@${WHISPER_VERSION} && \
    # 新增：下载tiny-int8模型到/config目录
    huggingface-cli download --resume-download --local-dir /config/rhasspy/${MODEL_NAME} \
        rhasspy/faster-whisper-${MODEL_NAME} && \
    printf "Linuxserver.io version: ${VERSION}\nBuild-date: ${BUILD_DATE}" > /build_version && \
    echo "**** cleanup ****" && \
    apt-get purge -y --auto-remove \
        build-essential \
        git \
        git-lfs \
        python3-dev && \
    rm -rf \
        /var/lib/apt/lists/* \
        /tmp/*

# 显式声明端口（已存在则无需修改）
EXPOSE 10300

COPY root/ /
VOLUME /config
