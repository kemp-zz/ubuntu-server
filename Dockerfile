# syntax=docker/dockerfile:1

FROM ghcr.io/linuxserver/baseimage-ubuntu:arm64v8-noble

# 设置Hugging Face镜像源（国内加速），但构建时不使用 huggingface-cli 下载模型
ENV  MODEL_NAME=tiny-int8

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
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        git \
        git-lfs \
        python3-dev \
        python3-venv && \
    if [ -z ${WHISPER_VERSION+x} ]; then \
        WHISPER_VERSION=$(curl -sX GET "https://api.github.com/repos/rhasspy/wyoming-faster-whisper/releases/latest" \
        | awk '/tag_name/{print $4;exit}' FS='[""]'); \
    fi && \
    python3 -m venv /lsiopy && \
    /lsiopy/bin/pip install -U --no-cache-dir pip wheel && \
    /lsiopy/bin/pip install -U --no-cache-dir --find-links https://wheel-index.linuxserver.io/ubuntu/ \
        git+https://github.com/rhasspy/wyoming-faster-whisper@${WHISPER_VERSION} && \
    echo "**** cleanup ****" && \
    apt-get purge -y --auto-remove build-essential git git-lfs python3-dev && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# 这里假设你已经把模型文件放在 root/config/rhasspy/tiny-int8/ 下，直接复制到镜像 /config 目录
COPY root/config/ /config/



VOLUME /config

EXPOSE 10300
