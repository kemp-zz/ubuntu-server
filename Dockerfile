# syntax=docker/dockerfile:1

FROM ghcr.io/linuxserver/baseimage-ubuntu:arm64v8-noble

ENV MODEL_NAME=tiny

ARG BUILD_DATE
ARG VERSION
ARG WHISPER_VERSION
LABEL build_version="Linuxserver.io version:- ${VERSION} Build-date:- ${BUILD_DATE}"
LABEL maintainer="thespad"

ENV HOME=/config \
    DEBIAN_FRONTEND="noninteractive" \
    TMPDIR="/run/whisper-temp"

# 安装必要依赖（官方源）
RUN echo "**** install packages ****" && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-venv \
        curl

# 创建 Python 虚拟环境
RUN python3 -m venv /lsiopy

# 升级 pip、wheel 并安装 huggingface_hub
RUN /lsiopy/bin/pip install --upgrade pip wheel huggingface_hub

# （可选）提前下载 whisper tiny 模型，通常不需要，whisper 库会自动下载
RUN echo "**** download whisper tiny model (optional) ****" && \
    /lsiopy/bin/python3 -c "import whisper; whisper.load_model('tiny')"

# 清理缓存，减小镜像体积
RUN echo "**** cleanup ****" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

VOLUME /config

EXPOSE 10300

CMD ["/bin/bash"]
