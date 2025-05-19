# syntax=docker/dockerfile:1

FROM ghcr.io/linuxserver/baseimage-ubuntu:arm64v8-noble

ENV MODEL_NAME=tiny-int8

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
        python3-pip \
        python3-venv \
        curl && \
    python3 -m venv /lsiopy && \
    /lsiopy/bin/pip install --upgrade pip wheel huggingface_hub && \
    echo "**** download tiny-int8 model ****" && \
    /lsiopy/bin/python3 -c "\
import huggingface_hub as hf; \
hf.snapshot_download(repo_id='rhasspy/faster-whisper-tiny-int8', local_dir='/config/rhasspy/tiny-int8')" && \
    echo "**** cleanup ****" && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

VOLUME /config

EXPOSE 10300

CMD ["/bin/bash"]
