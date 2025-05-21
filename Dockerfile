FROM python:3.11-slim-bullseye

ENV DEBIAN_FRONTEND=noninteractive
ENV TMPDIR=/run/whisper-temp
ENV HF_HOME=/data/huggingface 
ENV HF_HUB_OFFLINE=1         

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --upgrade pip wheel
RUN pip install --no-cache-dir faster-whisper wyoming-faster-whisper huggingface_hub

# 构建时下载模型到 /data/tiny 目录
RUN python3 -c "\
from huggingface_hub import snapshot_download; \
snapshot_download('Systran/faster-whisper-tiny', local_dir='/data/tiny', local_files_only=False)"

VOLUME /config
VOLUME /data

EXPOSE 10300

HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:10300/health || exit 1

CMD ["python3", "-m", "wyoming_faster_whisper", \
     "--model", "/data/tiny", \
     "--device", "cpu", \
     "--compute_type", "float32", \
     "--language", "zh", \
     "--uri", "tcp://0.0.0.0:10300", \
     "--data-dir", "/data"]
