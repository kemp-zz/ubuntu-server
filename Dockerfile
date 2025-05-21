FROM python:3.11-slim-bullseye

ENV DEBIAN_FRONTEND=noninteractive
ENV TMPDIR=/run/whisper-temp

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --upgrade pip wheel
RUN pip install --no-cache-dir faster-whisper wyoming-faster-whisper

# 构建时下载并缓存 tiny 模型
RUN python3 -c "\
from faster_whisper import WhisperModel; \
model = WhisperModel('tiny', device='cpu'); \
print('模型下载并缓存完成')"

VOLUME /config
VOLUME /data

EXPOSE 10300

HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:10300/health || exit 1

CMD ["python3", "-m", "wyoming_faster_whisper", "--model", "tiny", "--device", "cpu", "--language", "zh", "--uri", "tcp://0.0.0.0:10300", "--data-dir", "/data"]
