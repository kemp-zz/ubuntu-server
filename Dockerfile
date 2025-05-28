FROM alpine:latest as builder
ARG TARGETPLATFORM
RUN echo "I'm building for $TARGETPLATFORM"

# 下载规则文件
RUN apk add --no-cache gzip && \
    mkdir /mihomo-config && \
    wget -O /mihomo-config/geoip.metadb https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.metadb && \
    wget -O /mihomo-config/geosite.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geosite.dat && \
    wget -O /mihomo-config/geoip.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.dat

# 下载指定预发布二进制
ARG MIHOMO_VERSION="mihomo-linux-arm64-alpha-0a05fb2"
RUN wget -O /mihomo/mihomo.gz https://github.com/vernesong/mihomo/releases/download/Prerelease-Alpha/${MIHOMO_VERSION}.gz

# 下载 Model.bin 文件
RUN wget -O /mihomo-config/Model.bin https://github.com/vernesong/mihomo/releases/download/LightGBM-Model/Model.bin

WORKDIR /mihomo

# 解压并处理二进制文件
RUN gzip -d mihomo.gz && \
    chmod +x mihomo && \
    echo "${MIHOMO_VERSION}" > /mihomo-config/test

FROM alpine:latest
LABEL org.opencontainers.image.source="https://github.com/MetaCubeX/mihomo"

RUN apk add --no-cache ca-certificates tzdata iptables

VOLUME ["/root/.config/mihomo/"]

COPY --from=builder /mihomo-config/ /root/.config/mihomo/
COPY --from=builder /mihomo/mihomo /mihomo
ENTRYPOINT [ "/mihomo" ]
