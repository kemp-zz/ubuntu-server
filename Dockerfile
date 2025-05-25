FROM alpine:latest as builder

RUN apk add --no-cache gzip wget tar

# 下载配置文件
RUN mkdir /mihomo-config && \
    wget -O /mihomo-config/geoip.metadb https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.metadb && \
    wget -O /mihomo-config/geosite.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geosite.dat && \
    wget -O /mihomo-config/geoip.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.dat

WORKDIR /mihomo

# 直接下载 clash-linux-arm64.tar.gz 并解压
RUN wget -O clash-linux-arm64.tar.gz https://github.com/vernesong/OpenClash/releases/download/mihomo/clash-linux-arm64.tar.gz && \
    tar -xzf clash-linux-arm64.tar.gz && \
    # 假设解压后生成的可执行文件名为 clash，重命名为 mihomo
    mv clash mihomo && \
    chmod +x mihomo

FROM alpine:latest
LABEL org.opencontainers.image.source="https://github.com/vernesong/OpenClash/releases/download/mihomo/clash-linux-arm64.tar.gz"

RUN apk add --no-cache ca-certificates tzdata iptables

VOLUME ["/root/.config/mihomo/"]

COPY --from=builder /mihomo-config/ /root/.config/mihomo/
COPY --from=builder /mihomo/mihomo /mihomo

ENTRYPOINT [ "/mihomo" ]
