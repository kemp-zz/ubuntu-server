FROM alpine:latest as builder

RUN apk add --no-cache wget grep sed awk tar

WORKDIR /mihomo-config

# 复制 config.yaml 到构建目录
COPY docker/config.yaml ./config.yaml

# 1. 提取并下载 proxy-providers
RUN mkdir -p providers && \
    grep -A 10 'proxy-providers:' config.yaml | grep 'url:' | awk '{print $2}' | sed "s/'//g" | while read url; do \
      fname=$(basename "$url"); \
      wget -O providers/"$fname" "$url"; \
      # 替换 config.yaml 中的 url 为本地路径
      sed -i "s|$url|/root/.config/mihomo/providers/$fname|g" config.yaml; \
    done

# 2. 提取并下载 rule-providers
RUN mkdir -p rules && \
    grep -A 10 'rule-providers:' config.yaml | grep 'url:' | awk '{print $2}' | sed "s/'//g" | while read url; do \
      fname=$(basename "$url"); \
      wget -O rules/"$fname" "$url"; \
      sed -i "s|$url|/root/.config/mihomo/rules/$fname|g" config.yaml; \
    done

# 3. 其它 Mihomo 所需文件
RUN wget -O geoip.metadb https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.metadb && \
    wget -O geosite.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geosite.dat && \
    wget -O geoip.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.dat

WORKDIR /mihomo

RUN wget -O clash-linux-arm64.tar.gz https://github.com/vernesong/OpenClash/releases/download/mihomo/clash-linux-arm64.tar.gz && \
    tar -xzf clash-linux-arm64.tar.gz && \
    mv clash mihomo && \
    chmod +x mihomo

FROM alpine:latest
LABEL org.opencontainers.image.source="https://github.com/vernesong/OpenClash/releases/download/mihomo/clash-linux-arm64.tar.gz"

RUN apk add --no-cache ca-certificates tzdata iptables

VOLUME ["/root/.config/mihomo/"]

COPY --from=builder /mihomo-config/config.yaml /root/.config/mihomo/config.yaml
COPY --from=builder /mihomo-config/providers/ /root/.config/mihomo/providers/
COPY --from=builder /mihomo-config/rules/ /root/.config/mihomo/rules/
COPY --from=builder /mihomo-config/geoip.metadb /root/.config/mihomo/geoip.metadb
COPY --from=builder /mihomo-config/geosite.dat /root/.config/mihomo/geosite.dat
COPY --from=builder /mihomo-config/geoip.dat /root/.config/mihomo/geoip.dat
COPY --from=builder /mihomo/mihomo /mihomo

ENTRYPOINT [ "/mihomo" ]
