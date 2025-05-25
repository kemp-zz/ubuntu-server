FROM alpine:latest AS builder

RUN apk add --no-cache wget grep sed curl tar unzip

WORKDIR /mihomo-config

# 1. 下载 config.yaml
RUN wget -O ./config.yaml https://raw.githubusercontent.com/kemp-zz/ubuntu-server/main/config.yaml

# 2. 下载 proxy-providers 并替换 config.yaml 路径
RUN mkdir -p providers && \
    grep -A 20 'proxy-providers:' config.yaml | grep 'url:' | awk '{print $2}' | sed "s/'//g" | while read url; do \
      fname=$(basename "$url"); \
      wget -O providers/"$fname" "$url"; \
      sed -i "s|$url|/root/.config/mihomo/providers/$fname|g" config.yaml; \
    done

# 3. 下载 rule-providers 并替换 config.yaml 路径
RUN mkdir -p rules && \
    grep -A 20 'rule-providers:' config.yaml | grep 'url:' | awk '{print $2}' | sed "s/'//g" | while read url; do \
      fname=$(basename "$url"); \
      wget -O rules/"$fname" "$url"; \
      sed -i "s|$url|/root/.config/mihomo/rules/$fname|g" config.yaml; \
    done

# 4. 下载 Mihomo 所需 geo 文件
RUN wget -O geoip.metadb https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.metadb && \
    wget -O geosite.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geosite.dat && \
    wget -O geoip.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.dat

# 5. 下载并解压 zashboard UI
RUN mkdir -p ui && \
    wget -O ui/dist.zip https://github.com/Zephyruso/zashboard/releases/latest/download/dist.zip && \
    unzip ui/dist.zip -d ui && \
    rm ui/dist.zip

# 6. 写入 external-ui 字段到 config.yaml（如果已存在则替换，没有则追加）
RUN grep -q '^external-ui:' config.yaml && \
    sed -i '/^external-ui:/c\external-ui: /root/.config/mihomo/ui' config.yaml || \
    echo 'external-ui: /root/.config/mihomo/ui' >> config.yaml

WORKDIR /mihomo

# 7. 下载 mihomo 主程序
RUN wget -O clash-linux-arm64.tar.gz https://github.com/vernesong/OpenClash/releases/download/mihomo/clash-linux-arm64.tar.gz && \
    tar -xzf clash-linux-arm64.tar.gz && \
    mv clash mihomo && \
    chmod +x mihomo

# --------- 生产环境镜像 ---------
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
COPY --from=builder /mihomo-config/ui/ /root/.config/mihomo/ui/
COPY --from=builder /mihomo/mihomo /mihomo

ENTRYPOINT [ "/mihomo" ]
