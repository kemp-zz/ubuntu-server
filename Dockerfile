FROM alpine:latest AS builder

RUN apk add --no-cache wget curl tar unzip

WORKDIR /mihomo-config

# 1. 复制本地 config.yaml
COPY config.yaml ./config.yaml

# 2. 下载所有 rule-providers 文件
RUN mkdir -p rules && \
    wget -O rules/download.yaml https://github.com/666OS/YYDS/raw/main/mihomo/rules/download.yaml && \
    wget -O rules/fix-direct.yaml https://github.com/666OS/YYDS/raw/main/mihomo/rules/fix-direct.yaml && \
    wget -O rules/XPTV.yaml https://github.com/666OS/YYDS/raw/main/mihomo/rules/XPTV.yaml && \
    wget -O rules/telegram.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/telegram.mrs && \
    wget -O rules/category-ai-!cn.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/category-ai-!cn.mrs && \
    wget -O rules/geolocation-!cn.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/geolocation-!cn.mrs && \
    wget -O rules/youtube.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/youtube.mrs && \
    wget -O rules/spotify.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/spotify.mrs && \
    wget -O rules/netflix.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/netflix.mrs && \
    wget -O rules/disney.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/disney.mrs && \
    wget -O rules/hbo.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/hbo.mrs && \
    wget -O rules/connectivity-check.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/connectivity-check.mrs && \
    wget -O rules/private.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/private.mrs && \
    wget -O rules/cn.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/cn.mrs && \
    wget -O rules/apple-cn.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/apple-cn.mrs && \
    wget -O rules/cn_ip.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/cn_ip.mrs && \
    wget -O rules/private_ip.mrs https://github.com/666OS/YYDS/raw/main/mihomo/rules/private_ip.mrs

# 3. 下载 Mihomo 所需 geo 文件
RUN wget -O geoip.metadb https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.metadb && \
    wget -O geosite.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geosite.dat && \
    wget -O geoip.dat https://fastly.jsdelivr.net/gh/MetaCubeX/meta-rules-dat@release/geoip.dat

# 4. 下载并解压 zashboard UI
RUN mkdir -p ui && \
    wget -O ui/dist.zip https://github.com/Zephyruso/zashboard/releases/latest/download/dist.zip && \
    unzip ui/dist.zip -d ui && \
    rm ui/dist.zip

# 5. 下载所有 icons 图标
RUN mkdir -p icons && \
    wget -O icons/mihomo.png https://github.com/666OS/YYDS/raw/main/mihomo/image/mihomo.png && \
    wget -O icons/AI.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/AI.png && \
    wget -O icons/Telegram_X.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/Telegram_X.png && \
    wget -O icons/DomesticMedia.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/DomesticMedia.png && \
    wget -O icons/Hong_Kong.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/Hong_Kong.png && \
    wget -O icons/Taiwan.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/Taiwan.png && \
    wget -O icons/Singapore.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/Singapore.png && \
    wget -O icons/Japan.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/Japan.png && \
    wget -O icons/Korea.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/Korea.png && \
    wget -O icons/United_States.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/United_States.png && \
    wget -O icons/European_Union.png https://github.com/Koolson/Qure/raw/master/IconSet/Color/European_Union.png

WORKDIR /mihomo

# 6. 下载 mihomo 主程序
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
COPY --from=builder /mihomo-config/rules/ /root/.config/mihomo/rules/
COPY --from=builder /mihomo-config/geoip.metadb /root/.config/mihomo/geoip.metadb
COPY --from=builder /mihomo-config/geosite.dat /root/.config/mihomo/geosite.dat
COPY --from=builder /mihomo-config/geoip.dat /root/.config/mihomo/geoip.dat
COPY --from=builder /mihomo-config/ui/ /root/.config/mihomo/ui/
COPY --from=builder /mihomo-config/icons/ /root/.config/mihomo/icons/
COPY --from=builder /mihomo/mihomo /mihomo

ENTRYPOINT [ "/mihomo" ]
