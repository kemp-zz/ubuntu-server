# === Build Stage ===
FROM rust:1.87.0-alpine AS builder

RUN apk --no-cache --no-progress update && \
    apk --no-cache --no-progress add \
    musl-dev \
    openssl-dev \
    sqlite-dev \
    git

ENV RUSTFLAGS="-Ctarget-feature=-crt-static"

WORKDIR /build
RUN git clone --depth 1 https://gitlab.torproject.org/tpo/core/arti.git . && \
    cargo build --release --bin arti

# === Build Stage for Go Tools ===
FROM golang:1.22-alpine AS go_builder

RUN apk add --no-cache git
# obfs4proxy
RUN git clone --depth 1 https://gitlab.torproject.org/tpo/anti-censorship/pluggable-transports/obfs4.git /src/obfs4
WORKDIR /src/obfs4
RUN go build -o /go/bin/obfs4proxy ./obfs4proxy

# snowflake-client
RUN git clone --depth 1 https://gitlab.torproject.org/tpo/anti-censorship/pluggable-transports/snowflake.git /src/snowflake
WORKDIR /src/snowflake/client
RUN go build -o /go/bin/snowflake-client

# webtunnel-client
RUN git clone --depth 1 https://gitlab.torproject.org/tpo/anti-censorship/pluggable-transports/webtunnel.git /src/webtunnel
WORKDIR /src/webtunnel/client
RUN go build -o /go/bin/webtunnel-client

# === Runtime Stage ===
FROM alpine:3.22.0

RUN apk --no-cache --no-progress update && \
    apk --no-cache --no-progress add \
    curl \
    sqlite-libs \
    libgcc \
    tini

# 复制 go 工具
COPY --from=go_builder /go/bin/obfs4proxy /usr/bin/obfs4proxy
COPY --from=go_builder /go/bin/snowflake-client /usr/bin/snowflake-client
COPY --from=go_builder /go/bin/webtunnel-client /usr/bin/webtunnel-client

# 创建用户和目录
RUN adduser \
    --disabled-password \
    --home "/home/arti/" \
    --gecos "" \
    --shell "/sbin/nologin" \
    arti

WORKDIR /home/arti

# 复制编译好的 arti
COPY --from=builder /build/target/release/arti /usr/bin/arti

# 创建配置和数据目录
RUN mkdir -p /home/arti/.config/arti/arti.d/ \
    /home/arti/.cache/arti/ \
    /home/arti/.local/share/arti/

# 如果你有配置文件需要复制，取消下一行注释并确保构建上下文有 arti.toml
# COPY --chmod=644 arti.toml /home/arti/.config/arti/arti.d/

# 设置目录权限
RUN chown -R arti:arti /home/arti/

USER arti

HEALTHCHECK --interval=5m --timeout=15s --start-period=20s \
  CMD curl -s --socks5-hostname localhost:9150 'https://check.torproject.org/' | \
  grep -qm1 Congratulations

VOLUME [ "/home/arti/.cache/arti/", "/home/arti/.local/share/arti/" ]
EXPOSE 9150

ENTRYPOINT ["/sbin/tini", "--", "arti" ]
CMD [ "proxy" ]
