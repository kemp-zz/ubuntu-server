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

# === Runtime Stage ===
FROM alpine:3.22.0

RUN apk --no-cache --no-progress update && \
    apk --no-cache --no-progress add \
    curl \
    sqlite-libs \
    libgcc \
    tini

# Install obfs4proxy, snowflake and webtunnel
RUN wget -O /usr/bin/snowflake-client https://gitlab.torproject.org/tpo/anti-censorship/pluggable-transports/snowflake/-/releases/permalink/latest/download/snowflake-client-linux-amd64 && \
    chmod +x /usr/bin/snowflake-client

RUN wget -O /usr/bin/webtunnel-client https://gitlab.torproject.org/tpo/anti-censorship/pluggable-transports/webtunnel/-/releases/permalink/latest/download/webtunnel-client-linux-amd64 && \
    chmod +x /usr/bin/webtunnel-client || echo "webtunnel-client not installed (no release found)"

RUN wget -O /usr/bin/obfs4proxy https://gitlab.torproject.org/tpo/anti-censorship/pluggable-transports/obfs4/-/releases/permalink/latest/download/obfs4proxy-linux-amd64 && \
    chmod +x /usr/bin/obfs4proxy

# Create user and set up directories
RUN adduser \
    --disabled-password \
    --home "/home/arti/" \
    --gecos "" \
    --shell "/sbin/nologin" \
    arti

WORKDIR /home/arti

# Copy binary from builder
COPY --from=builder /build/target/release/arti /usr/bin/arti

# Set up config directory
RUN mkdir -p /home/arti/.config/arti/arti.d/ \
    /home/arti/.cache/arti/ \
    /home/arti/.local/share/arti/

# Copy and set up config
COPY --chmod=644 arti.toml /home/arti/.config/arti/arti.d/

# Set proper ownership
RUN chown -R arti:arti /home/arti/

# Switch to non-root user
USER arti

# Add healthcheck
HEALTHCHECK --interval=5m --timeout=15s --start-period=20s \
  CMD curl -s --socks5-hostname localhost:9150 'https://check.torproject.org/' | \
  grep -qm1 Congratulations

# Define volumes for persistent data
VOLUME [ "/home/arti/.cache/arti/", "/home/arti/.local/share/arti/" ]

# Expose SOCKS proxy port
EXPOSE 9150

# Use tini as init
ENTRYPOINT ["/sbin/tini", "--", "arti" ]
CMD [ "proxy" ]
