# === 构建阶段 ===
FROM rust:1.77 as builder

RUN apt-get update && apt-get install -y clang pkg-config libssl-dev

WORKDIR /build
RUN git clone --depth 1 https://gitlab.torproject.org/tpo/core/arti.git
WORKDIR /build/arti
RUN cargo build --release --bin arti

# === 运行阶段 ===
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y ca-certificates && rm -rf /var/lib/apt/lists/*

RUN useradd -m -u 1000 arti
WORKDIR /home/arti

COPY --from=builder /build/arti/target/release/arti /usr/local/bin/arti

RUN mkdir -p /home/arti/.config/arti

# 写入完整的 bridges 配置（包含 obfs4proxy、snowflake、webtunnel 示例）
RUN cat <<'EOF' > /home/arti/.config/arti/arti.toml
[bridges]
enabled = true

# For example:
bridges = '''
192.0.2.83:80 $0bac39417268b96b9f514ef763fa6fba1a788956
[2001:db8::3150]:8080 $0bac39417268b96b9f514e7f63fa6fb1aa788957
obfs4 bridge.example.net:80 $0bac39417268b69b9f514e7f63fa6fba1a788958 ed25519:dGhpcyBpcyBbpmNyZWRpYmx5IHNpbGx5ISEhISEhISA iat-mode=1
snowflake 192.0.2.3:80 2B280B23E1107BB62ABFC40DDCC8824814F80A72 fingerprint=2B280B23E1107BB62ABFC40DDCC8824814F80A72 url=https://snowflake-broker.torproject.net.global.prod.fastly.net/ fronts=foursquare.com,github.githubassets.com ice=stun:stun.l.google.com:19302,stun:stun.antisip.com:3478,stun:stun.bluesip.net:3478,stun:stun.dus.net:3478,stun:stun.epygi.com:3478,stun:stun.sonetel.com:3478,stun:stun.uls.co.za:3478,stun:stun.voipgate.com:3478,stun:stun.voys.nl:3478 utls-imitate=hellorandomizedalpn
webtunnel 192.0.2.3:1 url=https://akbwadp9lc5fyyz0cj4d76z643pxgbfh6oyc-167-71-71-157.sslip.io/5m9yq0j4ghkz0fz7qmuw58cvbjon0ebnrsp0
'''

[[bridges.transports]]
protocols = ["obfs4"]
path = "/usr/bin/obfs4proxy"
#arguments = ["-enableLogging", "-logLevel", "DEBUG"]
arguments = []
run_on_startup = false

[[bridges.transports]]
protocols = ["snowflake"]
path = "/usr/bin/snowflake-client"
#arguments = ["-log-to-state-dir", "-log", "snowflake.log"]
arguments = []
run_on_startup = false

[[bridges.transports]]
protocols = ["webtunnel"]
path = "/usr/bin/webtunnel-client"
arguments = []
run_on_startup = false
EOF

RUN chown -R arti:arti /home/arti/.config/arti

USER arti

CMD ["arti", "-c", "/home/arti/.config/arti/arti.toml"]
