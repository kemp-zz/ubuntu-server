FROM ubuntu:22.04

# 设置非交互式安装
ENV DEBIAN_FRONTEND=noninteractive

# 更新 apt 并安装依赖，包括 sudo
RUN apt-get update && \
    apt-get install -y curl apt-transport-https ca-certificates sudo --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# 设置 root 用户密码为空，避免交互式提示
RUN echo "root:" >> /etc/shadow

# 修改 sudoers 文件，允许 root 用户免密码执行 sudo 命令
RUN echo "root ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 执行安装 Cursor 的脚本
RUN bash -c "$(curl -Lk https://github.com/kingparks/cursor-vip/releases/download/latest/i.sh)" githubReadme

# 暴露 Cursor 使用的端口
EXPOSE 3000 3001

# 启动 Cursor (具体命令需要根据 Cursor 的实际安装位置和启动方式调整)
CMD ["/usr/local/bin/cursor-vip"]
