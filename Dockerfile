# 基础镜像
FROM python:3.12-slim AS base

# 安装系统依赖
RUN apt-get update && \
    apt-get install -y curl git build-essential && \
    rm -rf /var/lib/apt/lists/*

# 安装 Node.js (使用 nvm)
ENV NVM_DIR /root/.nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash && \
    . "$NVM_DIR/nvm.sh" && nvm install node && nvm use node && \
    ln -s $NVM_DIR/versions/node/$(ls $NVM_DIR/versions/node)/bin/node /usr/bin/node && \
    ln -s $NVM_DIR/versions/node/$(ls $NVM_DIR/versions/node)/bin/npm /usr/bin/npm && \
    ln -s $NVM_DIR/versions/node/$(ls $NVM_DIR/versions/node)/bin/yarn /usr/bin/yarn

# ---- 复制uv二进制 ----
FROM base AS builder
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# 设置工作目录
WORKDIR /app

# 复制代码
COPY . .

# 创建虚拟环境并安装Python依赖
RUN uv venv --python=3.12 .venv && \
    . .venv/bin/activate && \
    uv sync --all-extras

# 构建前端
WORKDIR /app/frontend
RUN npm install -g gatsby-cli && \
    npm install --global yarn && \
    yarn install && \
    yarn build

# 切回主目录
WORKDIR /app

# 启动命令
CMD [".venv/bin/python", "-m", "magentic", "ui", "--port", "8081"]
