#!/bin/bash

retry() {
    local i=1
    while true; do
        echo "Attempt $i: Running command $*"
        $* && break || {
            if [[ $i -eq 5 ]]; then
                echo "Command failed after 5 attempts."
                return 1
            fi
            i=$((i + 1))
            echo "Command failed. Waiting 5 seconds before retrying..."
            sleep 5
        }
    done
}

retry wget -qO - https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub | gpg --batch --dearmor -o /usr/share/keyrings/cuda-archive-keyring.gpg


chmod 600 /usr/share/keyrings/cuda-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/cuda-archive-keyring.gpg arch=amd64] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /" > /etc/apt/sources.list.d/cuda.list
