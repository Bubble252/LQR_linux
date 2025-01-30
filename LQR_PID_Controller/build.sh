#!/bin/bash

# 定义 build 目录
BUILD_DIR="build"

# 删除已有的 build 目录及其内容
if [ -d "$BUILD_DIR" ]; then
  echo "清理旧的编译文件..."
  rm -rf "$BUILD_DIR"
fi

# 创建新的 build 目录并进入该目录
echo "创建新的编译目录..."
mkdir "$BUILD_DIR"
cd "$BUILD_DIR" || exit

# 运行 cmake 和 make
echo "运行 cmake 和 make..."
cmake ..
make

# 检查是否成功编译
if [ -f "./LQR_PID_Controller" ]; then
    echo "编译完成，正在运行 MyProject..."
    ./LQR_PID_Controller
else
    echo "编译失败，无法运行 MyProject。"
fi

