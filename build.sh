# 清理构建
rmdir build 2>/dev/null || true

# 创建build目录
mkdir -p build
cd build

# CMake配置
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
make -j$(nproc)

# 运行
./run_slam