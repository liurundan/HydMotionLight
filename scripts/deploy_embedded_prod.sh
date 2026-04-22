#!/bin/bash
# ==================================================================
# 嵌入式部署脚本 (Embedded Deployment Script)
# 用于生成不包含仿真器的生产版本库文件
# ==================================================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ==================================================================
# 函数定义
# ==================================================================

print_header() {
    echo "╔══════════════════════════════════════════════════════════════╗"
    printf "║  %-58s ║\n" "$1"
    echo "╚══════════════════════════════════════════════════════════════╝"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# ==================================================================
# 主流程
# ==================================================================

print_header "嵌入式部署 - HydroMotionLib (生产版本)"

# 检查构建目录
BUILD_DIR="out/build/unixgcc"
INSTALL_DIR="out/install/embedded_prod"

if [ ! -d "$BUILD_DIR" ]; then
    print_error "构建目录不存在: $BUILD_DIR"
    echo "请先运行: cmake --preset unixgcc"
    exit 1
fi

# 清理旧的安装目录
if [ -d "$INSTALL_DIR" ]; then
    print_warning "清理旧的安装目录: $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi

mkdir -p "$INSTALL_DIR"

# ==================================================================
# 步骤1：构建核心库（不包含仿真器）
# ==================================================================

print_header "步骤1: 构建核心运动控制库"

cd "$BUILD_DIR"
if make HydroMotionLib -j$(nproc); then
    print_success "核心库构建成功"
else
    print_error "核心库构建失败"
    exit 1
fi

# ==================================================================
# 步骤2：复制库文件和头文件
# ==================================================================

print_header "步骤2: 安装库文件和头文件"

# 复制静态库文件
cp libHydroMotionLib.a "$INSTALL_DIR/"
print_success "库文件已复制: libHydroMotionLib.a"

# 获取库文件大小
LIB_SIZE=$(du -h "$INSTALL_DIR/libHydroMotionLib.a" | cut -f1)
print_success "库文件大小: $LIB_SIZE"

# 创建头文件目录
mkdir -p "$INSTALL_DIR/include"

# 复制核心头文件（排除仿真器相关）
CORE_HEADERS=(
    "motion_control.h"
    "common_types.h"
    "motion_planner.h"
    "pressure_controller.h"
    "pump_converter.h"
    "segment_completion.h"
    "ramp_controller.h"
    "motion_utils.h"
    "motion_validator.h"
    "diagnostics.h"
    "diagnostics_monitor.h"
    "diagnostics_criteria.h"
    "protection_manager.h"
    "state_reporter.h"
    "rbf_pid.h"
    "recipe_validator.h"
    "segment_limits.h"
)

for header in "${CORE_HEADERS[@]}"; do
    SRC_PATH="../../include/$header"
    if [ -f "$SRC_PATH" ]; then
        cp "$SRC_PATH" "$INSTALL_DIR/include/"
        print_success "头文件已复制: $header"
    else
        print_warning "头文件不存在: $header"
    fi
done

# 复制必要的接口头文件
INTERFACE_HEADERS=(
    "hydro_interfaces.h"
    "hydro_hardware.h"
    "hydro_config.h"
)

for header in "${INTERFACE_HEADERS[@]}"; do
    SRC_PATH="../../include/$header"
    if [ -f "$SRC_PATH" ]; then
        cp "$SRC_PATH" "$INSTALL_DIR/include/"
        print_success "接口头文件已复制: $header"
    fi
done

# ==================================================================
# 步骤3：验证部署结果
# ==================================================================

print_header "步骤3: 验证部署结果"

echo "部署目录结构:"
ls -lh "$INSTALL_DIR" | tail -n +2
echo ""
echo "头文件目录 ($INSTALL_DIR/include/):"
ls -lh "$INSTALL_DIR/include/"

# 检查库文件是否包含仿真器符号
if command -v nm >/dev/null 2>&1; then
    print_header "验证：检查库文件内容"

    if nm "$INSTALL_DIR/libHydroMotionLib.a" | grep -q "HydraulicSim"; then
        print_error "库文件包含仿真器符号（不应该出现）"
        exit 1
    else
        print_success "库文件不包含仿真器符号 ✓"
    fi

    # 统计符号数量
    TOTAL_SYMBOLS=$(nm "$INSTALL_DIR/libHydroMotionLib.a" 2>/dev/null | wc -l)
    print_success "库文件符号总数: $TOTAL_SYMBOLS"
fi

# ==================================================================
# 步骤4：生成集成说明
# ==================================================================

print_header "步骤4: 生成集成说明"

cat > "$INSTALL_DIR/README.txt" << 'EOF'
================================================================
HydroMotionLib - 嵌入式生产版本
================================================================

本目录包含用于嵌入式平台部署的运动控制库。

目录结构:
- libHydroMotionLib.a    核心运动控制库（不含仿真器）
- include/               头文件目录

集成步骤:
1. 将 libHydroMotionLib.a 复制到您的项目库目录
2. 将 include/ 中的头文件复制到您的项目头文件目录
3. 在编译器中添加头文件搜索路径
4. 链接时添加: -lm -lHydroMotionLib
5. 根据目标平台调整 include/hdy_config.h 中的配置项

注意事项:
- 本库不包含液压仿真器（HydraulicSimLib）
- 本库使用静态内存分配，无需动态内存管理
- 本库符合 C99 标准，支持多种嵌入式编译器
- 请参考 include/motion_control.h 了解主要API接口

版本信息:
- 构建时间: $(date)
- 库大小: $(du -h libHydroMotionLib.a | cut -f1)

================================================================
EOF

print_success "集成说明已生成: $INSTALL_DIR/README.txt"

# ==================================================================
# 总结
# ==================================================================

print_header "部署完成总结"

echo "✓ 核心库已部署到: $INSTALL_DIR"
echo "✓ 库文件大小: $LIB_SIZE"
echo "✓ 头文件已复制: $(ls -1 $INSTALL_DIR/include | wc -l) 个"
echo ""
echo "下一步操作:"
echo "1. 将 $INSTALL_DIR 目录的内容传输到嵌入式开发环境"
echo "2. 参考 README.txt 进行项目集成"
echo "3. 根据目标平台调整编译选项"
echo ""

print_success "嵌入式部署成功完成！"
