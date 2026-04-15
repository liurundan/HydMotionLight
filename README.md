# HDY Motion Control Library

这是一个注塑机运动控制库的初步实现，目标是将工艺层与运动控制层分离，支持嵌入式 C 平台。

## 目录结构

- `src/motion_control.h`：核心数据结构与 C 接口定义。
- `src/motion_control.c`：运动规划、控制模式与泵速换算逻辑实现。
- `src/main.c`：纯 C 示例程序，演示多段动作与状态更新。
- `项目需求与设计说明书.md`：需求分析、架构设计、Sprint 计划说明。

## 架构说明

运动控制库实现了：

- `HDY_AxisREF` 实时反馈数据结构。
- 两种规划算法：位置基 (`V = sqrt(2 * a * s)`) 和时间基 (`V = a * t`)。
- 位置模式、速度斜坡模式、压力闭环模式。
- 多段 `HDY_MotionSegment` 配方执行。
- 状态与诊断接口。
- 段切换信号 (`SEGMENT_CHANGED`) 用于检测段启动。

## 编译与运行

```bash
mkdir -p build
gcc -std=c99 src/motion_control.c src/main.c -o build/motion_control_example -lm
./build/motion_control_example
```

## 下一步

- 将工艺层逻辑与 `HDY_MotionControlFB` 对接。
- 扩展更精细的段结束条件与曲线平滑策略。
- 增加诊断、异常处理与 PLCopen 风格接口。
- 扩展到其他注塑成型场景（如开模、顶出）。 
