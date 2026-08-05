# MEMORY.md — hdy-motion-light 长期记忆

## 项目概况
- 嵌入式C99液压运动控制库，IEC风格PLC编程+C/C++库集成
- 注塑机伺服泵控系统，核心动作: 开合模/顶针/座台/射胶储料
- PLCopen标准功能块架构，无动态内存分配

## 关键评审结论 (2026-05-18)
- 完成运动控制算法全面评审，报告: docs/review-motion-algorithm-20260518.md
- 综合评分 7.2/10，基础框架扎实但工艺交互有缺口
- 八大缺口: 无速度/压力连续修改、无低压护模、V/P切换仅评估不执行、段间过渡零输出、无电动功能、无S曲线、Stop减速逻辑脱节
- 开发计划6个Phase: 紧急修复→速度/压力连续修改→注塑工艺增强→S曲线→电动功能→精度提升
- Speed CONTINUOUSUPDATE: P0优先级，需_activeSegment运行时overlay层+规划器目标速度变更支持
- Pressure CONTINUOUSUPDATE: P0优先级，PressureHandle新增引脚+核心overlay层，RampController已支持
- 电动功能: 需新增HYD_MODE_ELECTRIC_TORQUE/SPEED、HYD_MotorRef反馈结构、ElectricController三环控制、HYD_ELECTRICCONTROL IEC FB，约4-6周

## 2026-05-25 工作记录
- 生成 `AGENTS.md` v2：Harness 专属定制版，实际扫描 `.codebuddy/skills/` 下 22 个技能元数据
- 区分 rigid(7个) vs flexible(7个) vs GStack(8个) 三级分类
- 包含：构建命令、架构约束7铁律、C99编码规范、TDD流程、技能完整清单+决策树+场景矩阵、OpenAPI/OMX预留、调试协议、提交规范
- v3升级: 补充数据流约束图、完整源文件/头文件/枚举/调优常量附录、命名约定表、诊断标志位、BufferMode/LiveUpdateFlags、HydroSimLib链接说明

## 已知Bug
- `__mcl_cmd_MoveAbsolute` done信号后速度未归零: Running→Done过渡时速度斜坡未完成
- Stop命令减速逻辑绕过规划器，不更新_plannerState
- ~~3个失败测试~~: 2026-08-04 在 MinGW 下实测 **ctest 55/55 全过**，已不复现

## Windows 开发环境 (2026-08-04)
- MinGW-w64 在 `D:\mingw64\bin`（gcc 12.2.0 win32-seh / gdb 11.2），**刻意未加系统环境变量**
  → 用户要求避免多套 gcc 干扰，所有 PATH 注入必须是**局部作用域**
- VS Code: Ctrl+Shift+B = `CMake: Build (MinGW)`；F5 = `Debug (MinGW) - main.exe`
- tasks.json 用 `options.shell.executable=cmd.exe` + `options.env.PATH` 局部注入
- **构建逻辑在 `scripts\build-mingw.cmd`**（含 configure 守卫），tasks.json 只调脚本。
  用法 `build-mingw.cmd [target] [rebuild]`，默认 all。这样规避 VS Code 对
  `& | ( )` 的 quoting 不确定性，且能在终端直接跑排查
- settings.json 必须保留 **`"task.autoDetect": "off"`**：
  否则 CMake Tools 自注册的 `CMake: xxx` 任务会争抢 Ctrl+Shift+B 默认构建任务
- tasks.json 里**不要用** `dependsOn` + `"hide": true` 组合，解析不稳
- problemMatcher 用 `"fileLocation": "absolute"`（gcc 在本项目输出绝对路径，已实测）

## Windows 脚本铁律
- **.cmd/.bat 必须全文纯 ASCII，注释也不例外**。cmd 用 OEM 代码页解析，
  UTF-8 中文会截断 `rem` 行并让后续 `set` 失效，症状是
  `'X' 不是内部或外部命令` + 变量莫名为空。校验：检查文件有无 >127 字节
- **CMakePresets 铁律**：`environment.PATH` 必须用 `$penv{PATH}` 追加。
  整体替换会让 add_custom_command 调系统工具时报 `make (e=2) 系统找不到指定的文件`（已实验证实）
- preset 的 environment 仅对 `--preset` 形式生效，`cmake --build <dir>` 直接指目录不应用
- cacheVariables 建议用带类型对象 `{"type":"BOOL","value":"ON"}`，裸字符串会得到 UNINITIALIZED
- 规模基准: 56 个 add_executable / 55 个 add_test；全量并行编译约 70s，ctest 约 10s

## 技术约定
- 默认技术栈: Vite+React+MUI+Tailwind(仅前端工具)，项目本身为纯C99嵌入式库
- IEC功能块EN/ENO引脚由PLC IEC层处理，不在core测试范围
- 工作习惯: TDD驱动、增量迭代、UseSkill标签、斜杠命令前缀
