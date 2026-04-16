# HMI / 上位机中文诊断对照表

> 适用对象：HMI 画面、上位机报警列表、现场联调记录、售后排障单。
>
> 适用原则：**不要直接把英文 `message` 原样显示给操作员**，建议以 `DIAGNOSTIC.code` 做主映射，`DIAGNOSTIC.flags` 做并发偏差标签，`LAST_FAULT_SNAPSHOT` 做售后详情页数据源。

## 1. HMI 直接映射建议

| 控制器字段 | HMI / 上位机用途 | 推荐显示方式 |
| --- | --- | --- |
| `DIAGNOSTIC.code` | 主诊断标题 | 映射为中文短标题，例如“压力过高”“当前段超时” |
| `DIAGNOSTIC.severity` | 报警等级 | 映射颜色、报警灯、蜂鸣器等级 |
| `DIAGNOSTIC.source` | 来源模块 | 详情页显示“配方 / 命令 / 执行 / 传感器 / 内部” |
| `DIAGNOSTIC.recovery` | 恢复建议 | 显示为按钮提示或操作建议 |
| `DIAGNOSTIC.protectionAction` | 控制器动作 | 显示“提示 / 降额 / 停止” |
| `DIAGNOSTIC.flags` | 并发偏差标签 | 以多个小标签显示“超压 / 欠压 / 流量偏差”等 |
| `DIAGNOSTIC_LATCH` | 最近一次事件 | 用于报警复盘、事件列表 |
| `LAST_DIAGNOSTIC_SNAPSHOT` | 最近一次诊断快照 | 用于联调详情页 |
| `LAST_FAULT_SNAPSHOT` | 最近一次故障快照 | 用于售后故障页、导出报表 |
| `DIAGNOSTIC_HISTORY` | 循环历史 | 用于最近若干条事件记录 |

## 2. 等级、颜色与声音建议

| 严重级别 | 中文建议 | 颜色建议 | 声音 / 灯光建议 | 说明 |
| --- | --- | --- | --- | --- |
| `INFO` | 提示 | 蓝色 / 灰色 | 可无声，仅状态栏提示 | 通常不影响继续操作 |
| `WARNING` | 预警 | 黄色 / 橙色 | 短鸣或黄灯闪烁 | 需要操作员关注，可能伴随降额 |
| `FAULT` | 故障 | 红色 | 连续报警或红灯常亮 | 通常需要停机、排查、复位 |

## 3. 主诊断中文对照表

| 诊断码 | 中文短标题 | 现场显示建议 | 严重级别 | 控制器动作 | 操作员建议 | 售后 / 工程师排查重点 |
| --- | --- | --- | --- | --- | --- | --- |
| `NONE` | 无活动诊断 | 当前无异常 | `NONE` | 无 | 无需处理 | 无 |
| `RECIPE_EMPTY` | 配方为空 | 当前未装载有效配方 | `WARNING` | 提示 | 重新下载或装载配方后再启动 | 检查配方下发流程、配方文件是否为空 |
| `RECIPE_TOO_LARGE` | 配方段数超限 | 配方段数超过控制器上限 | `WARNING` | 提示 | 精简段数后重新下发 | 检查段数是否超过 `HDY_MAX_SEGMENTS = 16` |
| `SEGMENT_INVALID` | 配方参数无效 | 当前段参数不合法，无法执行 | `WARNING` | 提示 | 修改段参数并重新装载 | 检查模式、目标量、限幅、结束条件、压力策略组合是否合法 |
| `RUNTIME_CONFIG_INVALID` | 运行配置错误 | 运行参数非法，控制器已拒绝执行 | `FAULT` | 停止 | 停机后检查参数，必要时复位控制器 | 重点检查泵速增益、泵速限幅、运行期配置是否被错误覆盖 |
| `START_CONTEXT_INVALID` | 启动条件不满足 | 当前状态不允许启动该段 | `WARNING` | 提示 | 检查启动时机、段索引和当前状态 | 检查是否在故障态、未装载配方或上下文不完整时发起启动 |
| `NO_RECIPE` | 未装载配方 | 尚未装载配方，不能启动或切段 | `WARNING` | 提示 | 先装载配方再执行 | 检查上位机下发顺序和配方装载确认逻辑 |
| `SEGMENT_INDEX_OUT_OF_RANGE` | 启动段号越界 | 请求启动的段号超出范围 | `WARNING` | 提示 | 修正段号后重试 | 检查段号从 0 开始计数，且必须小于当前配方段数 |
| `SEGMENT_NOT_COMPLETED` | 当前段未完成 | 当前段尚未满足结束条件，不能切下一段 | `WARNING` | 提示 | 等待完成或确认是否需要人工终止 | 检查工艺层是否过早调用 `NextSegment()` |
| `RECIPE_ALREADY_FINISHED` | 配方已完成 | 当前配方已执行完成 | `INFO` | 无 | 无需重复切段，可重新启动新配方 | 检查上位机是否在完成后仍继续发切段命令 |
| `ABORTED` | 已人工终止 | 当前执行已被人工或上层逻辑终止 | `INFO` | 无 | 根据工艺需要重新启动或复位 | 检查是谁触发了 `Abort()`，确认是否为预期停机 |
| `TIMEOUT` | 当前段超时 | 当前段执行时间超过允许上限 | `FAULT` | 停止 | 停机检查，处理后重新启动当前段或复位 | 检查超时限值、机构是否卡滞、反馈是否变化、工艺参数是否过严 |
| `OVER_PRESSURE` | 压力过高 | 实测压力高于参考值允许范围 | `WARNING` | 降额 | 关注设备压力，必要时停机 | 检查压力传感器、目标压力、容差、液压回路阻塞和阀动作是否匹配 |
| `UNDER_PRESSURE` | 压力偏低 | 实测压力低于参考值允许范围 | `WARNING` | 提示 | 检查是否存在泄压、供压不足或工艺条件未满足 | 检查泵输出、泄漏、保压设定、容差设置、反馈采样质量 |
| `FLOW_DEVIATION` | 流量偏差过大 | 实测流量与参考流量偏差过大 | `WARNING` | 降额 | 检查执行机构响应和油路状态 | 检查泵速换算、流量反馈、执行机构负载、阀开度和流量容差 |
| `POSITION_DEVIATION` | 位置偏差过大 | 实测位置与目标位置偏差过大 | `WARNING` | 提示 | 检查机构是否未到位或位置反馈异常 | 检查编码器 / 位移传感器、目标位置、位置容差、机械卡阻 |
| `VELOCITY_DEVIATION` | 速度偏差过大 | 实测速度与参考速度偏差过大 | `WARNING` | 提示 | 检查速度响应是否异常 | 检查速度反馈计算、加减速参数、负载突变和容差设定 |
| `SENSOR_FAULT` | 反馈信号异常 | 采样值无效，控制器已进入故障保护 | `FAULT` | 停止 | 立即检查传感器和采集链路，排除后复位 | 检查位置 / 速度 / 流量 / 压力 / 时间戳是否为非法值、负压或非有限数 |
| `TIMESTAMP_ROLLBACK` | 时间戳回退 | 反馈时间戳倒退，数据时序异常 | `FAULT` | 停止 | 检查采样源和通讯链路后复位 | 检查 PLC 时间基准、采样缓存顺序、总线乱序、重复帧问题 |
| `INTERNAL_ERROR` | 控制器内部错误 | 控制器内部状态异常 | `FAULT` | 停止 | 停机并复位控制器，必要时更换版本 | 检查内部状态是否损坏、索引是否异常、是否存在未覆盖边界条件 |

## 4. 并发偏差标签对照表

> `DIAGNOSTIC.flags` 用于表示同一周期内可能同时出现的多个偏差。主标题仍建议显示 `DIAGNOSTIC.code`，标签区再显示 `flags`。

| 标志位 | 十六进制 | 推荐中文标签 | 说明 |
| --- | --- | --- | --- |
| `HDY_DIAG_FLAG_OVER_PRESSURE` | `0x01` | 超压 | 当前周期检测到压力偏高 |
| `HDY_DIAG_FLAG_UNDER_PRESSURE` | `0x02` | 欠压 | 当前周期检测到压力偏低 |
| `HDY_DIAG_FLAG_FLOW_DEVIATION` | `0x04` | 流量偏差 | 当前周期检测到流量偏差 |
| `HDY_DIAG_FLAG_POSITION_DEVIATION` | `0x08` | 位置偏差 | 当前周期检测到位置偏差 |
| `HDY_DIAG_FLAG_VELOCITY_DEVIATION` | `0x10` | 速度偏差 | 当前周期检测到速度偏差 |
| `HDY_DIAG_FLAG_TIMEOUT` | `0x20` | 超时 | 当前周期检测到段执行超时 |
| `HDY_DIAG_FLAG_SENSOR_FAULT` | `0x40` | 反馈异常 | 当前周期检测到采样异常 |
| `HDY_DIAG_FLAG_TIMESTAMP_ROLLBACK` | `0x80` | 时间回退 | 当前周期检测到时间戳倒退 |

## 5. HMI 主标题优先级建议

当同一周期同时出现多个执行偏差时，主标题建议与控制器 `DIAGNOSTIC.code` 保持一致，优先级如下：

1. `TIMEOUT`
2. `OVER_PRESSURE`
3. `UNDER_PRESSURE`
4. `FLOW_DEVIATION`
5. `POSITION_DEVIATION`
6. `VELOCITY_DEVIATION`

补充说明：

- `SENSOR_FAULT` 与 `TIMESTAMP_ROLLBACK` 属于前置故障事件，现场应直接按故障页处理。
- 即使主标题只显示一个诊断，也建议保留 `flags` 标签区，避免并发偏差被隐藏。

## 6. 恢复建议中文映射

| `recovery` 枚举 | 推荐中文文案 | HMI 按钮 / 提示建议 |
| --- | --- | --- |
| `NONE` | 无需处理 | 不显示按钮或仅显示“已确认” |
| `AUTO_CLEAR` | 自动清除 | 显示“等待自动恢复” |
| `CHECK_COMMAND` | 检查命令 | 显示“检查启动 / 切段命令” |
| `CHECK_SENSOR` | 检查传感器 | 显示“检查传感器与采样链路” |
| `RELOAD_RECIPE` | 重装配方 | 显示“重新下载 / 装载配方” |
| `RESTART_SEGMENT` | 重启当前段 | 显示“处理后重新启动当前段” |
| `RESET_CONTROLLER` | 复位控制器 | 显示“排查后执行控制器复位” |

## 7. 保护动作中文映射

| `protectionAction` 枚举 | 推荐中文文案 | HMI 展示建议 |
| --- | --- | --- |
| `NONE` | 无强制动作 | 仅状态提示 |
| `WARNING` | 提示 | 黄色状态栏 / 报警列表记录 |
| `DERATE` | 降额 | 显示“已降额运行”或“建议降速 / 降压” |
| `STOP` | 停止 | 红色故障弹窗，提示设备已保护停机 |

## 8. 现场页面布局建议

### 8.1 主运行页

建议至少显示：

- 主诊断标题：来自 `DIAGNOSTIC.code`
- 报警等级：来自 `DIAGNOSTIC.severity`
- 控制器动作：来自 `DIAGNOSTIC.protectionAction`
- 并发标签：来自 `DIAGNOSTIC.flags`
- 操作建议：来自 `DIAGNOSTIC.recovery`

### 8.2 售后详情页

建议增加：

- 最近一次故障快照：`LAST_FAULT_SNAPSHOT`
- 最近一次诊断快照：`LAST_DIAGNOSTIC_SNAPSHOT`
- 最近诊断历史：`DIAGNOSTIC_HISTORY`
- 事件发生时的段号 / 段名 / 压力 / 流量 / 位置 / 速度 / 时间戳

## 9. 联调注意事项

- `DIAGNOSTIC` 是实时结果，在非故障保持态会自动清除；不要把它当作长期历史存储。
- 需要保留最近事件时，应使用 `DIAGNOSTIC_LATCH`、`LAST_DIAGNOSTIC_SNAPSHOT`、`LAST_FAULT_SNAPSHOT` 和 `DIAGNOSTIC_HISTORY`。
- 在实时故障已消除且控制器不处于故障态时，可调用 `HDY_MotionControlFB_AcknowledgeDiagnostics()` 清除保留诊断。
- 故障停机后的关键保留信息仍建议通过 `RESET` 后重新建立基线。
