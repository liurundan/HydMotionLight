# `FB_HydAxis` 两份评审报告对比与修复优先级

- **日期：** 2026-11-02
- **对比报告：** [`FB_HydAxis评审结论.md`](FB_HydAxis评审结论.md)、[`FB_HydAxis_review_2026-11-02.md`](FB_HydAxis_review_2026-11-02.md)
- **源码基准：** [`iec/FB_HydAxis.st`](../iec/FB_HydAxis.st) 与 HydroMotionLib IEC 适配器、运行时实现及相关测试
- **结论性质：** 只读源码复核；本文件不代表已完成任何代码修复

## 总结

两份报告都发现了真实风险，但第一份报告把若干实现假设当成了事实。当前代码中，`WriteParameter` 是同步 C 适配器，同一扫描内先 `EXECUTE=FALSE` 再 `TRUE` 会形成有效上升沿；Stop 也通过运行库的单轴所有权接管运动命令。这两项不构成第一份所称的 P0 功能失效。

源码复核确认的首要风险是：**Step 105 可能吞掉单扫描 Stop/EStop 边沿；压力 ramp 将零时间改成 1 ms；扫描时间基准固定为 1 ms；方向切换时末速度先于当前方向最大速度计算。** 初始化错误处理和 jerk 接口契约也需要修正。建议先解决 P0，再处理 P1，避免先做大范围状态机重构。

## 两份报告的冲突裁定

| 议题 | 报告差异 | 源码裁定 | 置信度 |
|---|---|---|---|
| 同周期 `WriteParameter` 握手 | 第一份认为 `DONE` 必须跨扫描才会出现，因此面积/泵参数写不进去；第二份认为 setter 的返回值未检查 | 第一份结论不适用于本库。`HYD_WriteParameter` 的 POU 直接调用 C 适配器；`EXECUTE=FALSE` 先将 `EXECUTE0` 清零，随后 `TRUE` 调用检测到上升沿并立即执行 `HYD_MotionControlFB_WriteParameter`、设置 `DONE`。【[`FB_HydAxis.st:183`](../iec/FB_HydAxis.st#L183)】【[`motion_interface.c:3125`](../src/motion_interface.c#L3125)】第二份指出的返回值未检查是实际问题。【[`FB_HydAxis.st:684`](../iec/FB_HydAxis.st#L684)】 | 高 |
| Stop 与运动 FB 同时 `EXECUTE=TRUE` | 第一份认为会产生未定义的同轴并发命令，Stop 可能被拒绝；第二份未把它列为主要问题 | 当前库按每轴单一命令所有权工作，Stop 可接管运动命令，原运动 FB 收到 `COMMANDABORTED`。【[`motion-runtime-contract.md:136`](../docs/architecture/motion-runtime-contract.md#L136)】集成测试验证 Stop 接管后速度降至零。【[`test_moveabsolute_stop_integration.c:76`](../tests/test_moveabsolute_stop_integration.c#L76)】因此第一份的“必然冲突/Stop 不执行”不成立。 | 高 |
| Step 105 与命令边沿 | 第一份指出 Start/Stop/EStop 边沿可能丢失；第二份未列出 | 源码确认。边沿在扫描开头由 `R_TRIG` 产生单拍 Q；Step 105 不处理这些 Q，且 `bNewSeg` 不接受 `iStep=105`。因此命令脉冲若落在该扫描会被消费后丢失。【[`FB_HydAxis.st:554`](../iec/FB_HydAxis.st#L554)】【[`FB_HydAxis.st:607`](../iec/FB_HydAxis.st#L607)】【[`FB_HydAxis.st:763`](../iec/FB_HydAxis.st#L763)】 | 高 |
| EStop 完成报警 | 第一份认为成功 `DONE` 不应置报警；第二份强调 `DONE` 与 `ERROR` 被合并处理 | 成功急停是否锁存报警取决于设备安全/复位策略，不能直接要求回到 Idle。实际缺陷是 `DONE` 与 `ERROR` 共用同一分支，且错误码被替换为 `16#FFFF`，使“停止成功”和“停止失败”不可区分。【[`FB_HydAxis.st:781`](../iec/FB_HydAxis.st#L781)】 | 高（错误码混淆）；中（成功后报警策略） |
| 最大速度回退与加速度边界 | 第一份称 fallback 后加速度边界保持 0；也指出泵速 1800/1700 不一致 | fallback 之后代码还会基于 `rMaxSpeedDir` 设置 `rAccMin/rAccMax/rDecMin/rDecMax`，所以“fallback 一定得到零边界”不成立。【[`FB_HydAxis.st:585`](../iec/FB_HydAxis.st#L585)】【[`FB_HydAxis.st:592`](../iec/FB_HydAxis.st#L592)】但泵速不一致属实：类型 `>=100` 时最大速度按 1800 rpm 计算，泵配置为 1700 rpm；另有 `bValid` 未检查。【[`FB_HydAxis.st:689`](../iec/FB_HydAxis.st#L689)】【[`FB_HydAxis.st:696`](../iec/FB_HydAxis.st#L696)】 | 高 |
| `fbCreate` / 肘杆配置重复触发 | 第一份担心 `fbCreate` 重复分配、配置每扫描重复触发 | `CreateMotion` 有 DONE/ERROR 早退和 `_CREATE_ACTIVE` 事务，未见每扫描重复分配轴的证据。【[`motion_interface.c:999`](../src/motion_interface.c#L999)】肘杆配置按上升沿开始并在后续扫描推进事务；“每扫描重新触发”不准确。【[`motion_interface.c:1224`](../src/motion_interface.c#L1224)】【[`motion_interface.c:1250`](../src/motion_interface.c#L1250)】但 Step 1 未处理配置错误，仍可能永久等待，这一点两份报告都指出且源码支持。【[`FB_HydAxis.st:712`](../iec/FB_HydAxis.st#L712)】 | 高 |
| 压力 ramp | 第一份关注压力历史目标未复位与 800 bar/s 默认值；第二份指出零时间退化为 1 ms | 第二份指出的是确定的边界问题：调用端把 `uiPressureAccTime=0` 钳成 1 ms，因此方法内的零值默认保护不可达；大压差会生成极大速率。【[`FB_HydAxis.st:824`](../iec/FB_HydAxis.st#L824)】【[`FB_HydAxis.st:243`](../iec/FB_HydAxis.st#L243)】第一份关于模式切换后 `rPresLastCycle` 可能过期也有依据，但其影响取决于压力 ramp 的工艺基准，列为待验证项。 | 高（零时间）；中（旧压力基准） |
| 状态输出 | 第一份认为 `bInVelocity` 在非速度模式可能串值；第二份指出停止/报警状态不清输出，且 `bDone` 与带内信号混用 | 非活动模式的 Move FB 收到 `EXECUTE=FALSE` 时会清其内部带内信号，因此“正常模式切换必然串值”不准确。【[`motion_interface.c:2328`](../src/motion_interface.c#L2328)】【[`motion_interface.c:2555`](../src/motion_interface.c#L2555)】但工艺块在 Stop/EStop/Alarm 分支不重置 `bInVelocity/bInPressure`，可能保留停止前的值；`bDone` 在速度/压力模式映射到带内信号，语义仍需明确。【[`FB_HydAxis.st:931`](../iec/FB_HydAxis.st#L931)】【[`FB_HydAxis.st:943`](../iec/FB_HydAxis.st#L943)】 | 高（停止时残留）；中（`bDone` 语义） |

## 修复优先级

### P0：进入设备调试前修复

1. **保证 Stop/EStop 边沿不在 Step 105 丢失。** 在 Step 105 前处理停止优先级，或用最小的待处理标志锁存 Stop/EStop；先补一项单扫描脉冲回归测试。避免引入通用命令队列。
2. **定义压力 ramp 的零时间策略并限制最大速率。** 零值应明确拒绝或采用经过设备验证的默认时间；对压力目标变化覆盖零时间、小压差、大压差和降压四种场景。
3. **急停成功与失败分开处理。** 保留库返回的 `ERRORID`；只有确认停止条件满足后才报告停止完成。安全急停仍需由独立安全链负责，工艺层的 `HYD_Stop` 不能替代安全功能。

### P1：跑通核心运动后立即修复

4. **统一任务周期来源。** 让工艺层过渡计时和运行库时间基准使用实际 PLC 周期；若项目必须运行在 1 ms，则在集成处显式校验。
5. **统一方向能力计算顺序。** 先计算当前方向最大速度，再计算 `rVel` 和 `rEndVel`；覆盖伸出转缩回及缩回转伸出两种不对称面积场景。
6. **初始化失败显式报警。** 校验机构类型、`FB_CalcAxisMaxSpeed.bValid`、参数 setter 返回值和肘杆配置 `ERROR/DONE`；失败时停止启动并保留具体错误码。
7. **按当前库能力约束 jerk。** 在库未支持非零 `JERK` 时明确限制 `uiJerk=0` 并拒绝非零设置；不需要为此重构规划器。

### P2：稳定性与接口收口

8. **锁存位置段参数。** 位置 FB 不支持在线更新，但减速判断继续读取外部实时输入；若运行中允许修改 HMI 参数，应在新段触发时锁存位置、速度、末速度和方向，运行期统一使用锁存值。
9. **统一状态输出语义。** 明确 `bDone` 是“目标进入带宽”还是“段终止”；在 Stop/EStop/Alarm 状态清理 `bInVelocity/bInPressure`，并决定是否将库 `STATE` 对 PLC 调用方公开。
10. **统一泵能力参数。** 最大速度计算与 `SetPumpConfig` 使用同一泵速上限，并检查 `bValid`；类型 105 当前的 1800/1700 rpm 差异会高估可达速度。
11. **核对压力 ramp 的参考基准。** `rPresLastCycle` 在运行中切离压力模式后未复位；结合压力段重新以实际压力初始化 ramp 的实现，确认重新进入压力模式时斜率是按“前一目标”还是“当前实际压力”计算。

## 关键证据与验证范围

- 同周期参数写入逻辑来自自研同步适配器；`test_parameter_iec` 与 `test_parameter_access` 通过，支持“当前库同步写入有效”的判断。
- Stop 所有权接管由运行时契约明确，`test_motion_interface_arbitration`、`test_moveabsolute_stop_integration` 和 `test_stop_immediate_done` 通过；这反驳“同轴 `EXECUTE` 高电平必然阻止 Stop”的断言。
- 本轮 6 项聚焦 C 库测试全部通过：参数访问、IEC 参数写入、接口仲裁、位置 Stop 集成、立即 Stop 完成、肘杆接口。
- 测试未直接编译或执行 `iec/FB_HydAxis.st`。因此 Step 105 边沿竞争、工艺层输入锁存和真实 PLC 周期仍需在目标 IEC 运行环境验证。

## 最终判断

第一份报告对同周期参数握手、Stop 并发冲突、最大速度 fallback 边界和重复创建的表述有误或过度推断；第二份报告在压力 ramp 零时间、jerk 不兼容和配置错误处理方面更贴近实现。第一份发现的 Step 105 边沿丢失、任务周期固定、参数运行期未锁存和泵速能力不一致仍有价值。

按 P0/P1 的 7 项完成后，再进入台架调试；本轮证据不足以把“成功 EStop 是否置报警”定为缺陷，应按设备的复位与安全策略决定。
