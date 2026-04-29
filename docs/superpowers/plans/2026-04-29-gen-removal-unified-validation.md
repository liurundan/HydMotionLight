# GEN 参数消除与命令校验统一化 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** 从所有 IEC FB 公共接口删除 GEN 参数，增加 BufferMode 参数，统一命令校验到核心层状态机。

**Architecture:** 核心层 `_commandGeneration` 重命名为 `_executionId`，递增时机从 `AbortNow` 移到 `BeginSegment`；IEC 层每个运动 FB 增加 `BUFFERMODE : INT` 输入、`_PENDING` + `_EXEC_ID` 私有字段，按统一模板重写命令处理流程。

**Tech Stack:** C99, matiec IEC61131-3 类型系统

---

### Task 1: 新增 HDY_BufferMode 枚举

**Files:**
- Modify: `include/common_types.h` — 在枚举区域新增

- [ ] **Step 1: 在 common_types.h 中新增枚举**

在 `HDY_FbCommand` 枚举附近（或其他枚举区域）新增：

```c
/* BufferMode: PLCopen-standard buffering mode for motion commands.
 * ABORT  (0): preempt current motion, execute immediately.
 * BUFFER (1): execute only when axis is idle; reject if axis is busy.
 * Values 2-5 are reserved for future blending modes. */
typedef enum {
    HDY_BUFFER_MODE_ABORT  = 0,
    HDY_BUFFER_MODE_BUFFER = 1
} HDY_BufferMode;
```

- [ ] **Step 2: 构建验证**

```bash
cmake --build out/build/unixgcc --target HydroMotionLib
```
Expected: 编译通过（仅新增枚举，无引用者，无警告）。

- [ ] **Step 3: 提交**

```bash
git add include/common_types.h
git commit -m "feat: add HDY_BufferMode enum for PLCopen buffer mode support"
```

---

### Task 2: motion_control.h — 重命名 _commandGeneration 为 _executionId

**Files:**
- Modify: `include/motion_control.h:201`

- [ ] **Step 1: 重命名字段并更新注释**

```c
// 旧 (line ~200-201):
    HDY_UINT8 _index;
    uint16_t _commandGeneration;   /* incremented on Abort, used by IEC layer for COMMANDABORTED detection */

// 新:
    HDY_UINT8 _index;
    uint16_t _executionId;   /* incremented on BeginSegment success; IEC layer uses for ownership tracking */
```

- [ ] **Step 2: 构建验证**

```bash
cmake --build out/build/unixgcc --target HydroMotionLib
```
Expected: 编译**失败**——`motion_control.c` 中仍引用 `_commandGeneration`。

- [ ] **Step 3: 提交（暂不提交，在 Task 3 中一起提交）**

---

### Task 3: motion_control.c — 移动递增点到 BeginSegment

**Files:**
- Modify: `src/motion_control.c:609` — 删除 `AbortNow` 中的 `_commandGeneration++`
- Modify: `src/motion_control.c:495` — 在 `BeginSegment` 末尾新增 `_executionId++`

- [ ] **Step 1: 删除 AbortNow 中的递增**

在 `HDY_AbortNow` 函数中删除一行：

```c
// 旧 (line ~609):
static void HDY_AbortNow(HDY_MotionControlFB* fb,
                         HDY_TIME timestamp) {
    if (fb == NULL) {
        return;
    }

    fb->_commandGeneration++;    // ← 删除这行
    HDY_ClearStartCommandInput(fb);

// 新:
static void HDY_AbortNow(HDY_MotionControlFB* fb,
                         HDY_TIME timestamp) {
    if (fb == NULL) {
        return;
    }

    HDY_ClearStartCommandInput(fb);
```

- [ ] **Step 2: 在 BeginSegment 末尾新增递增**

在 `HDY_BeginSegment` 函数中，`HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_STARTING);` 之后、`return true;` 之前新增：

```c
// 旧 (line ~494-495):
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_STARTING);
    return true;

// 新:
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_STARTING);
    fb->_executionId++;
    return true;
```

- [ ] **Step 3: 构建验证**

```bash
cmake --build out/build/unixgcc --target HydroMotionLib
```
Expected: 编译通过（`motion_interface.c` 中引用 `_commandGeneration` 的代码会失败，仅检查核心库编译）。

Wait — `motion_interface.c` 也引用 `_commandGeneration`。先确认构建目标：

```bash
cmake --build out/build/unixgcc 2>&1 | tail -30
```
Expected: `motion_interface.c` 中 `_commandGeneration` 引用报错。这是预期的——后续 Task 修复。

- [ ] **Step 4: 提交**

```bash
git add include/motion_control.h src/motion_control.c
git commit -m "refactor: rename _commandGeneration to _executionId, move increment from AbortNow to BeginSegment"
```

---

### Task 4: motion_interface.h — 更新所有 IEC FB 结构体

**Files:**
- Modify: `include/motion_interface.h`

所有 7 个含 GEN 的 FB 结构体变更如下：

- [ ] **Step 1: MoveProfile (line 77)**

```c
// 旧:
   __DECLARE_VAR(WORD, STATE)
   __DECLARE_VAR(REAL, PUMP_SPEED)
   __DECLARE_VAR(BOOL, INIT)
   __DECLARE_VAR(BOOL, EXECUTE0)
   __DECLARE_VAR(BOOL, DONE0)
   __DECLARE_VAR(BOOL, ACTIVE0)
   __DECLARE_VAR(WORD, GEN)

// 新:
   __DECLARE_VAR(WORD, STATE)
   __DECLARE_VAR(REAL, PUMP_SPEED)
   __DECLARE_VAR(INT, BUFFERMODE)
   __DECLARE_VAR(BOOL, INIT)
   __DECLARE_VAR(BOOL, EXECUTE0)
   __DECLARE_VAR(BOOL, DONE0)
   __DECLARE_VAR(BOOL, ACTIVE0)
   __DECLARE_VAR(BOOL, _PENDING)
   __DECLARE_VAR(WORD, _EXEC_ID)
```

- [ ] **Step 2: LoadProfile (line 104)**

```c
// 旧:
   __DECLARE_VAR(BOOL, DONE0)
   __DECLARE_VAR(WORD, GEN)

// 新:
   __DECLARE_VAR(BOOL, DONE0)
```

- [ ] **Step 3: Stop (line 117-124)**

```c
// 旧:
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,DONE0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(WORD,GEN)

// 新:
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,DONE0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(BOOL,_PENDING)
```

- [ ] **Step 4: MoveAbsolute (line 143-149)**

```c
// 旧:
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,DONE0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(WORD,GEN)

// 新:
    __DECLARE_VAR(INT,BUFFERMODE)
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,DONE0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(BOOL,_PENDING)
    __DECLARE_VAR(WORD,_EXEC_ID)
```

- [ ] **Step 5: MoveVelocity (line 168-174)**

```c
// 旧:
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,INVELOCITY0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(WORD,GEN)

// 新:
    __DECLARE_VAR(INT,BUFFERMODE)
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,INVELOCITY0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(BOOL,_PENDING)
    __DECLARE_VAR(WORD,_EXEC_ID)
```

- [ ] **Step 6: Reset (line 188-190)**

```c
// 旧:
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,DONE0)
    __DECLARE_VAR(WORD,GEN)

// 新:
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,DONE0)
```

- [ ] **Step 7: PressureHandle (line 207-213)**

```c
// 旧:
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,INPRESSURE0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(WORD,GEN)

// 新:
    __DECLARE_VAR(INT,BUFFERMODE)
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,INPRESSURE0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(BOOL,_PENDING)
    __DECLARE_VAR(WORD,_EXEC_ID)
```

- [ ] **Step 8: 构建验证（预期失败——motion_interface.c 尚未更新）**

```bash
cmake --build out/build/unixgcc 2>&1 | grep -c 'error:'
```
Expected: 有编译错误（motion_interface.c 引用 GEN 和 _commandGeneration）。

- [ ] **Step 9: 提交**

```bash
git add include/motion_interface.h
git commit -m "refactor: remove GEN, add BUFFERMODE and internal tracking fields to IEC FB structs"
```

---

### Task 5: motion_interface.c — 重写 MoveAbsolute

**Files:**
- Modify: `src/motion_interface.c:484-599` — 完整的 `__mcl_cmd_MoveAbsolute` 函数

- [ ] **Step 1: 替换 MoveAbsolute 实现**

将 `__mcl_cmd_MoveAbsolute` 整个函数替换为：

```c
void __mcl_cmd_MoveAbsolute(HDY_MOVEABSOLUTE *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
    IEC_INT bufferMode = __GET_VAR(data__->BUFFERMODE);
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);
    IEC_WORD myExecId = __GET_VAR(data__->_EXEC_ID);

    if (execRising)
    {
        if (bufferMode == HDY_BUFFER_MODE_ABORT) {
            HDY_MotionControlFB_Abort(fb);
            HDY_MotionControlFB_Scan(fb);
        }

        HDY_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HDY_MotionSegment segment = buildPositionSegment(
            __GET_VAR(data__->POSITION),
            __GET_VAR(data__->VELOCITY),
            __GET_VAR(data__->ACCELERATION),
            dir);

        if (!HDY_MotionControlFB_LoadDirectSegment(fb, &segment))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (!HDY_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        if (fb->STATE.active && fb->_activeSegmentSource == HDY_SEGMENT_SOURCE_DIRECT) {
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)fb->_executionId);
            __SET_VAR(data__->, _PENDING, , false);
            myExecId = (IEC_WORD)fb->_executionId;
        } else if (fb->FB_STATE == HDY_FB_STATE_ABORTED && !fb->STATE.active) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (myExecId != 0)
    {
        if (myExecId != (IEC_WORD)fb->_executionId) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, DONE, , false);
        } else {
            if (fb->SEGMENT_COMPLETED || (HDY_MotionControlFB_IsDone(fb) && fb->STATE.finished))
            {
                __SET_VAR(data__->, DONE, , true);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
            }
            else if (fb->STATE.active)
            {
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , true);
            }
            else if (HDY_MotionControlFB_IsError(fb))
            {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
            }
            else
            {
                __SET_VAR(data__->, BUSY, , HDY_MotionControlFB_IsBusy(fb));
                __SET_VAR(data__->, ACTIVE, , fb->STATE.active ? true : false);
            }
        }
    }

    __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, EXECUTE0, , execute);
}
```

- [ ] **Step 2: 构建验证**

```bash
cmake --build out/build/unixgcc 2>&1 | tail -20
```
Expected: 仍有其他 FB 的编译错误。

- [ ] **Step 3: 提交**

```bash
git add src/motion_interface.c
git commit -m "refactor: rewrite MoveAbsolute with unified command flow, remove GEN, add BufferMode"
```

---

### Task 6: motion_interface.c — 重写 MoveVelocity

**Files:**
- Modify: `src/motion_interface.c:605-727` — 完整的 `__mcl_cmd_MoveVelocity` 函数

- [ ] **Step 1: 替换 MoveVelocity 实现**

```c
void __mcl_cmd_MoveVelocity(HDY_MOVEVELOCITY *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, INVELOCITY, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
    IEC_INT bufferMode = __GET_VAR(data__->BUFFERMODE);
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);
    IEC_WORD myExecId = __GET_VAR(data__->_EXEC_ID);
    HDY_REAL targetVelocity = __GET_VAR(data__->VELOCITY);

    if (execRising)
    {
        if (bufferMode == HDY_BUFFER_MODE_ABORT) {
            HDY_MotionControlFB_Abort(fb);
            HDY_MotionControlFB_Scan(fb);
        }

        HDY_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HDY_MotionSegment segment = buildVelocitySegment(
            targetVelocity,
            __GET_VAR(data__->ACCELERATION),
            dir);

        if (!HDY_MotionControlFB_LoadDirectSegment(fb, &segment))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (!HDY_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, INVELOCITY, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        if (fb->STATE.active && fb->_activeSegmentSource == HDY_SEGMENT_SOURCE_DIRECT) {
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)fb->_executionId);
            __SET_VAR(data__->, _PENDING, , false);
            myExecId = (IEC_WORD)fb->_executionId;
        } else if (fb->FB_STATE == HDY_FB_STATE_ABORTED && !fb->STATE.active) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INVELOCITY, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (myExecId != 0)
    {
        if (myExecId != (IEC_WORD)fb->_executionId) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INVELOCITY, , false);
        } else {
            if (fb->STATE.active)
            {
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , true);

                HDY_REAL velError = fb->AXIS_REF.velocity - targetVelocity;
                if (velError < 0.0f) velError = -velError;
                if (targetVelocity > 0.0f && velError < targetVelocity * 0.05f)
                {
                    __SET_VAR(data__->, INVELOCITY, , true);
                }
                else
                {
                    __SET_VAR(data__->, INVELOCITY, , false);
                }
            }
            else if (HDY_MotionControlFB_IsError(fb))
            {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INVELOCITY, , false);
            }
            else
            {
                __SET_VAR(data__->, BUSY, , HDY_MotionControlFB_IsBusy(fb));
                __SET_VAR(data__->, ACTIVE, , fb->STATE.active ? true : false);
            }
        }
    }

    __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, INVELOCITY0, , __GET_VAR(data__->INVELOCITY));
    __SET_VAR(data__->, EXECUTE0, , execute);
}
```

- [ ] **Step 2: 构建验证**

```bash
cmake --build out/build/unixgcc 2>&1 | tail -20
```

- [ ] **Step 3: 提交**

```bash
git add src/motion_interface.c
git commit -m "refactor: rewrite MoveVelocity with unified command flow, remove GEN, add BufferMode"
```

---

### Task 7: motion_interface.c — 重写 PressureHandle

**Files:**
- Modify: `src/motion_interface.c:790-918` — 完整的 `__mcl_cmd_PressureHandle` 函数

- [ ] **Step 1: 替换 PressureHandle 实现**

```c
void __mcl_cmd_PressureHandle(HDY_PRESSUREHANDLE *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
    IEC_INT bufferMode = __GET_VAR(data__->BUFFERMODE);
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);
    IEC_WORD myExecId = __GET_VAR(data__->_EXEC_ID);
    HDY_REAL targetPressure = __GET_VAR(data__->PRESSURE);

    if (execRising)
    {
        if (bufferMode == HDY_BUFFER_MODE_ABORT) {
            HDY_MotionControlFB_Abort(fb);
            HDY_MotionControlFB_Scan(fb);
        }

        HDY_MotionSegment segment = buildPressureSegment(
            targetPressure,
            __GET_VAR(data__->PRESSURERAMPRATE),
            __GET_VAR(data__->DURATION));

        if (!HDY_MotionControlFB_LoadDirectSegment(fb, &segment))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (!HDY_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        if (fb->STATE.active && fb->_activeSegmentSource == HDY_SEGMENT_SOURCE_DIRECT) {
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)fb->_executionId);
            __SET_VAR(data__->, _PENDING, , false);
            myExecId = (IEC_WORD)fb->_executionId;
        } else if (fb->FB_STATE == HDY_FB_STATE_ABORTED && !fb->STATE.active) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (myExecId != 0)
    {
        if (myExecId != (IEC_WORD)fb->_executionId) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
        } else {
            if (fb->SEGMENT_COMPLETED || (HDY_MotionControlFB_IsDone(fb) && fb->STATE.finished))
            {
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
            }
            else if (fb->STATE.active)
            {
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , true);

                HDY_REAL pressError = fb->AXIS_REF.pressure - targetPressure;
                if (pressError < 0.0f) pressError = -pressError;
                if (targetPressure > 0.0f && pressError < 0.5f)
                {
                    __SET_VAR(data__->, INPRESSURE, , true);
                }
                else
                {
                    __SET_VAR(data__->, INPRESSURE, , false);
                }
            }
            else if (HDY_MotionControlFB_IsError(fb))
            {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
            }
            else
            {
                __SET_VAR(data__->, BUSY, , HDY_MotionControlFB_IsBusy(fb));
                __SET_VAR(data__->, ACTIVE, , fb->STATE.active ? true : false);
            }
        }
    }

    __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, INPRESSURE0, , __GET_VAR(data__->INPRESSURE));
    __SET_VAR(data__->, EXECUTE0, , execute);
}
```

- [ ] **Step 2: 构建验证**

```bash
cmake --build out/build/unixgcc 2>&1 | tail -20
```

- [ ] **Step 3: 提交**

```bash
git add src/motion_interface.c
git commit -m "refactor: rewrite PressureHandle with unified command flow, remove GEN, add BufferMode"
```

---

### Task 8: motion_interface.c — 重写 MoveProfile

**Files:**
- Modify: `src/motion_interface.c:307-399` — 完整的 `__mcl_cmd_MoveProfile` 函数

- [ ] **Step 1: 替换 MoveProfile 实现**

```c
void __mcl_cmd_MoveProfile(HDY_MOVEPROFILE *data__)
{
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
    HDY_MotionControlFB *fb = __MK_GetPublic_MotionControlFB(axisIndex);

    if (fb == NULL) {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HDY_DIAG_CODE_INTERNAL_ERROR);
        __SET_VAR(data__->, ENO,, false);
        return;
    }

    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);

    /* Update AXIS_REF from MOTION feedback */
    HDY_AXISMOTION motionData = __GET_VAR(data__->MOTION);
    fb->AXIS_REF.position = motionData.ACTPOSITION;
    fb->AXIS_REF.velocity = motionData.ACTVELOCITY;
    fb->AXIS_REF.flow     = motionData.ACTFLOW;
    fb->AXIS_REF.pressure = motionData.ACTPRESSURE;
    fb->AXIS_REF.timestamp = motionData.TIMESTAMP;

    IEC_INT bufferMode = __GET_VAR(data__->BUFFERMODE);
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);
    IEC_WORD myExecId = __GET_VAR(data__->_EXEC_ID);

    if (execRising) {
        HDY_TIME currentTime = motionData.TIMESTAMP;

        if (bufferMode == HDY_BUFFER_MODE_ABORT) {
            HDY_MotionControlFB_Abort(fb);
            HDY_MotionControlFB_Scan(fb);
        }

        /* Build 1-segment recipe from MOTION if no preloaded recipe */
        if (fb->RECIPE_SIZE == 0 && !fb->DIRECT_SEGMENT_VALID) {
            HDY_MotionSegment segment = buildSegmentFromMotion(&motionData);
            if (!HDY_MotionControlFB_LoadRecipe(fb, &segment, 1)) {
                __SET_VAR(data__->, ERROR,, true);
                __SET_VAR(data__->, ERRORID,, (IEC_WORD)HDY_DIAG_CODE_SEGMENT_INVALID);
                __SET_VAR(data__->, EXECUTE0,, execute);
                return;
            }
        }

        /* Start segment (recipe or direct) */
        if (!HDY_MotionControlFB_StartSegment(fb, 0, currentTime)) {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
            __SET_VAR(data__->, EXECUTE0,, execute);
            return;
        }

        __SET_VAR(data__->, _PENDING,, true);
        __SET_VAR(data__->, _EXEC_ID,, (IEC_WORD)0);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    /* Ownership tracking */
    if (isPending) {
        if (fb->STATE.active) {
            __SET_VAR(data__->, _EXEC_ID,, (IEC_WORD)fb->_executionId);
            __SET_VAR(data__->, _PENDING,, false);
            myExecId = (IEC_WORD)fb->_executionId;
        } else if (fb->FB_STATE == HDY_FB_STATE_ABORTED && !fb->STATE.active) {
            __SET_VAR(data__->, ACTIVE,, false);
            __SET_VAR(data__->, BUSY,, false);
            __SET_VAR(data__->, _PENDING,, false);
            __SET_VAR(data__->, EXECUTE0,, execute);
            return;
        }
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    if (myExecId != 0) {
        if (myExecId != (IEC_WORD)fb->_executionId) {
            __SET_VAR(data__->, ACTIVE,, false);
            __SET_VAR(data__->, BUSY,, false);
        } else {
            __SET_VAR(data__->, ACTIVE,, fb->STATE.active ? true : false);
            __SET_VAR(data__->, BUSY,, HDY_MotionControlFB_IsBusy(fb));
            __SET_VAR(data__->, DONE,, (HDY_MotionControlFB_IsDone(fb) && fb->STATE.finished) ? true : false);
            __SET_VAR(data__->, ERROR,, HDY_MotionControlFB_IsError(fb) ? true : false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, STATE,, (IEC_WORD)fb->STATE.status);
            __SET_VAR(data__->, PUMP_SPEED,, (IEC_REAL)fb->PUMP_SPEED);
            __SET_VAR(data__->, ENO,, true);

            if (fb->_activeSegmentValid) {
                HDY_AXISMOTION motionOut = __GET_VAR(data__->MOTION);
                writeMotionFromSegment(&motionOut, fb);
                __SET_VAR(data__->, MOTION,, motionOut);
            }
        }
    }

    __SET_VAR(data__->, EXECUTE0,, execute);
}
```

- [ ] **Step 2: 构建验证**

```bash
cmake --build out/build/unixgcc 2>&1 | tail -20
```

- [ ] **Step 3: 提交**

```bash
git add src/motion_interface.c
git commit -m "refactor: rewrite MoveProfile with unified command flow, remove GEN, add BufferMode"
```

---

### Task 9: motion_interface.c — 重写 Stop

**Files:**
- Modify: `src/motion_interface.c:405-478` — 完整的 `__mcl_cmd_Stop` 函数

- [ ] **Step 1: 替换 Stop 实现**

Stop 不使用 `_executionId`——通过 `FB_STATE` 和 `STATE.active` 直接判断 Abort 完成。

```c
void __mcl_cmd_Stop(HDY_STOP *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);

    if (execRising)
    {
        HDY_MotionControlFB_Abort(fb);
        HDY_MotionControlFB_Scan(fb);
        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        if (!fb->STATE.active && fb->FB_STATE != HDY_FB_STATE_RUNNING)
        {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, _PENDING, , false);
        }
        else
        {
            __SET_VAR(data__->, BUSY, , true);
        }

        if (HDY_MotionControlFB_IsError(fb))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
        }
    }

    __SET_VAR(data__->, EXECUTE0, , execute);
}
```

- [ ] **Step 2: 构建验证**

```bash
cmake --build out/build/unixgcc 2>&1 | tail -20
```

- [ ] **Step 3: 提交**

```bash
git add src/motion_interface.c
git commit -m "refactor: rewrite Stop with simplified state-based completion, remove GEN"
```

---

### Task 10: motion_interface.c — 重写 Reset（+ LoadProfile GEN 清理）

**Files:**
- Modify: `src/motion_interface.c:733-784` — `__mcl_cmd_Reset` 函数
- Modify: `src/motion_interface.c:296-305` — `__mcl_cmd_LoadProfile` GEN 引用清理

- [ ] **Step 1: 替换 Reset 实现**

Reset 不涉及段或抢占，直接 SoftReset → DONE。

```c
void __mcl_cmd_Reset(HDY_RESET *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
    if (fb->FB_STATE == HDY_FB_STATE_DISABLED)
    {
        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        HDY_MotionControlFB_SoftReset(fb);
        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
    }

    __SET_VAR(data__->, EXECUTE0, , execute);
}
```

- [ ] **Step 2: 构建验证**

```bash
cmake --build out/build/unixgcc 2>&1 | tail -20
```
Expected: 编译通过，无错误无警告。

- [ ] **Step 3: 提交**

```bash
git add src/motion_interface.c
git commit -m "refactor: rewrite Reset and LoadProfile, remove remaining GEN references"
```

---

### Task 11: 构建、测试、验证

- [ ] **Step 1: 完整构建**

```bash
cmake --build out/build/unixgcc 2>&1
```
Expected: 零错误、零警告。

- [ ] **Step 2: 运行全部测试**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```
Expected: 所有测试通过。

- [ ] **Step 3: 提交（如有遗漏）**

```bash
git status
```
如有未提交的变更，提交。

---

### 变更总结

| 文件 | 变更 |
|------|------|
| `include/common_types.h` | +`HDY_BufferMode` 枚举 |
| `include/motion_control.h` | `_commandGeneration` → `_executionId` |
| `src/motion_control.c` | 递增点移动（`AbortNow` 删除，`BeginSegment` 新增） |
| `include/motion_interface.h` | 7 个 FB 删除 GEN，5 个 FB 新增 BUFFERMODE + _PENDING + _EXEC_ID |
| `src/motion_interface.c` | 6 个 IEC FB 实现重写 |
