FUNCTION_BLOCK FB_CalcAxisMaxSpeed
  VAR_INPUT
    rAreaExtendSide : REAL;
    rAreaRetractSide : REAL;
    rPumpMaxRPM : REAL := 1800;
    rPumpDisplacement : REAL := 50;
    iMechanismType : SINT := 0;
  END_VAR
  VAR_OUTPUT
    rMaxSpeedExtend : REAL;
    rMaxSpeedRetract : REAL;
    bValid : BOOL;
  END_VAR
  VAR
    rMaxFlowRate_cc : REAL;
    rCoeff : REAL := 16.667;
    rKVelocityRateExtend : REAL := 0.73;
    rKVelocityRateRetract : REAL := 1.18;
  END_VAR


  // 1. 默认状态为无效
  bValid := FALSE;

  // 2. 参数有效性检查 (面积必须大于0，防止除零)
  IF (rAreaExtendSide <= 0.0) OR (rAreaRetractSide <= 0.0) OR 
     (rPumpMaxRPM < 0.0) OR (rPumpDisplacement <= 0.0) THEN
      
      rMaxSpeedExtend  := 0.0;
      rMaxSpeedRetract := 0.0;
      
  ELSE
      bValid := TRUE;

      // 3. 计算油泵最大流量
      // 结果单位为 cc/min (立方厘米/分)=mL/min
      rMaxFlowRate_cc := rPumpDisplacement * rPumpMaxRPM;

      // 4. 计算伸出最大速度
      // 公式: (流量 cc/min * 1000) / (面积 mm² * 60)
      // 简化为: 流量 * 16.667 / 面积
      rMaxSpeedExtend := (rMaxFlowRate_cc * rCoeff) / rAreaExtendSide;

      // 5. 计算缩回最大速度
      rMaxSpeedRetract := (rMaxFlowRate_cc * rCoeff) / rAreaRetractSide;

      IF iMechanismType = 1 THEN
        rMaxSpeedExtend  := rMaxSpeedExtend * rKVelocityRateExtend; // kv=f(Xm),变速比是动模板位置的函数
        rMaxSpeedRetract := rMaxSpeedRetract * rKVelocityRateRetract;
      END_IF;
  END_IF;
END_FUNCTION_BLOCK

FUNCTION IS_BIT_TRUE : BOOL
  VAR_INPUT
    Data : WORD;
    BitPos : INT;
  END_VAR
  VAR
    constData : WORD := 16#0001;
  END_VAR

  // 位数超范围时直接返回 FALSE（也可增加 Error 输出）
  IF (BitPos < 0) OR (BitPos > 15) THEN
      IS_BIT_TRUE := FALSE;
  ELSE
      IS_BIT_TRUE := (SHR(Data, BitPos) AND constData ) = 16#0001;
  END_IF
END_FUNCTION

FUNCTION_BLOCK FB_HydAxis
  VAR_IN_OUT
    stHydAxisRef : ST_HydAxisRef;
  END_VAR
  VAR_INPUT
    bStart : BOOL;
    bStop : BOOL;
    bEStop : BOOL; // 工艺层受控急停请求；设备安全急停须由独立安全链实现
    bReset : BOOL;
    uiDir : UINT;
    uiMode : UINT;
    uiPres : UINT;
    uiSpd : UINT;
    uiEndSpd : UINT;
    udiPos : UDINT;
    uiForce : UINT;
    uiAcc : UINT;
    uiDec : UINT;
    uiJerk : UINT;
    uiPressureAccTime : UINT;
  END_VAR
  VAR_OUTPUT
    bBusy : BOOL;
    bDone : BOOL;
    bInVelocity : BOOL;
    bInPressure : BOOL;
    bAlarm : BOOL;
    bStandStillOrAllowUpdate : BOOL;
    dwAlarmID : DWORD;
  END_VAR
  VAR
    iStep : INT;
    iStepPrev : INT := 0;
    uiActiveMode : UINT;
    bCmdExecute : BOOL;
    iDir : SINT;
    iDirSign : INT;
    iDirSignLastSeg : INT := 0;
    fbCreate : HYD_CreateMotion;
    fbMoveContAbs : HYD_MoveContinuousAbsolute;
    fbMoveVel : HYD_MoveVelocity;
    fbPresHdl : HYD_PressureHandle;
    fbStop : HYD_Stop;
    fbEStop : HYD_Stop;
    fbReset : HYD_Reset;
    fbReadFB : HYD_ReadSimFeedback;
    fbHYD_WriteParameter : HYD_WriteParameter;
    fbHYD_SetAxisFeedback : HYD_SetAxisFeedback;
    fbCalcMaxSpeed : FB_CalcAxisMaxSpeed;
    fbHYD_ConfigureToggleMechanism : HYD_ConfigureToggleMechanism;
    fbHYD_ReadStatus : HYD_ReadStatus;
    siAxisId : SINT;
    iMechBaseClass : SINT := 0;
    rTrigStart : R_TRIG;
    rTrigStop : R_TRIG;
    rTrigEStop : R_TRIG;
    rTrigReset : R_TRIG;
    rTrigDone : R_TRIG;
    rPos : REAL;
    rVel : REAL;
    rEndVel : REAL;
    rPres : REAL;
    rAcc : REAL := 200.0;
    rDec : REAL := 200.0;
    rJerk : REAL;
    rStopDec : REAL := 5000.0;
    rFlowPercent : REAL;
    rPressureRampRate : REAL;
    rPresLastCycle : REAL := 0;
    rMaxSpeed : REAL;
    rMaxSpeedDir : REAL;
    rVelLastSeg : REAL := 0.0;
    rEndVelLastSeg : REAL := 0.0;
    uiLastSegMode : UINT := 0;
    bSegVelValid : BOOL := FALSE;
    rVel0 : REAL := 0.0;
    rVelSigned : REAL;
    bNewSeg : BOOL;
    bTransDecel : BOOL := FALSE;
    rDecSeg : REAL := 200.0;
    rTransTime : REAL := 0.0;
    rTransTimer : REAL := 0.0;
    rVelTransMax : REAL := 0.0;
    rDecDist : REAL;
    rAccMin : REAL;
    rAccMax : REAL;
    rDecMin : REAL;
    rDecMax : REAL;
    rStopDecMax : REAL;
    rRampMinTime : REAL := 0.05;
    rRampMaxTime : REAL := 10.0;
    rDeltaVDeadZone : REAL := 1.0;
    rCycleTime : REAL := 0.001;
    bFBError : BOOL;
    wFBErrorID : WORD;
    iDoneCnt : INT := 0;
    iAxisState : UINT := 0;
    rPosition : REAL;
    rVeloctity : REAL;
    rFlow : REAL;
    rPerssure : REAL;
    rCycle : REAL;
    bConfig : BOOL;
  END_VAR

  METHOD PUBLIC SetVelocityToFlowGain : BOOL
    VAR_INPUT
      rVelToFlowGain : REAL;
    END_VAR
    VAR
      iParamterIndex : INT := 5;
    END_VAR
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := iParamterIndex, VALUE := REAL_TO_LREAL(rVelToFlowGain) );

    SetVelocityToFlowGain := fbHYD_WriteParameter.Done;
  END_METHOD

  METHOD PUBLIC SetPidType : BOOL
    VAR_INPUT
      rTypePID : REAL;
    END_VAR
    VAR
      iParamterIndex : INT := 25;
    END_VAR
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := iParamterIndex, VALUE := REAL_TO_LREAL(rTypePID) );
    SetPidType := fbHYD_WriteParameter.Done;
  END_METHOD

  METHOD PUBLIC SetCylinderConfig : BOOL
    VAR_INPUT
      rExtendArea : REAL;
      rRetractArea : REAL;
    END_VAR
    VAR
      iCylinderExtendArea : INT := 31;
      iCylinderRetractArea : INT := 32;
    END_VAR
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := iCylinderExtendArea, VALUE := REAL_TO_LREAL(rExtendArea) );

    SetCylinderConfig := fbHYD_WriteParameter.Done;

    IF NOT SetCylinderConfig THEN
      RETURN;
    END_IF;

    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := iCylinderRetractArea, VALUE := REAL_TO_LREAL(rRetractArea) );


    SetCylinderConfig := fbHYD_WriteParameter.Done;
  END_METHOD

  METHOD PUBLIC CalcPresRampRate : REAL
    VAR_INPUT
      rTargetPres : REAL;
      rPrevPres : REAL;
      uiTimeMs : UINT;
    END_VAR
    VAR
      rDeltaPres : REAL;
      rTimeSec : REAL;
    END_VAR
    // 800 bar/s 是当前工艺层的默认值和软件上限；现场应按液压系统验证该值。
    CalcPresRampRate := 800.0; 

    // 1. 时间除零保护
    IF uiTimeMs = 0 THEN
        RETURN; // 返回默认 800.0 bar/s
    END_IF;

    // 2. 计算压差绝对值
    rDeltaPres := ABS(rTargetPres - rPrevPres);

    // 3. 压差为零退化保护 (如果目标压力没变，给最大斜率，让底层FB直接到位)
    IF rDeltaPres < 0.1 THEN
        RETURN; // 返回默认 800.0 bar/s
    END_IF;

    // 4. 核心换算逻辑: ms转s, 压差除以时间
    rTimeSec := UINT_TO_REAL(uiTimeMs) * 0.001;
    CalcPresRampRate := MIN(rDeltaPres / rTimeSec, 800.0); // 单位: bar/s，限制极端斜率
  END_METHOD

  METHOD PUBLIC SetToggleMechanism : BOOL
    VAR_INPUT
      iAxis : SINT;
      iType : SINT;
    END_VAR

    IF iAxis <> -1 AND iType = 3 THEN
      fbHYD_ConfigureToggleMechanism(
         AXISID := iAxis,
         EXECUTE := TRUE,
         LR := 190,
         LF := 260,
         LPF := 150,
         LPK := 100,
         LD := 70,
         HF := 150,
         HM := 110,
         DC := 435,
         SM := 210,
         XHANDOFF := 0,
         SIGMA_K := -1,
         SIGN_B := -1,
         TAU_S := -1,
         SIGMA_C := -1 );
    END_IF;

    IF iAxis <> -1 AND iType = 105 THEN
      fbHYD_ConfigureToggleMechanism(
         AXISID := iAxis,
         EXECUTE := TRUE,
         LR := 260,
         LF := 323,
         LPF := 190,
         LPK := 100,
         LD := 125,
         HF := 200,
         HM := 170,
         DC := 580,
         SM := 450,
         XHANDOFF := 0,
         SIGMA_K := -1,
         SIGN_B := -1,
         TAU_S := -1,
         SIGMA_C := -1 );
    END_IF;

    SetToggleMechanism := NOT fbHYD_ConfigureToggleMechanism.ERROR AND fbHYD_ConfigureToggleMechanism.DONE;
  END_METHOD

  METHOD PUBLIC SetPumpConfig : BOOL
    VAR_INPUT
      rDisplacementCCRev : REAL;
      rMaxSpeedRPM : REAL;
    END_VAR
    VAR
      rEfficiency : REAL := 0.95;
    END_VAR
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := 28, VALUE := REAL_TO_LREAL(rDisplacementCCRev) );

    SetPumpConfig := fbHYD_WriteParameter.Done;
    IF NOT SetPumpConfig THEN
      RETURN;
    END_IF;

    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := 30, VALUE := REAL_TO_LREAL(rMaxSpeedRPM) );


    SetPumpConfig := fbHYD_WriteParameter.Done;
    IF NOT SetPumpConfig THEN
      RETURN;
    END_IF;

    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := 29, VALUE := REAL_TO_LREAL(rEfficiency) );


    SetPumpConfig := fbHYD_WriteParameter.Done;
    IF NOT SetPumpConfig THEN
      RETURN;
    END_IF;
  END_METHOD

  METHOD PUBLIC SetPidKd : BOOL
    VAR_INPUT
      rKd : REAL;
    END_VAR
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := 15, VALUE := REAL_TO_LREAL(rKd) );

    SetPidKd := fbHYD_WriteParameter.Done;
  END_METHOD

  METHOD PUBLIC SetPidKp : BOOL
    VAR_INPUT
      rKp : REAL;
    END_VAR
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := 11, VALUE := REAL_TO_LREAL(rKp) );

    SetPidKp := fbHYD_WriteParameter.Done;
  END_METHOD

  METHOD PUBLIC SetPidKi : BOOL
    VAR_INPUT
      rKi : REAL;
    END_VAR
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := 14, VALUE := REAL_TO_LREAL(rKi) );

    SetPidKi := fbHYD_WriteParameter.Done;
  END_METHOD

  METHOD PRIVATE CalcStartVelocity : REAL
    (*----------------------------------------------------------------------
      V0 决策: 全部取命令侧锁存值, 不依赖速度传感器
        上段位置段 → 上段终点速度; 上段速度段 → 上段目标速度; 其它 → 0
    ------------------------------------------------------------------------*)

    IF bSegVelValid THEN
        CASE uiLastSegMode OF
            1:  rVel0 := rEndVelLastSeg;
            2:  rVel0 := rVelLastSeg;
        ELSE
            rVel0 := 0.0;
        END_CASE
    ELSE
        rVel0 := 0.0;
    END_IF

    CalcStartVelocity := rVel0;
  END_METHOD

  METHOD PRIVATE CalcStopDeceleration : BOOL
    VAR
      rVelStopRef : REAL;
    END_VAR
    (*----------------------------------------------------------------------
      停止减速度: 基于命令侧速度, uiDec 内停稳
      向下过渡期间设定速度高于 V1, 取包络 MAX(|V0|,|V1|)
    ------------------------------------------------------------------------*)

    IF bSegVelValid AND (rMaxSpeedDir > 0.0) THEN
        IF bTransDecel THEN
            rVelStopRef := rVelTransMax;
        ELSE
            rVelStopRef := ABS(rVelLastSeg);
        END_IF
        rStopDec := LIMIT( rDecMin,
                           rVelStopRef
                             / (MAX(UINT_TO_REAL(uiDec), 1.0) * 0.001),
                           rStopDecMax );
    ELSE
        rStopDec := 5000.0;
    END_IF
  END_METHOD

  METHOD PRIVATE ClearSegLatch : BOOL
    (* 停止/急停/复位/报警回到空闲: 段速度链断开, 下次 Start 的 V0=0 *)

    bSegVelValid    := FALSE;
    rVelLastSeg     := 0.0;
    rEndVelLastSeg  := 0.0;
    uiLastSegMode   := 0;
    iDirSignLastSeg := 0;
    rVel0           := 0.0;
    rPresLastCycle  := 0.0;
    bTransDecel     := FALSE;
    rTransTimer     := 0.0;
  END_METHOD

  METHOD PRIVATE CalcSegmentRamp : BOOL
    VAR
      rVel0Signed : REAL;
      rDeltaV : REAL;
      rDeltaVAbs : REAL;
      rDecSegRaw : REAL;
      tAcc : REAL;
      tDec : REAL;
    END_VAR
    (*----------------------------------------------------------------------
      段间斜率计算 (命令侧差值方式), 由主体在 bNewSeg 拍调用
      ----------------------------------------------------------------------
      库规则: setpoint 上升用 ACCELERATION, 下降用 DECELERATION
      - 向上过渡/死区: rDec 直接取段末值 rDecSeg (EndVel=0 时即
        "以本段 dec 减速到 0", 从一开始生效, 无需任何切换)
      - 向下过渡: 触发拍 rDec = |ΔV|/uiDec (精确兑现过渡时间), rDecSeg 暂存;
        速度段由主体 §3.4 直接切换 (fbMoveVel 有 CONTINUOUSUPDATE);
        位置段切换后经 105 重触发锁存 (fbMoveContAbs 无 CONTINUOUSUPDATE)
      压力模式不参与, 由压力斜率通道独立处理
      注意: 局部变量每次调用重新初始化, 跨周期状态留在实例 VAR
    ------------------------------------------------------------------------*)

    tAcc := MAX(UINT_TO_REAL(uiAcc), 1.0) * 0.001;
    tDec := MAX(UINT_TO_REAL(uiDec), 1.0) * 0.001;

    IF uiMode <> 3 THEN
        //--- V0 决策 ---
        rVel0Signed := INT_TO_REAL(iDirSignLastSeg) * CalcStartVelocity();
        rVelSigned  := INT_TO_REAL(iDirSign) * rVel;

        rDeltaV    := rVelSigned - rVel0Signed;
        rDeltaVAbs := MAX(ABS(rDeltaV), rDeltaVDeadZone);

        //--- 段末减速度 rDecSeg ---
        //    位置段: V1 → EndVel (EndVel=0 时 = V1/uiDec)
        //    速度段: V1 → 0 (完备性, 实际停止由 rStopDec 管)
        IF uiMode = 1 THEN
            rDecSegRaw := ABS(rVelSigned - INT_TO_REAL(iDirSign) * rEndVel);
        ELSE
            rDecSegRaw := ABS(rVelSigned);
        END_IF
        rDecSeg := LIMIT( rDecMin, MAX(rDecSegRaw, rDeltaVDeadZone) / tDec, rDecMax );

        //--- 按过渡方向分流 ---
        IF rDeltaV > rDeltaVDeadZone THEN
            // 向上过渡: ACCELERATION 管过渡, rDec 直接取段末值
            rAcc := LIMIT( rAccMin, rDeltaVAbs / tAcc, rAccMax );
            rDec := rDecSeg;
            bTransDecel := FALSE;
            rTransTimer := 0.0;

        ELSIF rDeltaV < -rDeltaVDeadZone THEN
            // 向下过渡: DECELERATION 管过渡, 段末值暂存待切换
            rDec := LIMIT( rDecMin, rDeltaVAbs / tDec, rDecMax );
            rAcc := LIMIT( rAccMin, ABS(rVelSigned) / tAcc, rAccMax );  // 过渡期兜底
            bTransDecel := TRUE;
            rTransTimer := 0.0;
            rTransTime  := rDeltaVAbs / rDec;   // 按钳位后实际减速度

        ELSE
            // ΔV 死区: 无过渡
            rAcc := LIMIT( rAccMin, ABS(rVelSigned) / tAcc, rAccMax );
            rDec := rDecSeg;
            bTransDecel := FALSE;
            rTransTimer := 0.0;
        END_IF

        //--- Jerk: 0 = 不限制 ---
        IF uiJerk = 0 THEN
            rJerk := 0.0;
        ELSE
            rJerk := rAcc / MAX(UINT_TO_REAL(uiJerk), 1.0) * 0.001;
        END_IF

        //--- 过渡期间速度包络 (停止减速参考) ---
        rVelTransMax := MAX(ABS(rVel0Signed), ABS(rVelSigned));
    END_IF

    //--- 锁存本段命令侧参数 ---
    rVelLastSeg     := rVel;
    rEndVelLastSeg  := rEndVel;
    uiLastSegMode   := uiMode;
    iDirSignLastSeg := iDirSign;
    bSegVelValid    := TRUE;
  END_METHOD

  (*=====================================================================
    液压轴工艺层封装功能块
    ----------------------------------------------------------------------
    版本历史:
      V2.4  适配无 CONTINUOUSUPDATE 的 HYD_MoveContinuousAbsolute:
            - fbMoveContAbs 移除 CONTINUOUSUPDATE (库版本无此参数)
            - 位置段 rDec 段末切换改用 105 式重触发:
              切换拍 rDec:=rDecSeg 并强制 iStep:=105 (EXECUTE 一拍下降),
              下一拍上升沿库以新 rDec 从当前状态重规划, 运动不中断;
              复用原代码段链已验证的重触发语义, 零新增机制
            - 速度段不受影响 (fbMoveVel 自带 CONTINUOUSUPDATE)
      V2.3  EndVel=0 段末减速保护: 减速区距离公式 d=(V1²-EndVel²)/(2·rDecSeg)
      V2.2  rDec 双角色分离 (向下过渡 = |ΔV|/uiDec, 段末值 rDecSeg 暂存)
      V2.0  段间差值+命令侧速度换算 (V0 不依赖传感器)
      V1.0  初版: 加速度 = Vmax/uiAcc (已废弃)
  =====================================================================*)

  //=====================================================================
  // 0. 监视调试数据回写 (不参与控制)
  //=====================================================================
  stHydAxisRef.uiDir   := uiDir;
  stHydAxisRef.udiPos  := udiPos;
  stHydAxisRef.uiSpd   := uiSpd;
  stHydAxisRef.uiEndSpd:= uiEndSpd;
  stHydAxisRef.uiPres  := uiPres;

  //=====================================================================
  // 1. 边沿检测 & 基本设定值换算
  //=====================================================================
  rTrigStart(CLK := bStart);
  rTrigStop (CLK := bStop);
  rTrigEStop(CLK := bEStop);
  rTrigReset(CLK := bReset);
  rTrigDone (CLK := fbMoveContAbs.InEndVelocity);

  CASE uiDir OF
      1: iDirSign := 1;
      2: iDirSign := -1;
  ELSE
      iDirSign := 0;
  END_CASE
  iDir := uiDir;

  rPres    := MAX(UINT_TO_REAL(uiPres), 0.0) * 0.1;              // bar
  rPos     := UDINT_TO_REAL(udiPos) * 0.01;                      // mm
  rFlowPercent := LIMIT(5, UINT_TO_REAL(uiSpd)*0.1, 100);        // %

  //=====================================================================
  // 2. 方向相关最大速度 & 液压能力边界
  //=====================================================================
  IF stHydAxisRef.rMaxSpeedExtend > 0.0 THEN
      CASE iDir OF
          1: rMaxSpeedDir := stHydAxisRef.rMaxSpeedExtend;
          2: rMaxSpeedDir := stHydAxisRef.rMaxSpeedRetract;
      ELSE
          rMaxSpeedDir := MIN(stHydAxisRef.rMaxSpeedExtend,
                              stHydAxisRef.rMaxSpeedRetract);
      END_CASE
      rMaxSpeed := rMaxSpeedDir;
  ELSIF stHydAxisRef.rMaxSpeed > 0.0 THEN
      rMaxSpeedDir := stHydAxisRef.rMaxSpeed;
      rMaxSpeed    := rMaxSpeedDir;
  END_IF

  // 方向能力确定后再换算末速度，避免方向切换时沿用上一方向的最大速度。
  rEndVel  := UINT_TO_REAL(uiEndSpd) * 0.1 * 0.01 * rMaxSpeed;   // mm/s
  rVel := UINT_TO_REAL(uiSpd) * 0.1 * 0.01 * rMaxSpeed;          // mm/s

  IF rMaxSpeedDir > 0.0 THEN
      rAccMax     := rMaxSpeedDir / rRampMinTime;
      rAccMin     := rMaxSpeedDir / rRampMaxTime;
      rDecMax     := rAccMax;
      rDecMin     := rAccMin;
      rStopDecMax := rDecMax;
  END_IF

  rVelSigned := INT_TO_REAL(iDirSign) * rVel;

  //=====================================================================
  // 3. 段间斜率计算 —— 主体只保留触发与编排, 算法在 PRIVATE METHOD
  //=====================================================================

  // 3.1 新段触发 → 斜率计算 + 段锁存
  bNewSeg := rTrigStart.Q
             AND (uiMode >= 1) AND (uiMode <= 3)
             AND NOT rTrigEStop.Q
             AND NOT rTrigStop.Q
             AND (iStep = 10 OR iStep = 100 OR iStep = 105);

  IF bNewSeg THEN
      CalcSegmentRamp();
  END_IF

  // 3.2 在线速度更新 (仅速度模式: fbMoveVel 有 CONTINUOUSUPDATE;
  //     位置模式无在线更新能力, 参数变更一律走段切换/105 重触发)
  IF (iStep = 100) AND (uiActiveMode = 2) AND bSegVelValid
     AND ABS(rVelSigned - INT_TO_REAL(iDirSignLastSeg)*rVelLastSeg) > rDeltaVDeadZone THEN
      IF rVelSigned > INT_TO_REAL(iDirSignLastSeg)*rVelLastSeg THEN
          rAcc := LIMIT( rAccMin,
                         ABS(rVelSigned - INT_TO_REAL(iDirSignLastSeg)*rVelLastSeg)
                           / (MAX(UINT_TO_REAL(uiAcc), 1.0) * 0.001),
                         rAccMax );
      ELSE
          rDec := LIMIT( rDecMin,
                         ABS(rVelSigned - INT_TO_REAL(iDirSignLastSeg)*rVelLastSeg)
                           / (MAX(UINT_TO_REAL(uiDec), 1.0) * 0.001),
                         rDecMax );
          rDecSeg := rDec;
          bTransDecel := FALSE;
      END_IF
      rVelLastSeg := rVel;
  END_IF

  // 3.3 停止减速度 (每周期)
  CalcStopDeceleration();

  // 3.4 向下过渡完成检测 → 切换为段末减速度 (V2.4)
  //     速度段: fbMoveVel 有 CONTINUOUSUPDATE, rDec 直接切换生效
  //     位置段: fbMoveContAbs 无 CONTINUOUSUPDATE, 切换 rDec 后强制
  //             iStep:=105 (EXECUTE 一拍下降), 下一拍上升沿库以新 rDec
  //             从当前状态重规划, 运动不中断 —— 复用已验证的段链语义
  //     注: §3.4 在状态机 CASE 之前, 本周期内 105 即被执行
  IF bTransDecel AND (iStep = 100) THEN
      rTransTimer := rTransTimer + rCycleTime;
      CASE uiActiveMode OF
          2:  // 速度段: 达速即切, 计时器兜底 (×1.2 裕量)
              IF fbMoveVel.InVelocity OR (rTransTimer >= rTransTime * 1.2) THEN
                  rDec := rDecSeg;
                  bTransDecel := FALSE;
              END_IF

          1:  // 位置段: 减速区距离公式优先, 计时器兜底; 切换后 105 式重触发
              //     EndVel=0 时 = V1/uiDec, 保证"以本段 dec 减速到 0"
              rDecDist := MAX( (rVel*rVel - rEndVel*rEndVel)
                               / (2.0 * MAX(rDecSeg, 1.0)), 0.0 );
              IF ( ABS(rPos - stHydAxisRef.rActPosition) <= (rDecDist * 1.2) )
                 OR ( rTransTimer >= rTransTime * 1.2 ) THEN
                  rDec := rDecSeg;
                  bTransDecel := FALSE;
                  iStep := 105;      // 重触发: 本拍 EXECUTE 下降, 下拍以新 rDec 锁存
              END_IF
      END_CASE
  END_IF

  //=====================================================================
  // 4. 状态机主逻辑
  //=====================================================================
  CASE iStep OF

      0:  // 轴创建与液压配置
          iMechBaseClass := 0;
          IF IS_BIT_TRUE( SINT_TO_WORD(stHydAxisRef.iMechanismType), 0 ) THEN
              iMechBaseClass := 1;
          END_IF;

          fbCreate( USE_SIMULATION := stHydAxisRef.bSimulation,
                    MECHANISM_TYPE  := iMechBaseClass );

          IF fbCreate.DONE THEN
              siAxisId := fbCreate.AXISID;
              stHydAxisRef.iAxisID := siAxisId;

              SetVelocityToFlowGain( stHydAxisRef.rVelToFlowGain );
              SetPidType( SINT_TO_REAL(stHydAxisRef.iTypePressurePID) );
              SetCylinderConfig( stHydAxisRef.rCylinderExtendArea,
                                 stHydAxisRef.rCylinderRetractArea );

              fbCalcMaxSpeed( stHydAxisRef.rCylinderExtendArea,
                              stHydAxisRef.rCylinderRetractArea,
                              1800, 50,
                              stHydAxisRef.iMechanismType );
              stHydAxisRef.rMaxSpeedExtend  := fbCalcMaxSpeed.rMaxSpeedExtend;
              stHydAxisRef.rMaxSpeedRetract := fbCalcMaxSpeed.rMaxSpeedRetract;

              IF stHydAxisRef.iMechanismType >= 100 THEN
                  SetPumpConfig( 50, 1700 );
              END_IF;

              IF iMechBaseClass = 1 THEN
                  iStep := 1;
              ELSE
                  iStep := 10;
              END_IF;

          ELSIF fbCreate.ERROR THEN
              bAlarm := TRUE;
              dwAlarmID := 16#00000001;
              iStep := 900;
          END_IF

      1:  // 肘杆机构配置
          bConfig := SetToggleMechanism( siAxisId, stHydAxisRef.iMechanismType );
          IF bConfig THEN
              iStep := 10;
          END_IF

      10: // 空闲
          bCmdExecute := FALSE;
          bDone       := FALSE;

          IF rTrigEStop.Q THEN
              iStep := 300;
          ELSIF rTrigStop.Q THEN
              iStep := 200;
          ELSIF rTrigReset.Q AND bAlarm THEN
              iStep := 400;
          ELSIF rTrigStart.Q AND (uiMode >= 1) AND (uiMode <= 3) THEN
              uiActiveMode := uiMode;   // 段斜坡已在 §3.1 method 内计算完毕
              bCmdExecute  := TRUE;
              iStep        := 100;
          END_IF

      100: // 运行中
          bCmdExecute := TRUE;

          CASE uiActiveMode OF
              1:  bFBError := fbMoveContAbs.ERROR;  wFBErrorID := fbMoveContAbs.ERRORID;
              2:  bFBError := fbMoveVel.ERROR;      wFBErrorID := fbMoveVel.ERRORID;
              3:  bFBError := fbPresHdl.ERROR;      wFBErrorID := fbPresHdl.ERRORID;
          END_CASE

          IF bFBError THEN
              bAlarm := TRUE;
              dwAlarmID := WORD_TO_DWORD(wFBErrorID);
              iStep := 900;
              bCmdExecute := FALSE;

          ELSIF rTrigStart.Q THEN
              IF uiMode = uiActiveMode THEN
                  iStep := 105;
              ELSIF (uiMode >= 1) AND (uiMode <= 3) THEN
                  uiActiveMode := uiMode;
              END_IF
          END_IF

          IF rTrigEStop.Q THEN
              iStep := 300;
          ELSIF rTrigStop.Q THEN
              iStep := 200;
          END_IF

      105: // 运行过渡步 (EXECUTE 一拍下降 → 100 上升沿锁存新参数)
           // 用途: ① 同模式新段切换(原逻辑) ② 段末 rDec 重锁存(V2.4)
           // EXECUTE 下降沿不中止运动, 库按既有规划继续
          bCmdExecute := FALSE;
          bDone       := FALSE;
          IF rTrigEStop.Q THEN
              iStep := 300;
          ELSIF rTrigStop.Q THEN
              iStep := 200;
          ELSE
              IF rTrigStart.Q AND (uiMode >= 1) AND (uiMode <= 3) THEN
                  uiActiveMode := uiMode;
              END_IF
              iStep := 100;
          END_IF

      200: // 正常停止
          IF fbStop.DONE THEN
              bCmdExecute := FALSE;
              iStep := 10;
          ELSIF fbStop.ERROR THEN
              bAlarm := TRUE;
              dwAlarmID := WORD_TO_DWORD(fbStop.ERRORID);
              iStep := 900;
          END_IF

      300: // 急停
          IF fbEStop.DONE THEN
              bAlarm := TRUE;
              dwAlarmID := 16#0000FFFF;
              bCmdExecute := FALSE;
              iStep := 900;
          ELSIF fbEStop.ERROR THEN
              bAlarm := TRUE;
              dwAlarmID := WORD_TO_DWORD(fbEStop.ERRORID);
              bCmdExecute := FALSE;
              iStep := 900;
          END_IF

      400: // 报警复位
          IF fbReset.DONE THEN
              bAlarm    := FALSE;
              dwAlarmID := 0;
              bCmdExecute := FALSE;
              iStep := 10;
          ELSIF fbReset.ERROR THEN
              bAlarm := TRUE;
              dwAlarmID := WORD_TO_DWORD(fbReset.ERRORID);
              iStep := 900;
          END_IF

      900: // 报警等待
          IF rTrigReset.Q THEN
              iStep := 400;
          END_IF

  END_CASE;

  //=====================================================================
  // 5. 段锁存生命周期: 回到空闲时速度链断开
  //=====================================================================
  IF iStep = 10 AND (iStepPrev = 200 OR iStepPrev = 300
                     OR iStepPrev = 400 OR iStepPrev = 900) THEN
      ClearSegLatch();
  END_IF
  iStepPrev := iStep;

  //=====================================================================
  // 6. 压力斜率动态计算 (模式3, 独立通道)
  //=====================================================================
  IF (iStep = 100 OR iStep = 105) AND (uiActiveMode = 3) THEN
      IF rTrigStart.Q THEN
          rPressureRampRate := CalcPresRampRate(
                  rTargetPres := rPres,
                  rPrevPres   := rPresLastCycle,
                  uiTimeMs    := uiPressureAccTime );
          rPresLastCycle := rPres;
      ELSIF ABS(rPres - rPresLastCycle) > 0.1 THEN
          rPressureRampRate := CalcPresRampRate(
                  rTargetPres := rPres,
                  rPrevPres   := rPresLastCycle,
                  uiTimeMs    := uiPressureAccTime );
          rPresLastCycle := rPres;
      END_IF
  END_IF

  //=====================================================================
  // 7. 底层 FB 统一调用区
  //=====================================================================
  rCycle := rCycle + 0.001;

  fbHYD_SetAxisFeedback(
      AXISID       := siAxisId,
      ENABLE       := 1,
      ACT_POSITION := stHydAxisRef.rActPosition,
      ACT_VELOCITY := stHydAxisRef.rActVelocity,
      ACT_FLOW     := 0,
      ACT_PRESSURE := stHydAxisRef.rActPressure,
      TIMESTAMP    := rCycle );

  // 位置模式: 库无 CONTINUOUSUPDATE, 参数仅在 EXECUTE 上升沿锁存;
  // 运行中参数变更(段末 rDec 切换)由 §3.4 → 105 重触发实现
  fbMoveContAbs(
      AXISID     := siAxisId,
      EXECUTE    := (uiActiveMode = 1) AND bCmdExecute,
      POSITION   := rPos,
      VELOCITY   := rVel,
      ENDVELOCITY:= rEndVel,
      ENDVELOCITYDIRECTION := iDir,
      ACCELERATION := rAcc,
      DECELERATION := rDec,
      JERK         := rJerk,
      ADAPTENDVELTOAVOIDOVERSHOOT := TRUE,
      PRESSURELIMIT := rPres );

  // 速度模式: 有 CONTINUOUSUPDATE, rDec/rAcc 在线更新直接生效
  fbMoveVel(
      AXISID     := siAxisId,
      EXECUTE    := (uiActiveMode = 2) AND bCmdExecute,
      CONTINUOUSUPDATE := TRUE,
      VELOCITY   := rVel,
      ACCELERATION := rAcc,
      DECELERATION := rDec,
      JERK       := rJerk,
      DIRECTION  := iDir,
      BUFFERMODE := 0,
      PRESSURELIMIT := rPres );

  fbPresHdl(
      AXISID     := siAxisId,
      EXECUTE    := (uiActiveMode = 3) AND bCmdExecute,
      CONTINUOUSUPDATE := TRUE,
      PRESSURE   := rPres,
      PRESSURERAMPRATE := rPressureRampRate,
      DURATION   := 0.0,
      FLOWLIMITPERCENT := rFlowPercent,
      BUFFERMODE := 0 );

  IF iStep = 200 THEN
      fbStop( AXISID := siAxisId, EXECUTE := TRUE,  DECELERATION := rStopDec );
  ELSE
      fbStop( AXISID := siAxisId, EXECUTE := FALSE, DECELERATION := rStopDec );
  END_IF

  IF iStep = 300 THEN
      fbEStop( AXISID := siAxisId, EXECUTE := TRUE,  DECELERATION := 99999.0 );
  ELSE
      fbEStop( AXISID := siAxisId, EXECUTE := FALSE, DECELERATION := 99999.0 );
  END_IF

  IF iStep = 400 THEN
      fbReset( AXISID := siAxisId, EXECUTE := TRUE );
  ELSE
      fbReset( AXISID := siAxisId, EXECUTE := FALSE );
  END_IF

  //=====================================================================
  // 8. 仿真反馈 & 轴状态读取 (仅监视/诊断)
  //=====================================================================
  IF stHydAxisRef.bSimulation THEN
      fbReadFB( AXISID := siAxisId, ENABLE := TRUE );
      IF fbReadFB.VALID THEN
          stHydAxisRef.rPosition  := fbReadFB.POSITION;
          stHydAxisRef.rVelocity := fbReadFB.VELOCITY;
          stHydAxisRef.rFlow      := fbReadFB.FLOW;
          stHydAxisRef.rPressure  := fbReadFB.PRESSURE;
      END_IF
  END_IF

  fbHYD_ReadStatus( AXISID := siAxisId, ENABLE := TRUE, STATE => iAxisState );
  stHydAxisRef.iStep := iStep;

  //=====================================================================
  // 9. 状态输出汇总
  //=====================================================================
  CASE iStep OF
      10:
          bBusy := FALSE;
          bDone := FALSE;
          bInVelocity := FALSE;
          bInPressure := FALSE;

      100, 105:
          bBusy := TRUE;
          CASE uiActiveMode OF
              1:  bDone := fbMoveContAbs.InEndVelocity;
              2:  bDone := fbMoveVel.InVelocity;
              3:  bDone := fbPresHdl.INPRESSURE;
              ELSE
                  bDone := FALSE;
          END_CASE
          bInVelocity := fbMoveVel.InVelocity;
          bInPressure := fbPresHdl.INPRESSURE;

      200, 300, 400:
          bBusy := TRUE;
          bDone := FALSE;

  ELSE
      bBusy := FALSE;
      bDone := FALSE;
  END_CASE;

  bStandStillOrAllowUpdate := NOT bBusy OR (iStep = 100) OR (iStep = 105);
END_FUNCTION_BLOCK
