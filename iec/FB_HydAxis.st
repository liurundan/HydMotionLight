FUNCTION_BLOCK FB_HydAxis
  VAR_IN_OUT
    stHydAxisRef : ST_HydAxisRef;
  END_VAR
  VAR_INPUT
    bStart : BOOL;
    bStop : BOOL;
    bEStop : BOOL;
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
    siAxisId : SINT;
    rTrigStart : R_TRIG;
    rTrigStop : R_TRIG;
    rTrigEStop : R_TRIG;
    rTrigReset : R_TRIG;
    rTrigDone : R_TRIG;
    uiActiveMode : UINT;
    bCmdExecute : BOOL;
    iDir : SINT;
    iMechBaseClass : SINT := 0;
    rPos : REAL;
    rVel : REAL;
    rEndVel : REAL;
    rPres : REAL;
    rAcc : REAL;
    rDec : REAL;
    rStopDec : REAL := 5000.0;
    rJerk : REAL;
    rFlowPercent : REAL;
    rPressureRampRate : REAL;
    rPresLastCycle : REAL := 0;
    bFBError : BOOL;
    wFBErrorID : WORD;
    iDoneCnt : INT := 0;
    iAxisState : UINT := 0;
    rMaxSpeed : REAL;
    rPosition : REAL;
    rVeloctity : REAL;
    rFlow : REAL;
    rPerssure : REAL;
    rCycle : REAL;
    fbHYD_ReadStatus : HYD_ReadStatus;
    bConfig : BOOL;

    //--- 段间速度链与方向锁存 -------------------------------------------
    // 命令侧语义：V0/Vset/Vend 恒为非负速度幅值，方向由 iDir* 独立承载。
    // 不采用实际速度反馈重建 V0，避免液压压缩/负载造成命令斜率抖动。
    iDirLastSeg : SINT := 0;      // 上一段方向
    iDirActive : SINT := 0;       // 当前实际下发给底层 FB 的方向
    iDirTarget : SINT := 0;       // 当前工艺目标方向
    rMaxSpeedDir : REAL := 0.0;   // 方向相关最大速度（物理单位/秒）
    rVelLastSeg : REAL := 0.0;    // 上一段 Vset 幅值
    rEndVelLastSeg : REAL := 0.0; // 上一段 Vend 幅值（位置段）
    uiLastSegMode : UINT := 0;    // 1=位置 2=速度 3=压力
    bSegVelValid : BOOL := FALSE; // 速度链是否有效
    bDirectionChange : BOOL := FALSE;
    bDecelToZero : BOOL := FALSE; // 换向前零速阶段
    bTransDecel : BOOL := FALSE;  // 段间过渡减速尚未完成
    bNewSeg : BOOL := FALSE;
    bPendingReverse : BOOL := FALSE;    // 状态机跃迁挂起（CASE 后统一应用）
    bPendingRetrigger : BOOL := FALSE;
    rVel0 : REAL := 0.0;          // 本段起始速度幅值（监视用）
    rVel0Magnitude : REAL := 0.0;
    rDeltaV : REAL := 0.0;
    rDecSeg : REAL := 200.0;      // 本段末端减速度 |Vset-Vend|/tDec
    rTransTime : REAL := 0.0;
    rTransTimer : REAL := 0.0;
    rDecDist : REAL := 0.0;
    rPosActive : REAL := 0.0;     // 段起点锁存的命令值
    rVelActive : REAL := 0.0;
    rEndVelActive : REAL := 0.0;
    rPresActive : REAL := 0.0;
    rCycleTime : REAL := 0.001;   // 与主循环 rCycle 一致的扫描周期
    rMinRampTime : REAL := 100.0; // 固定 100 ms 时间下限
    rDeltaVDeadZone : REAL := 0.001;
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
    // 默认返回值，防止异常
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
    CalcPresRampRate := rDeltaPres / rTimeSec; // 单位: bar/s
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

    IF iAxis <> -1 AND iType = 1 THEN
      SetToggleMechanism := TRUE;
    ELSE
      SetToggleMechanism := NOT fbHYD_ConfigureToggleMechanism.ERROR AND fbHYD_ConfigureToggleMechanism.DONE;
    END_IF;
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

  METHOD PUBLIC SetPidKd : BOOL
    VAR_INPUT
      rKd : REAL;
    END_VAR
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
    fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE,
                          PARAMETERNUMBER := 15, VALUE := REAL_TO_LREAL(rKd) );

    SetPidKd := fbHYD_WriteParameter.Done;
  END_METHOD

  //=====================================================================
  // 段间速度斜率规划：私有 method 边界
  //=====================================================================

  // 下一段 V0：位置段取上一段 Vend，速度段取上一段 Vset，压力段断链取 0。
  // 幅值是上一段已换算出的物理速度，不用下一段的最大速度重新归一化。
  METHOD PRIVATE CalcStartVelocity : REAL
    IF NOT bSegVelValid THEN
        CalcStartVelocity := 0.0;
        RETURN;
    END_IF;

    CASE uiLastSegMode OF
        1: CalcStartVelocity := MAX(rEndVelLastSeg, 0.0);
        2: CalcStartVelocity := MAX(rVelLastSeg, 0.0);
    ELSE
        CalcStartVelocity := 0.0;
    END_CASE;
  END_METHOD

  // 离开运动生命周期时统一清空命令侧锁存。
  // 注意：state 150 的 fbStop.DONE 之前不得调用，反向段仍需挂起的 V0/方向。
  METHOD PRIVATE ClearSegLatch : BOOL
    bSegVelValid := FALSE;
    bDirectionChange := FALSE;
    bDecelToZero := FALSE;
    bTransDecel := FALSE;
    bNewSeg := FALSE;
    bPendingReverse := FALSE;
    bPendingRetrigger := FALSE;
    iDirLastSeg := 0;
    iDirActive := 0;
    iDirTarget := 0;
    rVelLastSeg := 0.0;
    rEndVelLastSeg := 0.0;
    uiLastSegMode := 0;
    rVel0 := 0.0;
    rVel0Magnitude := 0.0;
    rDeltaV := 0.0;
    rTransTime := 0.0;
    rTransTimer := 0.0;
    ClearSegLatch := TRUE;
  END_METHOD

  // 由段间真实速度幅值差 |Vset-V0| 与工艺时间 uiAcc/uiDec(ms) 求底层斜率。
  // 液压泵控轴使用"正速度幅值 + 独立方向"，本 method 不产生任何带符号速度。
  METHOD PRIVATE CalcSegmentRamp : BOOL
    VAR
      tAcc : REAL;
      tDec : REAL;
    END_VAR
    CalcSegmentRamp := FALSE;

    // 1. 100 ms 时间下限，毫秒 -> 秒；uiAcc/uiDec=0 不产生除零或无限斜率
    tAcc := MAX(UINT_TO_REAL(uiAcc), rMinRampTime) * 0.001;
    tDec := MAX(UINT_TO_REAL(uiDec), rMinRampTime) * 0.001;

    // 2. 命令侧 V0（非负幅值），并钳在本方向能力边界内
    rVel0Magnitude := CalcStartVelocity();
    rVel0Magnitude := MIN(rVel0Magnitude, MAX(rMaxSpeedDir, 0.0));
    rVel0 := rVel0Magnitude;
    iDirTarget := iDir;

    // 3. 压力段断开速度链，后续运动段从 V0=0 重新开始
    IF uiMode = 3 THEN
        ClearSegLatch();
        CalcSegmentRamp := TRUE;
        RETURN;
    END_IF;

    // 4. 能力边界无效：零速度零斜率，且不污染 rStopDec
    IF rMaxSpeedDir <= 0.0 THEN
        rVel := 0.0;
        rEndVel := 0.0;
        rVelActive := 0.0;
        rEndVelActive := 0.0;
        rAcc := 0.0;
        rDec := 0.0;
        RETURN;
    END_IF;

    // 5. 本段末端减速度：位置段 Vset->Vend，速度段 Vset->0。
    //    必须先于同向分支计算，使加速段的 DECELERATION 直接取该值。
    IF uiMode = 1 THEN
        rDecSeg := MAX(ABS(rVel - rEndVel) / tDec, 1.0);
    ELSE
        rDecSeg := MAX(rVel / tDec, 1.0);
    END_IF;

    // 6. 方向判断先于幅值差比较：液压无反向制动能力，必须先减到零再换向
    bDirectionChange := bSegVelValid
                        AND (rVel0Magnitude > rDeltaVDeadZone)
                        AND (iDirLastSeg <> 0)
                        AND (iDirTarget <> 0)
                        AND (iDirLastSeg <> iDirTarget);

    IF bDirectionChange THEN
        // 保持旧方向，按 uiDec 把幅值降到零；不计算带符号的 Vset-V0
        bDecelToZero := TRUE;
        bTransDecel := FALSE;
        iDirActive := iDirLastSeg;
        rDec := MAX(rVel0Magnitude / tDec, 1.0);
        rAcc := MAX(rVelActive / tAcc, 1.0);
        rTransTime := rVel0Magnitude / MAX(rDec, 1.0);
        rTransTimer := 0.0;
        bPendingReverse := TRUE;
    ELSE
        // 同向/首段：每个新段都要重算，不能因速度链已建立而跳过
        bDecelToZero := FALSE;
        bPendingReverse := FALSE;
        IF NOT bSegVelValid OR (iDirActive = 0) THEN
            iDirActive := iDirTarget;
        END_IF;

        rDeltaV := rVel - rVel0Magnitude;
        IF rDeltaV > rDeltaVDeadZone THEN
            rAcc := rDeltaV / tAcc;
            rDec := rDecSeg;
            bTransDecel := FALSE;
        ELSIF rDeltaV < -rDeltaVDeadZone THEN
            // 段间过渡减速先占用唯一 DECELERATION 输入，完成后再切到 rDecSeg
            rDec := ABS(rVel - rVel0Magnitude) / tDec;
            rAcc := MAX(rVel / tAcc, 1.0);
            bTransDecel := TRUE;
            rTransTime := ABS(rVel - rVel0Magnitude) / MAX(rDec, 1.0);
            rTransTimer := 0.0;
        ELSE
            rAcc := MAX(rVel / tAcc, 1.0);
            rDec := rDecSeg;
            bTransDecel := FALSE;
        END_IF;
    END_IF;

    // 7. 锁存当前段，形成下一段的 V0 与方向判据
    rVelLastSeg := MAX(rVel, 0.0);
    rEndVelLastSeg := MAX(rEndVel, 0.0);
    uiLastSegMode := uiMode;
    iDirLastSeg := iDirTarget;
    bSegVelValid := TRUE;
    CalcSegmentRamp := TRUE;
  END_METHOD

  // 仅作监视调试使用
  stHydAxisRef.uiDir := uiDir;
  stHydAxisRef.udiPos := udiPos;
  stHydAxisRef.uiSpd := uiSpd;
  stHydAxisRef.uiEndSpd := uiEndSpd;
  stHydAxisRef.uiPres := uiPres;

  //--- 1. 边沿检测 & 参数映射 ---
  rTrigStart(CLK := bStart);
  rTrigStop(CLK := bStop);
  rTrigEStop(CLK := bEStop);
  rTrigReset(CLK := bReset);
  rTrigDone(CLK := fbMoveContAbs.InEndVelocity);

  // 1.1 方向相关最大速度必须在速度百分比换算之前选定。
  //     1=伸出 Extend，2=缩回 Retract，0/3=自动/保持沿用通用 rMaxSpeed。
  //     方向能力未标定(<=0)时回退通用最大速度，避免换算出零速度。
  CASE uiDir OF
      1:  iDir := 1; rMaxSpeedDir := stHydAxisRef.rMaxSpeedExtend;
      2:  iDir := 2; rMaxSpeedDir := stHydAxisRef.rMaxSpeedRetract;
  ELSE
      iDir := 0; rMaxSpeedDir := stHydAxisRef.rMaxSpeed;
  END_CASE;

  IF rMaxSpeedDir <= 0.0 THEN
      rMaxSpeedDir := MAX(stHydAxisRef.rMaxSpeed, 0.0);
  END_IF;
  rMaxSpeed := rMaxSpeedDir;

  // 1.2 速度恒为正幅值；缩回由 iDirActive 单独下发，绝不使用负速度。
  //     uiSpd/uiEndSpd 单位为 0.1%，故 0.1*0.01 = 0.001。
  rPres    := MAX(UINT_TO_REAL(uiPres), 0.0) * 0.1;   // 压力设定 bar
  rVel     := LIMIT(0.0, UINT_TO_REAL(uiSpd) * 0.001 * rMaxSpeedDir, rMaxSpeedDir);
  rEndVel  := LIMIT(0.0, UINT_TO_REAL(uiEndSpd) * 0.001 * rMaxSpeedDir, rMaxSpeedDir);
  rPos     := UDINT_TO_REAL(udiPos) * 0.01;           // 位置设定
  rFlowPercent := UINT_TO_REAL(uiSpd) * 0.1;
  rFlowPercent := LIMIT(5.0, rFlowPercent, 100.0);

  // 1.3 兼容初值只在速度链未建立时计算。
  //     一旦 CalcSegmentRamp 锁存了段间斜率，本段内不得被每周期覆盖。
  IF NOT bSegVelValid THEN
      IF rMaxSpeedDir > 0.0 THEN
          rAcc := MAX(rMaxSpeedDir / (MAX(UINT_TO_REAL(uiAcc), rMinRampTime) * 0.001), 1.0);
          rDec := MAX(rMaxSpeedDir / (MAX(UINT_TO_REAL(uiDec), rMinRampTime) * 0.001), 1.0);
      ELSE
          rAcc := 0.0;
          rDec := 0.0;
      END_IF;
  END_IF;
  IF uiJerk = 0 THEN rJerk := 0.0; ELSE rJerk := rMaxSpeedDir / UINT_TO_REAL(uiJerk); END_IF;

  // rStopDec 只接受更大的有效减速度，段间斜率计算失败不会使其变零
  IF rDec > 200.0 THEN
      rStopDec := rDec;
  END_IF;

  //--- 1.5 新段速度斜率规划 ---
  // 1.5.1 新段触发：只在段起点锁存命令值，段内外部输入不再改写运动 FB 参数。
  //       换向减速(state 150)期间不接受新段，防止覆盖挂起的方向/V0。
  bNewSeg := rTrigStart.Q
             AND (uiMode >= 1) AND (uiMode <= 3)
             AND NOT rTrigStop.Q
             AND NOT rTrigEStop.Q
             AND (uiMode = 3 OR rMaxSpeedDir > 0.0)
             AND (iStep = 10 OR iStep = 100 OR iStep = 105);

  IF bNewSeg THEN
      rPosActive := rPos;
      rVelActive := rVel;
      rEndVelActive := rEndVel;
      rPresActive := rPres;
      CalcSegmentRamp();
  END_IF;

  //--- 2. 状态机主逻辑 ---
  CASE iStep OF

      0: // 初始化
          iMechBaseClass := 0;

          IF IS_BIT_TRUE( SINT_TO_WORD(stHydAxisRef.iMechanismType),0 ) THEN
            iMechBaseClass := 1;
          END_IF;
          fbCreate(USE_SIMULATION := stHydAxisRef.bSimulation, MECHANISM_TYPE:= iMechBaseClass);
          IF fbCreate.DONE THEN
              siAxisId := fbCreate.AXISID;
              stHydAxisRef.iAxisID := siAxisId;
              SetVelocityToFlowGain(stHydAxisRef.rVelToFlowGain);
              SetPidType( SINT_TO_REAL(stHydAxisRef.iTypePressurePID) );
              SetCylinderConfig(stHydAxisRef.rCylinderExtendArea, stHydAxisRef.rCylinderRetractArea );
              fbCalcMaxSpeed(stHydAxisRef.rCylinderExtendArea, stHydAxisRef.rCylinderRetractArea, 1800, 50, stHydAxisRef.iMechanismType );

              stHydAxisRef.rMaxSpeedExtend  := fbCalcMaxSpeed.rMaxSpeedExtend;
              stHydAxisRef.rMaxSpeedRetract := fbCalcMaxSpeed.rMaxSpeedRetract;

              IF stHydAxisRef.iMechanismType >= 100 THEN // horizontal machine
                SetPumpConfig(50, 1700);
              END_IF;

              IF iMechBaseClass = 1 THEN
                iStep := 1;
              ELSE
                iStep := 10;
              END_IF;

          ELSIF fbCreate.ERROR THEN
              bAlarm := TRUE; dwAlarmID := 16#00000001; iStep := 900;
          END_IF;

      1:
          bConfig := SetToggleMechanism(siAxisId, stHydAxisRef.iMechanismType );
          IF bConfig THEN
            iStep := 10;
          END_IF;


      10: // 空闲
          bCmdExecute := FALSE;
          bDone := FALSE;
          IF rTrigEStop.Q THEN
              iStep := 300;
          ELSIF rTrigStop.Q THEN
              iStep := 200;
          ELSIF rTrigReset.Q AND bAlarm THEN
              iStep := 400;
          ELSIF rTrigStart.Q AND uiMode >= 1 AND uiMode <= 3
                AND (uiMode = 3 OR rMaxSpeedDir > 0.0) THEN
              uiActiveMode := uiMode;
              bCmdExecute := TRUE;
              iStep := 100;
          END_IF;

      100: // 运行中
          bCmdExecute := TRUE;

          // 1. 错误监测
          CASE uiActiveMode OF
              1: bFBError := fbMoveContAbs.ERROR; wFBErrorID := fbMoveContAbs.ERRORID;
              2: bFBError := fbMoveVel.ERROR; wFBErrorID := fbMoveVel.ERRORID;
              3: bFBError := fbPresHdl.ERROR; wFBErrorID := fbPresHdl.ERRORID;
          END_CASE;

          IF bFBError THEN
              bAlarm := TRUE; dwAlarmID := wFBErrorID; iStep := 900;
              ClearSegLatch();

          // 2. 段切换触发：同模式与跨模式都必须经过 105 产生 EXECUTE 上升沿
          ELSIF rTrigStart.Q THEN
              IF uiMode = uiActiveMode THEN
                  iStep := 105;
              ELSIF uiMode >= 1 AND uiMode <= 3 THEN
                  uiActiveMode := uiMode;
                  iStep := 105;
              END_IF;
          END_IF;

          // 3. 中断监测
          IF rTrigEStop.Q THEN iStep := 300;
          ELSIF rTrigStop.Q THEN iStep := 200;
          END_IF;

      105: // 运行过渡步
          bCmdExecute := FALSE;
          bDone := FALSE;
          // 过渡扫描不得吞掉停止/急停边沿；新段参数已在 1.5 节统一锁存
          IF rTrigEStop.Q THEN
              iStep := 300;
          ELSIF rTrigStop.Q THEN
              iStep := 200;
          ELSIF rTrigStart.Q AND (uiMode >= 1) AND (uiMode <= 3) THEN
              iStep := 100;
          ELSE
              iStep := 100;
          END_IF;

      150: // 反向减速到零（液压换向保护）
          // 保持 iDirActive 为上一段方向，HYD_Stop 按 uiDec 导出的斜率减速；
          // 零速完成前不得把方向参数切到目标方向。
          bCmdExecute := FALSE;
          IF fbStop.DONE THEN
              bDecelToZero := FALSE;
              bDirectionChange := FALSE;
              iDirActive := iDirTarget;
              iDirLastSeg := iDirTarget;
              rVel0 := 0.0;
              rDec := rDecSeg;
              bTransDecel := FALSE;
              iStep := 105;
          ELSIF fbStop.ERROR THEN
              bAlarm := TRUE; dwAlarmID := fbStop.ERRORID; iStep := 900;
              ClearSegLatch();
          END_IF;

      200: // 正常停止
          IF fbStop.DONE THEN
              ClearSegLatch();
              iStep := 10;
              bCmdExecute := FALSE;
          ELSIF fbStop.ERROR THEN
              bAlarm := TRUE; dwAlarmID := fbStop.ERRORID; iStep := 900;
              ClearSegLatch();
          END_IF;

      300: // 急停
          IF fbEStop.DONE THEN
              bAlarm := TRUE; dwAlarmID := 16#0000FFFF; iStep := 900;
              bCmdExecute := FALSE;
              ClearSegLatch();
          ELSIF fbEStop.ERROR THEN
              bAlarm := TRUE; dwAlarmID := fbEStop.ERRORID; iStep := 900;
              bCmdExecute := FALSE;
              ClearSegLatch();
          END_IF;

      400: // 报警复位
          IF fbReset.DONE THEN
              ClearSegLatch();
              bAlarm := FALSE; dwAlarmID := 0; iStep := 10;
              bCmdExecute := FALSE;
          ELSIF fbReset.ERROR THEN
              bAlarm := TRUE; dwAlarmID := fbReset.ERRORID; iStep := 900;
              ClearSegLatch();
          END_IF;

      900: // 报警等待
          IF rTrigReset.Q THEN
              iStep := 400;
          END_IF;

  END_CASE;

  //--- 2.5 过渡状态覆写 ---
  // 放在 CASE 之后，避免被状态机自身的 iStep 跃迁覆盖。
  IF bPendingReverse THEN
      bPendingReverse := FALSE;
      iStep := 150;
  END_IF;

  //--- 2.6 在线过渡减速切换 ---
  // 段间过渡减速完成后，把唯一的 DECELERATION 输入交给本段末端减速度。
  // 速度段依赖 CONTINUOUSUPDATE 在线改值；位置段需借助 105 让 FB 重新锁存。
  IF bTransDecel AND (iStep = 100) THEN
      rTransTimer := rTransTimer + rCycleTime;
      CASE uiActiveMode OF
          2:
              IF fbMoveVel.InVelocity OR (rTransTimer >= rTransTime * 1.2) THEN
                  rDec := rDecSeg;
                  bTransDecel := FALSE;
              END_IF;
          1:
              rDecDist := MAX((rVelActive * rVelActive - rEndVelActive * rEndVelActive)
                              / (2.0 * MAX(rDecSeg, 1.0)), 0.0);
              IF (ABS(rPosActive - stHydAxisRef.rActPosition) <= rDecDist * 1.2) OR
                 (rTransTimer >= rTransTime * 1.2) THEN
                  rDec := rDecSeg;
                  bTransDecel := FALSE;
                  bPendingRetrigger := TRUE;
              END_IF;
      END_CASE;
  END_IF;

  IF bPendingRetrigger THEN
      bPendingRetrigger := FALSE;
      iStep := 105;
  END_IF;

  //--- 压力斜率动态计算逻辑  ---
  // 区分两种情形：
  // 1. 段切换/模式切入时：伴随 rTrigStart.Q，计算一次斜率
  // 2. 连续更新模式下：无 Start 信号，只要目标压力变化，立即计算新斜率
  IF (iStep = 100 OR iStep = 105) AND uiActiveMode = 3 THEN
      // 情形1：Start触发 (新模式/新段进入)
      IF rTrigStart.Q THEN
          rPressureRampRate := CalcPresRampRate(rTargetPres := rPres, rPrevPres := rPresLastCycle, uiTimeMs := uiPressureAccTime);
          rPresLastCycle := rPres; // 锁存基准
      // 情形2：连续更新检测 (目标压力被外部修改)
      ELSIF ABS(rPres - rPresLastCycle) > 0.1 THEN
          rPressureRampRate := CalcPresRampRate(rTargetPres := rPres, rPrevPres := rPresLastCycle, uiTimeMs := uiPressureAccTime);
          rPresLastCycle := rPres;
      END_IF;
  END_IF;

  //--- 3. 底层 FB 统一调用区 ---
  rCycle := rCycle + 0.001;
  fbHYD_SetAxisFeedback(
       AXISID := siAxisId,
       ENABLE := 1,
       ACT_POSITION := stHydAxisRef.rActPosition,
       ACT_VELOCITY := stHydAxisRef.rActVelocity,
       ACT_FLOW := 0,
       ACT_PRESSURE := stHydAxisRef.rActPressure,
       TIMESTAMP := rCycle
  );

  fbMoveContAbs(
      AXISID := siAxisId,
      EXECUTE := (uiActiveMode = 1) AND bCmdExecute,
      POSITION := rPosActive,
      VELOCITY := rVelActive,
      ENDVELOCITY := rEndVelActive,
      ENDVELOCITYDIRECTION := iDirActive,
      ACCELERATION := rAcc,
      DECELERATION := rDec,
      JERK := rJerk,
      DIRECTION := iDirActive,
      ADAPTENDVELTOAVOIDOVERSHOOT := TRUE,
      PRESSURELIMIT := rPresActive
  );

  fbMoveVel(
      AXISID := siAxisId, EXECUTE := (uiActiveMode = 2) AND bCmdExecute,
      CONTINUOUSUPDATE := TRUE, VELOCITY := rVelActive,
      ACCELERATION := rAcc, DECELERATION := rDec, JERK := rJerk, DIRECTION := iDirActive, BUFFERMODE := 0,PRESSURELIMIT := rPresActive);

  fbPresHdl(
      AXISID := siAxisId,
      EXECUTE := (uiActiveMode = 3) AND bCmdExecute,
      CONTINUOUSUPDATE := TRUE,
      PRESSURE := rPres,
      PRESSURERAMPRATE := rPressureRampRate,
      DURATION := 0.0,
      FLOWLIMITPERCENT:=rFlowPercent,
      BUFFERMODE := 0);

  IF iStep = 150 THEN
      fbStop(AXISID := siAxisId, EXECUTE := TRUE, DECELERATION := rDec);
  ELSIF iStep = 200 THEN
      fbStop(AXISID := siAxisId, EXECUTE := TRUE, DECELERATION := rStopDec);
  ELSE
      fbStop(AXISID := siAxisId, EXECUTE := FALSE, DECELERATION := rStopDec);
  END_IF;

  IF iStep = 300 THEN
      fbEStop(AXISID := siAxisId, EXECUTE := TRUE, DECELERATION := 99999.0);
  ELSE
      fbEStop(AXISID := siAxisId, EXECUTE := FALSE, DECELERATION := 99999.0);
  END_IF;

  IF iStep = 400 THEN
      fbReset(AXISID := siAxisId, EXECUTE := TRUE);
  ELSE
      fbReset(AXISID := siAxisId, EXECUTE := FALSE);
  END_IF;

  // 仅作监视调试使用
  IF stHydAxisRef.bSimulation THEN
    fbReadFB(AXISID := siAxisId, ENABLE := TRUE);
    IF fbReadFB.VALID THEN
        stHydAxisRef.rPosition  := fbReadFB.POSITION;
        stHydAxisRef.rVelocity := fbReadFB.VELOCITY;
        stHydAxisRef.rFlow      := fbReadFB.FLOW;
        stHydAxisRef.rPressure  := fbReadFB.PRESSURE;
    END_IF;
  END_IF;

  fbHYD_ReadStatus(  AXISID := siAxisId,  ENABLE := TRUE, STATE => iAxisState );

  stHydAxisRef.iStep := iStep;

  //--- 4. 状态输出汇总 ---
  IF iStep = 10 THEN
      bBusy := FALSE;
      bDone := FALSE;
  ELSIF iStep = 100 OR iStep = 105 THEN
      bBusy := TRUE;
      IF uiActiveMode = 1 THEN
          bDone := fbMoveContAbs.InEndVelocity;
      ELSIF uiActiveMode = 2 THEN
          bDone := fbMoveVel.InVelocity;
      ELSIF uiActiveMode = 3 THEN
          bDone := fbPresHdl.INPRESSURE;
      ELSE
          bDone := FALSE;
      END_IF;
      bInVelocity := fbMoveVel.InVelocity;
      bInPressure := fbPresHdl.INPRESSURE;
  ELSIF iStep = 150 THEN
      bBusy := TRUE; bDone := FALSE;
  ELSIF iStep = 200 THEN
      bBusy := TRUE; bDone := FALSE;
  ELSIF iStep = 300 THEN
      bBusy := TRUE; bDone := FALSE;
  ELSIF iStep = 400 THEN
      bBusy := TRUE; bDone := FALSE;
  ELSE
      bBusy := FALSE; bDone := FALSE;
  END_IF;

  bStandStillOrAllowUpdate := NOT bBusy OR ( iStep = 100 OR iStep = 105 );
END_FUNCTION_BLOCK
