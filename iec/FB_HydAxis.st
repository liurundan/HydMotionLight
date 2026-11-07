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
         LF := 320,
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

  rMaxSpeed := stHydAxisRef.rMaxSpeed;

  rPres := MAX(UINT_TO_REAL(uiPres), 0.0)*0.1;   //压力设定 bar
  rVel  := UINT_TO_REAL(uiSpd)*0.1*0.01*rMaxSpeed;  //速度设定
  rEndVel := UINT_TO_REAL(uiEndSpd)*0.1*0.01*rMaxSpeed;  //终点速度
  rPos  := UDINT_TO_REAL(udiPos)*0.01;  //位置设定
  rFlowPercent := UINT_TO_REAL(uiSpd)*0.1;
  rFlowPercent := LIMIT(5, rFlowPercent, 100);
      
  IF uiAcc = 0 THEN rAcc := 200; ELSE rAcc  := rMaxSpeed/UINT_TO_REAL(uiAcc)*1000; END_IF;  
  IF uiDec = 0 THEN rDec := 200; ELSE rDec  := rMaxSpeed/UINT_TO_REAL(uiDec)*1000; END_IF;  
  IF uiJerk = 0 THEN rJerk := 0; ELSE rJerk := rMaxSpeed/UINT_TO_REAL(uiJerk); END_IF;

  IF rDec > 200 THEN
    rStopDec := rDec;
  END_IF;

  iDir := uiDir;  

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
              fbCalcMaxSpeed(stHydAxisRef.rCylinderExtendArea, stHydAxisRef.rCylinderRetractArea, 1700, 50, stHydAxisRef.iMechanismType );            
              
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
          ELSIF rTrigStart.Q AND uiMode >= 1 AND uiMode <= 3 THEN
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
              
          // 2. 段切换触发
          ELSIF rTrigStart.Q THEN
              IF uiMode = uiActiveMode THEN
                  iStep := 105; 
              ELSIF uiMode >= 1 AND uiMode <= 3 THEN
                  uiActiveMode := uiMode; 
              END_IF;
          END_IF;
          
          // 3. 中断监测
          IF rTrigEStop.Q THEN iStep := 300;
          ELSIF rTrigStop.Q THEN iStep := 200;
          END_IF;

      105: // 运行过渡步
          bCmdExecute := FALSE; 
          bDone := FALSE;      
          iStep := 100;        

      200: // 正常停止
          IF fbStop.DONE THEN
              iStep := 10; 
              bCmdExecute := FALSE; 
          ELSIF fbStop.ERROR THEN
              bAlarm := TRUE; dwAlarmID := fbStop.ERRORID; iStep := 900;
          END_IF;

      300: // 急停
          IF fbEStop.DONE OR fbEStop.ERROR THEN
              bAlarm := TRUE; dwAlarmID := 16#0000FFFF; iStep := 900;
              bCmdExecute := FALSE;
          END_IF;

      400: // 报警复位
          IF fbReset.DONE THEN
              bAlarm := FALSE; dwAlarmID := 0; iStep := 10;
              bCmdExecute := FALSE;
          ELSIF fbReset.ERROR THEN
              bAlarm := TRUE; dwAlarmID := fbReset.ERRORID; iStep := 900;
          END_IF;
          
      900: // 报警等待
          IF rTrigReset.Q THEN
              iStep := 400;
          END_IF;

  END_CASE;

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
      POSITION := rPos, 
      VELOCITY := rVel,
      ENDVELOCITY := rEndVel,    
      ENDVELOCITYDIRECTION := iDir, 
      ACCELERATION := rAcc, 
      DECELERATION := rDec, 
      JERK := rJerk, 
      DIRECTION := iDir,
      ADAPTENDVELTOAVOIDOVERSHOOT := TRUE,
      PRESSURELIMIT := rPres
  );

  fbMoveVel(
      AXISID := siAxisId, EXECUTE := (uiActiveMode = 2) AND bCmdExecute, 
      CONTINUOUSUPDATE := TRUE, VELOCITY := rVel,
      ACCELERATION := rAcc, DECELERATION := rDec, JERK := rJerk, DIRECTION := iDir, BUFFERMODE := 0,PRESSURELIMIT := rPres);

  fbPresHdl(
      AXISID := siAxisId, 
      EXECUTE := (uiActiveMode = 3) AND bCmdExecute, 
      CONTINUOUSUPDATE := TRUE, 
      PRESSURE := rPres, 
      PRESSURERAMPRATE := rPressureRampRate, 
      DURATION := 0.0, 
      FLOWLIMITPERCENT:=rFlowPercent, 
      BUFFERMODE := 0);

  IF iStep = 200 THEN
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
        stHydAxisRef.rVelocity  := fbReadFB.VELOCITY;
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