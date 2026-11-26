TYPE ST_HydAxisRef :
STRUCT
  uiDir                  : UINT; // 轴：设定方向
  udiPos                 : UDINT; // 轴：设定位置
  uiSpd                  : UINT; // 轴：设定速度
  uiEndSpd               : UINT; // 轴：设定结束速度
  uiPres                 : UINT; // 轴：设定压力
  rPosition              : REAL; // 轴：反馈位置
  rVelocity              : REAL; // 轴：反馈速度
  rPressure              : REAL; // 轴：反馈压力
  rFlow                  : REAL; // 轴：反馈流量
  iStep                  : INT; // 轴：反馈当前步号
  rActPressure           : REAL;
  rActPosition           : REAL;
  rActVelocity           : REAL;
  rMaxSpeed              : REAL; // 当前轴的最大速度设定
  rMaxSpeedExtend        : REAL; // 液压轴伸出侧最大速度
  rMaxSpeedRetract       : REAL; // 液压轴缩回侧最大速度
  rVelToFlowGain         : REAL := 0.2; // 速度流量增益
  rCylinderExtendArea    : REAL := 0; // 油缸伸出腔有效面积mm2
  rCylinderRetractArea   : REAL := 0; // 油缸缩回腔有效面积mm2
  iTypePressurePID       : SINT := 4; // 压力闭环类型
  iAxisID                : SINT := -1; // 轴索引
  iMechanismType         : SINT := 0; // 机械运动学类型
  bSimulation            : BOOL := 0; // 启用仿真
  rResponseDelayMs       : REAL := 0; // 液压纯滞后补偿，零终速停车
  rZeroEndSettleVelocity : REAL := 0; // 停稳速度阈值
  rPressureKsys          : REAL := 0; // 液压模型静态增益
  rPressureKq            : REAL := 0; // 液压刚度
  rPressureBrakeTimeMs   : REAL := 0; // 回路总延迟
  rPumpRippleOrder       : REAL := 0; // 纹波阶次 12
  rPressureKp            : REAL := 0;
  rPressureKi            : REAL := 0;
  rPressureLimitKp       : REAL := 0; // 速度段限压 PI 比例增益 L/min/bar（参数 40），0 = 库默认 1.0
  rPressureLimitKi       : REAL := 0; // 速度段限压 PI 积分增益 L/min/(bar·s)（参数 41），0 = 库默认 16
END_STRUCT;
END_TYPE