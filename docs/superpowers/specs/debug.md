void TESTMOTION_init__(TESTMOTION *data__, BOOL retain) {
  __INIT_VAR(data__->NEWLOCALVAR0,0,retain)
  __INIT_VAR(data__->NEWLOCALVAR1,0,retain)
  __INIT_VAR(data__->FPOS,0,retain)
  __INIT_VAR(data__->FVEL,0,retain)
  __INIT_VAR(data__->FFLOW,0,retain)
  __INIT_VAR(data__->FPRESSURE,0,retain)
  __INIT_VAR(data__->ISTEP,-1,retain)
  __INIT_VAR(data__->BERROR,0,retain)
  __INIT_VAR(data__->WERRORID,0,retain)
  __INIT_VAR(data__->IAXIS0,0,retain)
  __INIT_VAR(data__->BTEMP,__BOOL_LITERAL(FALSE),retain)
  TON_init__(&data__->TDELAY,retain);
  HYD_CREATEMOTION_init__(&data__->FBHDY_CREATEMOTION,retain);
  HYD_MOVEABSOLUTE_init__(&data__->FBHYD_MOVEABSOLUTE,retain);
  HYD_READSIMFEEDBACK_init__(&data__->FBHYD_READSIMFEEDBACK,retain);
  HYD_READSTATUS_init__(&data__->FBHYD_READSTATUS,retain);
  HYD_SETAXISFEEDBACK_init__(&data__->FBHYD_SETAXISFEEDBACK,retain);
  __INIT_VAR(data__->ISTATE,0,retain)
  __INIT_VAR(data__->BACTIVE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->TCYCLE,0,retain)
  FB_WORDBITRISING_init__(&data__->CMD0_DETECT,retain);
  HYD_STOP_init__(&data__->FBHYD_STOP,retain);
  __INIT_EXTERNAL(WORD,GGRP_MBUS_ACTPOS,data__->GGRP_MBUS_ACTPOS,retain)
  __INIT_EXTERNAL(WORD,GGRP_MBUS_COMMAD,data__->GGRP_MBUS_COMMAD,retain)
  __INIT_EXTERNAL(WORD,GGRP_MBUS_ACTVEL,data__->GGRP_MBUS_ACTVEL,retain)
}

// Code part
void TESTMOTION_body__(TESTMOTION *data__) {
  // Initialise TEMP variables

  __SET_VAR(data__->,NEWLOCALVAR0,,(__GET_VAR(data__->NEWLOCALVAR0,) + 1));
  __SET_VAR(data__->,TCYCLE,,(__GET_VAR(data__->TCYCLE,) + 0.001));
  __SET_VAR(data__->CMD0_DETECT.,WORDIN,,__GET_EXTERNAL(data__->GGRP_MBUS_COMMAD,));
  __SET_VAR(data__->CMD0_DETECT.,BITPOS,,0);
  FB_WORDBITRISING_body__(&data__->CMD0_DETECT);
  if (__GET_VAR(data__->CMD0_DETECT.RISING,)) {
    __SET_VAR(data__->,ISTEP,,0);
  };
  if ((__GET_VAR(data__->ISTEP,) > 0)) {
    __SET_VAR(data__->FBHYD_READSIMFEEDBACK.,ENABLE,,1);
    __SET_VAR(data__->FBHYD_READSIMFEEDBACK.,AXISID,,__GET_VAR(data__->IAXIS0,));
    HYD_READSIMFEEDBACK_body__(&data__->FBHYD_READSIMFEEDBACK);
    __SET_VAR(data__->,FPOS,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.POSITION));
    __SET_VAR(data__->,FVEL,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.VELOCITY));
    __SET_VAR(data__->,FFLOW,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.FLOW));
    __SET_VAR(data__->,FPRESSURE,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.PRESSURE));
    __SET_VAR(data__->FBHYD_READSTATUS.,ENABLE,,1);
    __SET_VAR(data__->FBHYD_READSTATUS.,AXISID,,__GET_VAR(data__->IAXIS0,));
    HYD_READSTATUS_body__(&data__->FBHYD_READSTATUS);
    __SET_VAR(data__->,ISTATE,,__GET_VAR(data__->FBHYD_READSTATUS.STATE));
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ENABLE,,1);
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,AXISID,,__GET_VAR(data__->IAXIS0,));
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ACT_POSITION,,__GET_VAR(data__->FPOS,));
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ACT_VELOCITY,,__GET_VAR(data__->FVEL,));
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ACT_FLOW,,__GET_VAR(data__->FFLOW,));
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ACT_PRESSURE,,__GET_VAR(data__->FPRESSURE,));
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,TIMESTAMP,,__GET_VAR(data__->TCYCLE,));
    HYD_SETAXISFEEDBACK_body__(&data__->FBHYD_SETAXISFEEDBACK);
  };
  {
    DINT __case_expression = __GET_VAR(data__->ISTEP,);
    if ((__case_expression == 0)) {
      __SET_VAR(data__->FBHDY_CREATEMOTION.,USE_RECIPE,,0);
      __SET_VAR(data__->FBHDY_CREATEMOTION.,FLOW_TO_PUMPSPEED,,1.0);
      __SET_VAR(data__->FBHDY_CREATEMOTION.,USE_SIMULATION,,1);
      HYD_CREATEMOTION_body__(&data__->FBHDY_CREATEMOTION);
      if (__GET_VAR(data__->FBHDY_CREATEMOTION.DONE,)) {
        __SET_VAR(data__->,IAXIS0,,__GET_VAR(data__->FBHDY_CREATEMOTION.AXISID,));
        __SET_VAR(data__->,ISTEP,,(__GET_VAR(data__->ISTEP,) + 1));
        __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(FALSE));
        HYD_MOVEABSOLUTE_body__(&data__->FBHYD_MOVEABSOLUTE);
      };
    }
    else if ((__case_expression == 1)) {
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,POSITION,,400);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,VELOCITY,,20);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,ACCELERATION,,200);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,DECELERATION,,200);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,DIRECTION,,1);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,BUFFERMODE,,0);
      HYD_MOVEABSOLUTE_body__(&data__->FBHYD_MOVEABSOLUTE);
      __SET_VAR(data__->,BERROR,,__GET_VAR(data__->FBHYD_MOVEABSOLUTE.ERROR,));
      __SET_VAR(data__->,WERRORID,,__GET_VAR(data__->FBHYD_MOVEABSOLUTE.ERRORID,));
      __SET_VAR(data__->,BACTIVE,,__GET_VAR(data__->FBHYD_MOVEABSOLUTE.ACTIVE,));
      if (__GET_VAR(data__->FBHYD_MOVEABSOLUTE.DONE,)) {
        __SET_VAR(data__->TDELAY.,IN,,1);
        __SET_VAR(data__->TDELAY.,PT,,__time_to_timespec(1, 0, 0, 0, 0, 0));
        TON_body__(&data__->TDELAY);
        if (__GET_VAR(data__->TDELAY.Q,)) {
          __SET_VAR(data__->,ISTEP,,2);
          __SET_VAR(data__->TDELAY.,IN,,0);
          TON_body__(&data__->TDELAY);
          __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(FALSE));
          __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,AXISID,,__GET_VAR(data__->IAXIS0,));
          HYD_MOVEABSOLUTE_body__(&data__->FBHYD_MOVEABSOLUTE);
        };
      };
    }
    else if ((__case_expression == 2)) {
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,POSITION,,0);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,VELOCITY,,20);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,ACCELERATION,,200);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,DECELERATION,,200);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,DIRECTION,,-1);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,BUFFERMODE,,0);
      HYD_MOVEABSOLUTE_body__(&data__->FBHYD_MOVEABSOLUTE);
      if (__GET_VAR(data__->FBHYD_MOVEABSOLUTE.DONE,)) {
        __SET_VAR(data__->TDELAY.,IN,,1);
        __SET_VAR(data__->TDELAY.,PT,,__time_to_timespec(1, 0, 0, 0, 0, 0));
        TON_body__(&data__->TDELAY);
        if (__GET_VAR(data__->TDELAY.Q,)) {
          __SET_VAR(data__->,ISTEP,,1);
          __SET_VAR(data__->TDELAY.,IN,,0);
          TON_body__(&data__->TDELAY);
          __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(FALSE));
          __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,AXISID,,__GET_VAR(data__->IAXIS0,));
          HYD_MOVEABSOLUTE_body__(&data__->FBHYD_MOVEABSOLUTE);
        };
      };
    }
    else if ((__case_expression == 3)) {
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(FALSE));
      HYD_MOVEABSOLUTE_body__(&data__->FBHYD_MOVEABSOLUTE);
      __SET_VAR(data__->FBHYD_STOP.,EXECUTE,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->FBHYD_STOP.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_STOP.,DECELERATION,,50.0);
      HYD_STOP_body__(&data__->FBHYD_STOP);
      __SET_VAR(data__->,BERROR,,__GET_VAR(data__->FBHYD_STOP.ERROR,));
      __SET_VAR(data__->,WERRORID,,__GET_VAR(data__->FBHYD_STOP.ERRORID,));
      if (__GET_VAR(data__->FBHYD_STOP.DONE,)) {
        __SET_VAR(data__->,ISTEP,,4);
        __SET_VAR(data__->FBHYD_STOP.,EXECUTE,,__BOOL_LITERAL(FALSE));
        HYD_STOP_body__(&data__->FBHYD_STOP);
      };
    }
  };
