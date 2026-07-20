void LOGGER_init__(LOGGER *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->TRIG,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->MSG,__STRING_LITERAL(0,""),retain)
  __INIT_VAR(data__->LEVEL,LOGLEVEL__INFO,retain)
  __INIT_VAR(data__->TRIG0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void LOGGER_body__(LOGGER *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  if ((__GET_VAR(data__->TRIG,) && !(__GET_VAR(data__->TRIG0,)))) {
    #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
    #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)

   LogMessage(GetFbVar(LEVEL),(char*)GetFbVar(MSG, .body),GetFbVar(MSG, .len));
  
    #undef GetFbVar
    #undef SetFbVar
;
  };
  __SET_VAR(data__->,TRIG0,,__GET_VAR(data__->TRIG,));

  goto __end;

__end:
  return;
} // LOGGER_body__() 





// FUNCTION
TIME GETCURRENTTIME(
  BOOL EN, 
  BOOL *__ENO, 
  INT MODE)
{
  BOOL ENO = __BOOL_LITERAL(TRUE);
  TIME GETCURRENTTIME = __time_to_timespec(1, 0, 0, 0, 0, 0);

  // Control execution
  if (!EN) {
    if (__ENO != NULL) {
      *__ENO = __BOOL_LITERAL(FALSE);
    }
    return GETCURRENTTIME;
  }
  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 GETCURRENTTIME = __CURRENT_TIME; 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  if (__ENO != NULL) {
    *__ENO = ENO;
  }
  return GETCURRENTTIME;
}


void HYD_MOVEPROFILE_init__(HYD_MOVEPROFILE *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  {
    static const HYD_AXISMOTION temp = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    __SET_VAR(data__->,MOTION,,temp);
  }
  __INIT_VAR(data__->BUFFERMODE,0,retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACTIVE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->COMMANDABORTED,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACTIVE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_PENDING,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_EXEC_ID,0,retain)
}

// Code part
void HYD_MOVEPROFILE_body__(HYD_MOVEPROFILE *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_MoveProfile(HYD_MOVEPROFILE *data__);	__mcl_cmd_MoveProfile(data__);	 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_MOVEPROFILE_body__() 





void HYD_LOADPROFILE_init__(HYD_LOADPROFILE *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  {
    static const HYD_AXISMOTION temp = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    __SET_VAR(data__->,MOTION,,temp);
  }
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_LOADPROFILE_body__(HYD_LOADPROFILE *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_LoadProfile(HYD_LOADPROFILE *data__);	__mcl_cmd_LoadProfile(data__);	 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_LOADPROFILE_body__() 





void HYD_STOP_init__(HYD_STOP *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DECELERATION,0,retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->COMMANDABORTED,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACTIVE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_PENDING,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_STOP_body__(HYD_STOP *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_Stop(HYD_STOP *data__);	__mcl_cmd_Stop(data__);	 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_STOP_body__() 





void HYD_HOLD_init__(HYD_HOLD *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_PENDING,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_HOLD_body__(HYD_HOLD *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_Hold(HYD_HOLD*); __mcl_cmd_Hold(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_HOLD_body__() 





void HYD_RESUME_init__(HYD_RESUME *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_PENDING,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_RESUME_body__(HYD_RESUME *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_Resume(HYD_RESUME*); __mcl_cmd_Resume(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_RESUME_body__() 





void HYD_MOVEABSOLUTE_init__(HYD_MOVEABSOLUTE *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->CONTINUOUSUPDATE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->POSITION,0,retain)
  __INIT_VAR(data__->VELOCITY,0,retain)
  __INIT_VAR(data__->ACCELERATION,0,retain)
  __INIT_VAR(data__->DECELERATION,0,retain)
  __INIT_VAR(data__->JERK,0,retain)
  __INIT_VAR(data__->DIRECTION,0,retain)
  __INIT_VAR(data__->BUFFERMODE,0,retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACTIVE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->COMMANDABORTED,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACTIVE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_PENDING,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_EXEC_ID,0,retain)
}

// Code part
void HYD_MOVEABSOLUTE_body__(HYD_MOVEABSOLUTE *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_MoveAbsolute(HYD_MOVEABSOLUTE*); __mcl_cmd_MoveAbsolute(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_MOVEABSOLUTE_body__() 





void HYD_MOVEVELOCITY_init__(HYD_MOVEVELOCITY *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->CONTINUOUSUPDATE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->VELOCITY,0,retain)
  __INIT_VAR(data__->ACCELERATION,0,retain)
  __INIT_VAR(data__->DECELERATION,0,retain)
  __INIT_VAR(data__->JERK,0,retain)
  __INIT_VAR(data__->DIRECTION,0,retain)
  __INIT_VAR(data__->BUFFERMODE,0,retain)
  __INIT_VAR(data__->PRESSURELIMIT,0,retain)
  __INIT_VAR(data__->INVELOCITY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACTIVE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->COMMANDABORTED,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->INVELOCITY0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACTIVE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_PENDING,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_EXEC_ID,0,retain)
}

// Code part
void HYD_MOVEVELOCITY_body__(HYD_MOVEVELOCITY *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_MoveVelocity(HYD_MOVEVELOCITY*); __mcl_cmd_MoveVelocity(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_MOVEVELOCITY_body__() 





void HYD_RESET_init__(HYD_RESET *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_RESET_body__(HYD_RESET *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_Reset(HYD_RESET*); __mcl_cmd_Reset(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_RESET_body__() 





void HYD_PRESSUREHANDLE_init__(HYD_PRESSUREHANDLE *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->CONTINUOUSUPDATE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->PRESSURE,0,retain)
  __INIT_VAR(data__->PRESSURERAMPRATE,0,retain)
  __INIT_VAR(data__->DURATION,0,retain)
  __INIT_VAR(data__->BUFFERMODE,0,retain)
  __INIT_VAR(data__->INPRESSURE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACTIVE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->COMMANDABORTED,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->INPRESSURE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACTIVE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_PENDING,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->_EXEC_ID,0,retain)
}

// Code part
void HYD_PRESSUREHANDLE_body__(HYD_PRESSUREHANDLE *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_PressureHandle(HYD_PRESSUREHANDLE*); __mcl_cmd_PressureHandle(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_PRESSUREHANDLE_body__() 





void HYD_CREATEMOTION_init__(HYD_CREATEMOTION *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->USE_RECIPE,0,retain)
  __INIT_VAR(data__->FLOW_TO_PUMPSPEED,20.0,retain)
  __INIT_VAR(data__->PUMPSPEED_LIMIT,1800.0,retain)
  __INIT_VAR(data__->USE_SIMULATION,0,retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_CREATEMOTION_body__(HYD_CREATEMOTION *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_CreateMotion(HYD_CREATEMOTION*); __mcl_cmd_CreateMotion(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_CREATEMOTION_body__() 





void HYD_SETAXISFEEDBACK_init__(HYD_SETAXISFEEDBACK *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->ENABLE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ACT_POSITION,0,retain)
  __INIT_VAR(data__->ACT_VELOCITY,0,retain)
  __INIT_VAR(data__->ACT_FLOW,0,retain)
  __INIT_VAR(data__->ACT_PRESSURE,0,retain)
  __INIT_VAR(data__->TIMESTAMP,0,retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_SETAXISFEEDBACK_body__(HYD_SETAXISFEEDBACK *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_SetAxisFeedback(HYD_SETAXISFEEDBACK*); __mcl_cmd_SetAxisFeedback(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_SETAXISFEEDBACK_body__() 





void HYD_GETPUMPREQUEST_init__(HYD_GETPUMPREQUEST *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENABLE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->STRATEGY,0,retain)
  __INIT_VAR(data__->PUMPSPEED,0,retain)
  __INIT_VAR(data__->CONFLICT,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_GETPUMPREQUEST_body__(HYD_GETPUMPREQUEST *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_GetPumpRequest(HYD_GETPUMPREQUEST*); __mcl_cmd_GetPumpRequest(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_GETPUMPREQUEST_body__() 





void HYD_READSTATUS_init__(HYD_READSTATUS *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->ENABLE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->VALID,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->STATE,0,retain)
}

// Code part
void HYD_READSTATUS_body__(HYD_READSTATUS *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_ReadStatus(HYD_READSTATUS*); __mcl_cmd_ReadStatus(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_READSTATUS_body__() 





void HYD_READERROR_init__(HYD_READERROR *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->ENABLE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->VALID,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
}

// Code part
void HYD_READERROR_body__(HYD_READERROR *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_ReadError(HYD_READERROR*); __mcl_cmd_ReadError(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_READERROR_body__() 





void HYD_READSIMFEEDBACK_init__(HYD_READSIMFEEDBACK *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->ENABLE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->VALID,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->POSITION,0,retain)
  __INIT_VAR(data__->VELOCITY,0,retain)
  __INIT_VAR(data__->FLOW,0,retain)
  __INIT_VAR(data__->PRESSURE,0,retain)
}

// Code part
void HYD_READSIMFEEDBACK_body__(HYD_READSIMFEEDBACK *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_ReadSimFeedback(HYD_READSIMFEEDBACK*); __mcl_cmd_ReadSimFeedback(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_READSIMFEEDBACK_body__() 





void HYD_READPARAMETER_init__(HYD_READPARAMETER *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->ENABLE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->PARAMETERNUMBER,0,retain)
  __INIT_VAR(data__->VALID,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->VALUE,0,retain)
  __INIT_VAR(data__->ENABLE0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_READPARAMETER_body__(HYD_READPARAMETER *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_ReadParameter(HYD_READPARAMETER*); __mcl_cmd_ReadParameter(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_READPARAMETER_body__() 





void HYD_WRITEPARAMETER_init__(HYD_WRITEPARAMETER *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->PARAMETERNUMBER,0,retain)
  __INIT_VAR(data__->VALUE,0,retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_WRITEPARAMETER_body__(HYD_WRITEPARAMETER *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_WriteParameter(HYD_WRITEPARAMETER*); __mcl_cmd_WriteParameter(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_WRITEPARAMETER_body__() 





void HYD_READBOOLPARAMETER_init__(HYD_READBOOLPARAMETER *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->ENABLE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->PARAMETERNUMBER,0,retain)
  __INIT_VAR(data__->VALID,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->VALUE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ENABLE0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_READBOOLPARAMETER_body__(HYD_READBOOLPARAMETER *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_ReadBoolParameter(HYD_READBOOLPARAMETER*); __mcl_cmd_ReadBoolParameter(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_READBOOLPARAMETER_body__() 





void HYD_WRITEBOOLPARAMETER_init__(HYD_WRITEBOOLPARAMETER *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->AXISID,0,retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->PARAMETERNUMBER,0,retain)
  __INIT_VAR(data__->VALUE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->EXECUTE0,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE0,__BOOL_LITERAL(FALSE),retain)
}

// Code part
void HYD_WRITEBOOLPARAMETER_body__(HYD_WRITEBOOLPARAMETER *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __IL_DEFVAR_T __IL_DEFVAR;
  __IL_DEFVAR_T __IL_DEFVAR_BACK;
  #define GetFbVar(var,...) __GET_VAR(data__->var,__VA_ARGS__)
  #define SetFbVar(var,val,...) __SET_VAR(data__->,var,__VA_ARGS__,val)
 extern void __mcl_cmd_WriteBoolParameter(HYD_WRITEBOOLPARAMETER*); __mcl_cmd_WriteBoolParameter(data__); 
  #undef GetFbVar
  #undef SetFbVar
;

  goto __end;

__end:
  return;
} // HYD_WRITEBOOLPARAMETER_body__() 





void FIRSTORDERSYSTEM_init__(FIRSTORDERSYSTEM *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->MAXDELAYSTEPS,500,retain)
  __INIT_VAR(data__->ENABLE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->MOTORVELOCITY,0,retain)
  __INIT_VAR(data__->TCYCLE,0.01,retain)
  __INIT_VAR(data__->KNUM,50.0,retain)
  __INIT_VAR(data__->TTAU,0.2,retain)
  __INIT_VAR(data__->DELAYTIME,0.0,retain)
  __INIT_VAR(data__->PRESSURE,0,retain)
  __INIT_VAR(data__->PRE_PRESSURE,0.0,retain)
  __INIT_VAR(data__->DISTURBANCE,0.0,retain)
  __INIT_VAR(data__->TCURR,0.0,retain)
  __INIT_VAR(data__->DELAYSTEPS,0,retain)
  {
    static const __ARRAY_OF_REAL_500 temp = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
    __SET_VAR(data__->,OUTPUTBUFFER,,temp);
  }
  __INIT_VAR(data__->BUFFERINDEX,0,retain)
  __INIT_VAR(data__->CURRENTOUTPUT,0.0,retain)
  __INIT_VAR(data__->DELAYINDEX,0,retain)
  __INIT_VAR(data__->I,0,retain)
  __INIT_VAR(data__->DISTURBANCESIGNAL,0,retain)
}

// Code part
void FIRSTORDERSYSTEM_body__(FIRSTORDERSYSTEM *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  if (__GET_VAR(data__->ENABLE,)) {
    __SET_VAR(data__->,DELAYSTEPS,,REAL_TO_DINT(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (REAL)LIMIT__REAL__REAL__REAL__REAL(
        (BOOL)__BOOL_LITERAL(TRUE),
        NULL,
        (REAL)0.0,
        (REAL)(__GET_VAR(data__->DELAYTIME,) / __GET_VAR(data__->TCYCLE,)),
        (REAL)UINT_TO_REAL(
          (BOOL)__BOOL_LITERAL(TRUE),
          NULL,
          (UINT)(__GET_VAR(data__->MAXDELAYSTEPS,) - 1)))));
    __SET_VAR(data__->,CURRENTOUTPUT,,((((__GET_VAR(data__->KNUM,) * __GET_VAR(data__->MOTORVELOCITY,)) * __GET_VAR(data__->TCYCLE,)) + (__GET_VAR(data__->TTAU,) * __GET_VAR(data__->PRE_PRESSURE,))) / (__GET_VAR(data__->TTAU,) + __GET_VAR(data__->TCYCLE,))));
    __SET_VAR(data__->,CURRENTOUTPUT,,LIMIT__REAL__REAL__REAL__REAL(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (REAL)0,
      (REAL)__GET_VAR(data__->CURRENTOUTPUT,),
      (REAL)250));
    __SET_VAR(data__->,OUTPUTBUFFER,.table[(__GET_VAR(data__->BUFFERINDEX,)) - (0)],__GET_VAR(data__->CURRENTOUTPUT,));
    if ((__GET_VAR(data__->DELAYSTEPS,) > 0)) {
      __SET_VAR(data__->,I,,((__GET_VAR(data__->MAXDELAYSTEPS,) == 0)?0:(((__GET_VAR(data__->BUFFERINDEX,) + __GET_VAR(data__->MAXDELAYSTEPS,)) - __GET_VAR(data__->DELAYSTEPS,)) % __GET_VAR(data__->MAXDELAYSTEPS,))));
      __SET_VAR(data__->,PRESSURE,,__GET_VAR(data__->OUTPUTBUFFER,.table[(__GET_VAR(data__->I,)) - (0)]));
    } else {
      __SET_VAR(data__->,PRESSURE,,__GET_VAR(data__->CURRENTOUTPUT,));
    };
    __SET_VAR(data__->,BUFFERINDEX,,((__GET_VAR(data__->MAXDELAYSTEPS,) == 0)?0:((__GET_VAR(data__->BUFFERINDEX,) + 1) % __GET_VAR(data__->MAXDELAYSTEPS,))));
    __SET_VAR(data__->,PRE_PRESSURE,,__GET_VAR(data__->CURRENTOUTPUT,));
    __SET_VAR(data__->,TCURR,,(__GET_VAR(data__->TCURR,) + __GET_VAR(data__->TCYCLE,)));
  } else {
    /* FOR ... */
    __SET_VAR(data__->,I,,0);
    {
      int __do_increment = 0;
      while(1) {
        if(__do_increment){
          /* BY ... (of FOR loop) */
          __SET_VAR(data__->,I,,(__GET_VAR(data__->I,) + 1));
        } else __do_increment = 1;
        if(__GET_VAR(data__->I,) <= (__GET_VAR(data__->MAXDELAYSTEPS,) - 1)        ){
          __SET_VAR(data__->,OUTPUTBUFFER,.table[(__GET_VAR(data__->I,)) - (0)],0.0);
        }else break;
      }
    } /* END_FOR */;
    __SET_VAR(data__->,BUFFERINDEX,,0);
    __SET_VAR(data__->,PRE_PRESSURE,,0.0);
    __SET_VAR(data__->,PRESSURE,,0.0);
    __SET_VAR(data__->,TCURR,,0.0);
  };

  goto __end;

__end:
  return;
} // FIRSTORDERSYSTEM_body__() 





void FB_HYDAXIS3_init__(FB_HYDAXIS3 *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->BSTART,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BSTOP,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BESTOP,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BRESET,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->UIDIR,0,retain)
  __INIT_VAR(data__->UIMODE,0,retain)
  __INIT_VAR(data__->UIPRES,0,retain)
  __INIT_VAR(data__->UISPD,0,retain)
  __INIT_VAR(data__->UDIPOS,0,retain)
  __INIT_VAR(data__->UIFORCE,0,retain)
  __INIT_VAR(data__->UIACC,0,retain)
  __INIT_VAR(data__->UIDEC,0,retain)
  __INIT_VAR(data__->UIJERK,0,retain)
  __INIT_VAR(data__->BBUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BDONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BINVELOCITY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BINPRESSURE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BALARM,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DWALARMID,0,retain)
  __INIT_VAR(data__->ISTEP,0,retain)
  HYD_CREATEMOTION_init__(&data__->FBCREATE,retain);
  HYD_MOVEABSOLUTE_init__(&data__->FBMOVEABS,retain);
  HYD_MOVEVELOCITY_init__(&data__->FBMOVEVEL,retain);
  HYD_PRESSUREHANDLE_init__(&data__->FBPRESHDL,retain);
  HYD_STOP_init__(&data__->FBSTOP,retain);
  HYD_STOP_init__(&data__->FBESTOP,retain);
  HYD_RESET_init__(&data__->FBRESET,retain);
  HYD_READSIMFEEDBACK_init__(&data__->FBREADFB,retain);
  __INIT_VAR(data__->SIAXISID,0,retain)
  R_TRIG_init__(&data__->RTRIGSTART,retain);
  R_TRIG_init__(&data__->RTRIGSTOP,retain);
  R_TRIG_init__(&data__->RTRIGESTOP,retain);
  R_TRIG_init__(&data__->RTRIGRESET,retain);
  __INIT_VAR(data__->UIACTIVEMODE,0,retain)
  __INIT_VAR(data__->BCMDEXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->RPOS,0,retain)
  __INIT_VAR(data__->RVEL,0,retain)
  __INIT_VAR(data__->RPRES,0,retain)
  __INIT_VAR(data__->RACC,0,retain)
  __INIT_VAR(data__->RDEC,0,retain)
  __INIT_VAR(data__->RJERK,0,retain)
  __INIT_VAR(data__->BFBERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->WFBERRORID,0,retain)
  __INIT_VAR(data__->IDIR,0,retain)
  __INIT_VAR(data__->RPOSITION,0,retain)
  __INIT_VAR(data__->RVELOCTITY,0,retain)
  __INIT_VAR(data__->RFLOW,0,retain)
  __INIT_VAR(data__->RPERSSURE,0,retain)
}

// Code part
void FB_HYDAXIS3_body__(FB_HYDAXIS3 *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __SET_VAR(data__->RTRIGSTART.,CLK,,__GET_VAR(data__->BSTART,));
  R_TRIG_body__(&data__->RTRIGSTART);
  __SET_VAR(data__->RTRIGSTOP.,CLK,,__GET_VAR(data__->BSTOP,));
  R_TRIG_body__(&data__->RTRIGSTOP);
  __SET_VAR(data__->RTRIGESTOP.,CLK,,__GET_VAR(data__->BESTOP,));
  R_TRIG_body__(&data__->RTRIGESTOP);
  __SET_VAR(data__->RTRIGRESET.,CLK,,__GET_VAR(data__->BRESET,));
  R_TRIG_body__(&data__->RTRIGRESET);
  __SET_VAR(data__->,RVEL,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UISPD,)));
  __SET_VAR(data__->,RACC,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UIACC,)));
  __SET_VAR(data__->,RDEC,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UIDEC,)));
  __SET_VAR(data__->,RJERK,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UIJERK,)));
  __SET_VAR(data__->,RPOS,,UDINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UDINT)__GET_VAR(data__->UDIPOS,)));
  __SET_VAR(data__->,RPRES,,MAX__REAL__REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)2,
    (REAL)UINT_TO_REAL(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (UINT)__GET_VAR(data__->UIPRES,)),
    (REAL)UINT_TO_REAL(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (UINT)__GET_VAR(data__->UIFORCE,))));
  if ((__GET_VAR(data__->UIDIR,) == 2)) {
    __SET_VAR(data__->,IDIR,,-1);
  } else {
    __SET_VAR(data__->,IDIR,,UINT_TO_SINT(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (UINT)__GET_VAR(data__->UIDIR,)));
  };
  {
    INT __case_expression = __GET_VAR(data__->ISTEP,);
    if ((__case_expression == 0)) {
      if (__GET_VAR(data__->FBCREATE.DONE,)) {
        __SET_VAR(data__->,SIAXISID,,__GET_VAR(data__->FBCREATE.AXISID,));
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->FBCREATE.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,0x1);
        __SET_VAR(data__->,ISTEP,,900);
      };
    }
    else if ((__case_expression == 10)) {
      __SET_VAR(data__->,BCMDEXECUTE,,__BOOL_LITERAL(FALSE));
      if (__GET_VAR(data__->RTRIGESTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,300);
      } else if (__GET_VAR(data__->RTRIGSTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,200);
      } else if ((__GET_VAR(data__->RTRIGRESET.Q,) && __GET_VAR(data__->BALARM,))) {
        __SET_VAR(data__->,ISTEP,,400);
      } else if (((__GET_VAR(data__->RTRIGSTART.Q,) && (__GET_VAR(data__->UIMODE,) >= 1)) && (__GET_VAR(data__->UIMODE,) <= 3))) {
        __SET_VAR(data__->,UIACTIVEMODE,,__GET_VAR(data__->UIMODE,));
        __SET_VAR(data__->,ISTEP,,100);
      };
    }
    else if ((__case_expression == 100)) {
      __SET_VAR(data__->,BCMDEXECUTE,,__BOOL_LITERAL(TRUE));
      {
        UINT __case_expression = __GET_VAR(data__->UIACTIVEMODE,);
        if ((__case_expression == 1)) {
          __SET_VAR(data__->,BFBERROR,,__GET_VAR(data__->FBMOVEABS.ERROR,));
          __SET_VAR(data__->,WFBERRORID,,__GET_VAR(data__->FBMOVEABS.ERRORID,));
        }
        else if ((__case_expression == 2)) {
          __SET_VAR(data__->,BFBERROR,,__GET_VAR(data__->FBMOVEVEL.ERROR,));
          __SET_VAR(data__->,WFBERRORID,,__GET_VAR(data__->FBMOVEVEL.ERRORID,));
        }
        else if ((__case_expression == 3)) {
          __SET_VAR(data__->,BFBERROR,,__GET_VAR(data__->FBPRESHDL.ERROR,));
          __SET_VAR(data__->,WFBERRORID,,__GET_VAR(data__->FBPRESHDL.ERRORID,));
        }
      };
      if (__GET_VAR(data__->BFBERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->WFBERRORID,));
        __SET_VAR(data__->,ISTEP,,900);
      } else if (((__GET_VAR(data__->UIACTIVEMODE,) == 1) && __GET_VAR(data__->FBMOVEABS.DONE,))) {
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->RTRIGESTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,300);
      } else if (__GET_VAR(data__->RTRIGSTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,200);
      } else if (__GET_VAR(data__->RTRIGSTART.Q,)) {
        if ((__GET_VAR(data__->UIMODE,) == __GET_VAR(data__->UIACTIVEMODE,))) {
          __SET_VAR(data__->,ISTEP,,105);
        } else if (((__GET_VAR(data__->UIMODE,) >= 1) && (__GET_VAR(data__->UIMODE,) <= 3))) {
          __SET_VAR(data__->,UIACTIVEMODE,,__GET_VAR(data__->UIMODE,));
        };
      };
    }
    else if ((__case_expression == 105)) {
      __SET_VAR(data__->,BCMDEXECUTE,,__BOOL_LITERAL(FALSE));
      __SET_VAR(data__->,ISTEP,,100);
    }
    else if ((__case_expression == 200)) {
      if (__GET_VAR(data__->FBSTOP.DONE,)) {
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->FBSTOP.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->FBSTOP.ERRORID,));
        __SET_VAR(data__->,ISTEP,,900);
      };
    }
    else if ((__case_expression == 300)) {
      if ((__GET_VAR(data__->FBESTOP.DONE,) || __GET_VAR(data__->FBESTOP.ERROR,))) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,0xFFFF);
        __SET_VAR(data__->,ISTEP,,900);
      };
    }
    else if ((__case_expression == 400)) {
      if (__GET_VAR(data__->FBRESET.DONE,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(FALSE));
        __SET_VAR(data__->,DWALARMID,,0);
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->FBRESET.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->FBRESET.ERRORID,));
        __SET_VAR(data__->,ISTEP,,900);
      };
    }
    else if ((__case_expression == 900)) {
      if (__GET_VAR(data__->RTRIGRESET.Q,)) {
        __SET_VAR(data__->,ISTEP,,400);
      };
    }
  };
  __SET_VAR(data__->FBCREATE.,USE_SIMULATION,,__BOOL_LITERAL(TRUE));
  HYD_CREATEMOTION_body__(&data__->FBCREATE);
  __SET_VAR(data__->FBMOVEABS.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBMOVEABS.,EXECUTE,,((__GET_VAR(data__->UIACTIVEMODE,) == 1) && __GET_VAR(data__->BCMDEXECUTE,)));
  __SET_VAR(data__->FBMOVEABS.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
  __SET_VAR(data__->FBMOVEABS.,POSITION,,__GET_VAR(data__->RPOS,));
  __SET_VAR(data__->FBMOVEABS.,VELOCITY,,__GET_VAR(data__->RVEL,));
  __SET_VAR(data__->FBMOVEABS.,ACCELERATION,,__GET_VAR(data__->RACC,));
  __SET_VAR(data__->FBMOVEABS.,DECELERATION,,__GET_VAR(data__->RDEC,));
  __SET_VAR(data__->FBMOVEABS.,JERK,,__GET_VAR(data__->RJERK,));
  __SET_VAR(data__->FBMOVEABS.,DIRECTION,,__GET_VAR(data__->IDIR,));
  __SET_VAR(data__->FBMOVEABS.,BUFFERMODE,,0);
  HYD_MOVEABSOLUTE_body__(&data__->FBMOVEABS);
  __SET_VAR(data__->FBMOVEVEL.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBMOVEVEL.,EXECUTE,,((__GET_VAR(data__->UIACTIVEMODE,) == 2) && __GET_VAR(data__->BCMDEXECUTE,)));
  __SET_VAR(data__->FBMOVEVEL.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
  __SET_VAR(data__->FBMOVEVEL.,VELOCITY,,__GET_VAR(data__->RVEL,));
  __SET_VAR(data__->FBMOVEVEL.,ACCELERATION,,__GET_VAR(data__->RACC,));
  __SET_VAR(data__->FBMOVEVEL.,DECELERATION,,__GET_VAR(data__->RDEC,));
  __SET_VAR(data__->FBMOVEVEL.,JERK,,__GET_VAR(data__->RJERK,));
  __SET_VAR(data__->FBMOVEVEL.,DIRECTION,,__GET_VAR(data__->IDIR,));
  __SET_VAR(data__->FBMOVEVEL.,BUFFERMODE,,0);
  HYD_MOVEVELOCITY_body__(&data__->FBMOVEVEL);
  __SET_VAR(data__->FBPRESHDL.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBPRESHDL.,EXECUTE,,((__GET_VAR(data__->UIACTIVEMODE,) == 3) && __GET_VAR(data__->BCMDEXECUTE,)));
  __SET_VAR(data__->FBPRESHDL.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
  __SET_VAR(data__->FBPRESHDL.,PRESSURE,,__GET_VAR(data__->RPRES,));
  __SET_VAR(data__->FBPRESHDL.,PRESSURERAMPRATE,,__GET_VAR(data__->RACC,));
  __SET_VAR(data__->FBPRESHDL.,DURATION,,0.0);
  __SET_VAR(data__->FBPRESHDL.,BUFFERMODE,,0);
  HYD_PRESSUREHANDLE_body__(&data__->FBPRESHDL);
  __SET_VAR(data__->FBSTOP.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBSTOP.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 200));
  __SET_VAR(data__->FBSTOP.,DECELERATION,,__GET_VAR(data__->RDEC,));
  HYD_STOP_body__(&data__->FBSTOP);
  __SET_VAR(data__->FBESTOP.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBESTOP.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 300));
  __SET_VAR(data__->FBESTOP.,DECELERATION,,99999.0);
  HYD_STOP_body__(&data__->FBESTOP);
  __SET_VAR(data__->FBRESET.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBRESET.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 400));
  HYD_RESET_body__(&data__->FBRESET);
  __SET_VAR(data__->FBREADFB.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBREADFB.,ENABLE,,__BOOL_LITERAL(TRUE));
  HYD_READSIMFEEDBACK_body__(&data__->FBREADFB);
  if (__GET_VAR(data__->FBREADFB.VALID,)) {
    __SET_VAR(data__->,RPOSITION,,__GET_VAR(data__->FBREADFB.POSITION,));
    __SET_VAR(data__->,RVELOCTITY,,__GET_VAR(data__->FBREADFB.VELOCITY,));
    __SET_VAR(data__->,RFLOW,,__GET_VAR(data__->FBREADFB.FLOW,));
    __SET_VAR(data__->,RPERSSURE,,__GET_VAR(data__->FBREADFB.PRESSURE,));
  };
  if ((__GET_VAR(data__->ISTEP,) == 10)) {
    __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(FALSE));
    __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(TRUE));
  } else if (((__GET_VAR(data__->ISTEP,) == 100) || (__GET_VAR(data__->ISTEP,) == 105))) {
    __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(TRUE));
    __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(FALSE));
    __SET_VAR(data__->,BINVELOCITY,,__GET_VAR(data__->FBMOVEVEL.INVELOCITY,));
    __SET_VAR(data__->,BINPRESSURE,,__GET_VAR(data__->FBPRESHDL.INPRESSURE,));
  } else if ((__GET_VAR(data__->ISTEP,) == 200)) {
    __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBSTOP.BUSY,));
    __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(FALSE));
  } else if ((__GET_VAR(data__->ISTEP,) == 300)) {
    __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(TRUE));
    __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(FALSE));
  } else if ((__GET_VAR(data__->ISTEP,) == 400)) {
    __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBRESET.BUSY,));
    __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(FALSE));
  } else {
    __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(FALSE));
    __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(FALSE));
  };

  goto __end;

__end:
  return;
} // FB_HYDAXIS3_body__() 





void FB_HYDAXIS1_init__(FB_HYDAXIS1 *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->BSTART,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BSTOP,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BESTOP,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BRESET,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->UIDIR,0,retain)
  __INIT_VAR(data__->UIMODE,0,retain)
  __INIT_VAR(data__->UIPRES,0,retain)
  __INIT_VAR(data__->UISPD,0,retain)
  __INIT_VAR(data__->UDIPOS,0,retain)
  __INIT_VAR(data__->UIFORCE,0,retain)
  __INIT_VAR(data__->UIACC,0,retain)
  __INIT_VAR(data__->UIDEC,0,retain)
  __INIT_VAR(data__->UIJERK,0,retain)
  __INIT_VAR(data__->BBUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BDONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BALARM,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DWALARMID,0,retain)
  __INIT_VAR(data__->ISTEP,0,retain)
  HYD_CREATEMOTION_init__(&data__->FBHYD_CREATEMOTION,retain);
  HYD_MOVEABSOLUTE_init__(&data__->FBHYD_MOVEABSOLUTE,retain);
  HYD_MOVEVELOCITY_init__(&data__->FBHYD_MOVEVELOCITY,retain);
  HYD_PRESSUREHANDLE_init__(&data__->FBHYD_PRESSUREHANDLE,retain);
  HYD_READSIMFEEDBACK_init__(&data__->FBHYD_READSIMFEEDBACK,retain);
  HYD_STOP_init__(&data__->FBHYD_STOP,retain);
  HYD_STOP_init__(&data__->FBHYD_ESTOP,retain);
  HYD_RESET_init__(&data__->FBHYD_RESET,retain);
  __INIT_VAR(data__->SIAXISID,0,retain)
  __INIT_VAR(data__->RPOSITION,0,retain)
  __INIT_VAR(data__->RVELOCTITY,0,retain)
  __INIT_VAR(data__->RFLOW,0,retain)
  __INIT_VAR(data__->RPERSSURE,0,retain)
  __INIT_VAR(data__->IDIR,0,retain)
  R_TRIG_init__(&data__->RTRIGSTART,retain);
  R_TRIG_init__(&data__->RTRIGSTOP,retain);
  R_TRIG_init__(&data__->RTRIGESTOP,retain);
  R_TRIG_init__(&data__->RTRIGRESET,retain);
  __INIT_VAR(data__->RTARGETPOS,0,retain)
  __INIT_VAR(data__->RTARGETVEL,0,retain)
  __INIT_VAR(data__->RTARGETPRES,0,retain)
  __INIT_VAR(data__->RTARGETACC,0,retain)
  __INIT_VAR(data__->RTARGETDEC,0,retain)
  __INIT_VAR(data__->RTARGETJERK,0,retain)
}

// Code part
void FB_HYDAXIS1_body__(FB_HYDAXIS1 *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __SET_VAR(data__->RTRIGSTART.,CLK,,__GET_VAR(data__->BSTART,));
  R_TRIG_body__(&data__->RTRIGSTART);
  __SET_VAR(data__->RTRIGSTOP.,CLK,,__GET_VAR(data__->BSTOP,));
  R_TRIG_body__(&data__->RTRIGSTOP);
  __SET_VAR(data__->RTRIGESTOP.,CLK,,__GET_VAR(data__->BESTOP,));
  R_TRIG_body__(&data__->RTRIGESTOP);
  __SET_VAR(data__->RTRIGRESET.,CLK,,__GET_VAR(data__->BRESET,));
  R_TRIG_body__(&data__->RTRIGRESET);
  __SET_VAR(data__->,RTARGETVEL,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UISPD,)));
  __SET_VAR(data__->,RTARGETACC,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UIACC,)));
  __SET_VAR(data__->,RTARGETDEC,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UIDEC,)));
  __SET_VAR(data__->,RTARGETJERK,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UIJERK,)));
  __SET_VAR(data__->,RTARGETPOS,,UDINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UDINT)__GET_VAR(data__->UDIPOS,)));
  if ((__GET_VAR(data__->UIPRES,) > 0)) {
    __SET_VAR(data__->,RTARGETPRES,,UINT_TO_REAL(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (UINT)__GET_VAR(data__->UIPRES,)));
  } else if ((__GET_VAR(data__->UIFORCE,) > 0)) {
    __SET_VAR(data__->,RTARGETPRES,,UINT_TO_REAL(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (UINT)__GET_VAR(data__->UIFORCE,)));
  } else {
    __SET_VAR(data__->,RTARGETPRES,,0.0);
  };
  if ((__GET_VAR(data__->UIDIR,) == 2)) {
    __SET_VAR(data__->,IDIR,,-1);
  } else {
    __SET_VAR(data__->,IDIR,,UINT_TO_SINT(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (UINT)__GET_VAR(data__->UIDIR,)));
  };
  {
    INT __case_expression = __GET_VAR(data__->ISTEP,);
    if ((__case_expression == 0)) {
      if (__GET_VAR(data__->FBHYD_CREATEMOTION.DONE,)) {
        __SET_VAR(data__->,SIAXISID,,__GET_VAR(data__->FBHYD_CREATEMOTION.AXISID,));
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->FBHYD_CREATEMOTION.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,0x1);
        __SET_VAR(data__->,ISTEP,,90);
      };
    }
    else if ((__case_expression == 10)) {
      if (__GET_VAR(data__->RTRIGESTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,60);
      } else if (__GET_VAR(data__->RTRIGSTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,50);
      } else if ((__GET_VAR(data__->RTRIGRESET.Q,) && __GET_VAR(data__->BALARM,))) {
        __SET_VAR(data__->,ISTEP,,70);
      } else if (__GET_VAR(data__->RTRIGSTART.Q,)) {
        {
          UINT __case_expression = __GET_VAR(data__->UIMODE,);
          if ((__case_expression == 1)) {
            __SET_VAR(data__->,ISTEP,,20);
          }
          else if ((__case_expression == 2)) {
            __SET_VAR(data__->,ISTEP,,30);
          }
          else if ((__case_expression == 3)) {
            __SET_VAR(data__->,ISTEP,,40);
          }
        };
      };
    }
    else if ((__case_expression == 20)) {
      if (__GET_VAR(data__->FBHYD_MOVEABSOLUTE.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->FBHYD_MOVEABSOLUTE.ERRORID,));
        __SET_VAR(data__->,ISTEP,,90);
      } else if (__GET_VAR(data__->FBHYD_MOVEABSOLUTE.DONE,)) {
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->RTRIGESTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,60);
      } else if (__GET_VAR(data__->RTRIGSTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,50);
      } else if (__GET_VAR(data__->RTRIGSTART.Q,)) {
        if ((__GET_VAR(data__->UIMODE,) == 1)) {
          __SET_VAR(data__->,ISTEP,,25);
        } else {
          {
            UINT __case_expression = __GET_VAR(data__->UIMODE,);
            if ((__case_expression == 2)) {
              __SET_VAR(data__->,ISTEP,,30);
            }
            else if ((__case_expression == 3)) {
              __SET_VAR(data__->,ISTEP,,40);
            }
          };
        };
      };
    }
    else if ((__case_expression == 25)) {
      __SET_VAR(data__->,ISTEP,,20);
    }
    else if ((__case_expression == 30)) {
      if (__GET_VAR(data__->FBHYD_MOVEVELOCITY.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->FBHYD_MOVEVELOCITY.ERRORID,));
        __SET_VAR(data__->,ISTEP,,90);
      } else if (__GET_VAR(data__->RTRIGESTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,60);
      } else if (__GET_VAR(data__->RTRIGSTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,50);
      } else if (__GET_VAR(data__->RTRIGSTART.Q,)) {
        if ((__GET_VAR(data__->UIMODE,) == 2)) {
          __SET_VAR(data__->,ISTEP,,35);
        } else {
          {
            UINT __case_expression = __GET_VAR(data__->UIMODE,);
            if ((__case_expression == 1)) {
              __SET_VAR(data__->,ISTEP,,20);
            }
            else if ((__case_expression == 3)) {
              __SET_VAR(data__->,ISTEP,,40);
            }
          };
        };
      };
    }
    else if ((__case_expression == 35)) {
      __SET_VAR(data__->,ISTEP,,30);
    }
    else if ((__case_expression == 40)) {
      if (__GET_VAR(data__->FBHYD_PRESSUREHANDLE.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.ERRORID,));
        __SET_VAR(data__->,ISTEP,,90);
      } else if (__GET_VAR(data__->RTRIGESTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,60);
      } else if (__GET_VAR(data__->RTRIGSTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,50);
      } else if (__GET_VAR(data__->RTRIGSTART.Q,)) {
        if ((__GET_VAR(data__->UIMODE,) == 3)) {
          __SET_VAR(data__->,ISTEP,,45);
        } else {
          {
            UINT __case_expression = __GET_VAR(data__->UIMODE,);
            if ((__case_expression == 1)) {
              __SET_VAR(data__->,ISTEP,,20);
            }
            else if ((__case_expression == 2)) {
              __SET_VAR(data__->,ISTEP,,30);
            }
          };
        };
      };
    }
    else if ((__case_expression == 45)) {
      __SET_VAR(data__->,ISTEP,,40);
    }
    else if ((__case_expression == 50)) {
      if (__GET_VAR(data__->FBHYD_STOP.DONE,)) {
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->FBHYD_STOP.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->FBHYD_STOP.ERRORID,));
        __SET_VAR(data__->,ISTEP,,90);
      };
    }
    else if ((__case_expression == 60)) {
      __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->,DWALARMID,,0xFFFF);
      if ((__GET_VAR(data__->FBHYD_ESTOP.DONE,) || __GET_VAR(data__->FBHYD_ESTOP.ERROR,))) {
        __SET_VAR(data__->,ISTEP,,90);
      };
    }
    else if ((__case_expression == 70)) {
      if (__GET_VAR(data__->FBHYD_RESET.DONE,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(FALSE));
        __SET_VAR(data__->,DWALARMID,,0);
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->FBHYD_RESET.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->FBHYD_RESET.ERRORID,));
        __SET_VAR(data__->,ISTEP,,90);
      };
    }
    else if ((__case_expression == 90)) {
      if (__GET_VAR(data__->RTRIGRESET.Q,)) {
        __SET_VAR(data__->,ISTEP,,70);
      };
    }
  };
  __SET_VAR(data__->FBHYD_CREATEMOTION.,USE_SIMULATION,,__BOOL_LITERAL(TRUE));
  HYD_CREATEMOTION_body__(&data__->FBHYD_CREATEMOTION);
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 20));
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,POSITION,,__GET_VAR(data__->RTARGETPOS,));
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,VELOCITY,,__GET_VAR(data__->RTARGETVEL,));
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,ACCELERATION,,__GET_VAR(data__->RTARGETACC,));
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,DECELERATION,,__GET_VAR(data__->RTARGETDEC,));
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,JERK,,__GET_VAR(data__->RTARGETJERK,));
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,DIRECTION,,__GET_VAR(data__->IDIR,));
  __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,BUFFERMODE,,0);
  HYD_MOVEABSOLUTE_body__(&data__->FBHYD_MOVEABSOLUTE);
  __SET_VAR(data__->FBHYD_MOVEVELOCITY.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBHYD_MOVEVELOCITY.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 30));
  __SET_VAR(data__->FBHYD_MOVEVELOCITY.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
  __SET_VAR(data__->FBHYD_MOVEVELOCITY.,VELOCITY,,__GET_VAR(data__->RTARGETVEL,));
  __SET_VAR(data__->FBHYD_MOVEVELOCITY.,ACCELERATION,,__GET_VAR(data__->RTARGETACC,));
  __SET_VAR(data__->FBHYD_MOVEVELOCITY.,DECELERATION,,__GET_VAR(data__->RTARGETDEC,));
  __SET_VAR(data__->FBHYD_MOVEVELOCITY.,JERK,,__GET_VAR(data__->RTARGETJERK,));
  __SET_VAR(data__->FBHYD_MOVEVELOCITY.,DIRECTION,,__GET_VAR(data__->IDIR,));
  __SET_VAR(data__->FBHYD_MOVEVELOCITY.,BUFFERMODE,,0);
  HYD_MOVEVELOCITY_body__(&data__->FBHYD_MOVEVELOCITY);
  __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 40));
  __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
  __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,PRESSURE,,__GET_VAR(data__->RTARGETPRES,));
  __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,PRESSURERAMPRATE,,__GET_VAR(data__->RTARGETACC,));
  __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,DURATION,,0.0);
  __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,BUFFERMODE,,0);
  HYD_PRESSUREHANDLE_body__(&data__->FBHYD_PRESSUREHANDLE);
  __SET_VAR(data__->FBHYD_STOP.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBHYD_STOP.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 50));
  __SET_VAR(data__->FBHYD_STOP.,DECELERATION,,__GET_VAR(data__->RTARGETDEC,));
  HYD_STOP_body__(&data__->FBHYD_STOP);
  __SET_VAR(data__->FBHYD_ESTOP.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBHYD_ESTOP.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 60));
  __SET_VAR(data__->FBHYD_ESTOP.,DECELERATION,,99999.0);
  HYD_STOP_body__(&data__->FBHYD_ESTOP);
  __SET_VAR(data__->FBHYD_RESET.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBHYD_RESET.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 70));
  HYD_RESET_body__(&data__->FBHYD_RESET);
  __SET_VAR(data__->FBHYD_READSIMFEEDBACK.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBHYD_READSIMFEEDBACK.,ENABLE,,__BOOL_LITERAL(TRUE));
  HYD_READSIMFEEDBACK_body__(&data__->FBHYD_READSIMFEEDBACK);
  if (__GET_VAR(data__->FBHYD_READSIMFEEDBACK.VALID,)) {
    __SET_VAR(data__->,RPOSITION,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.POSITION,));
    __SET_VAR(data__->,RVELOCTITY,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.VELOCITY,));
    __SET_VAR(data__->,RFLOW,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.FLOW,));
    __SET_VAR(data__->,RPERSSURE,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.PRESSURE,));
  };
  {
    INT __case_expression = __GET_VAR(data__->ISTEP,);
    if ((__case_expression == 10)) {
      __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(FALSE));
      __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(TRUE));
    }
    else if ((__case_expression == 20) ||
             (__case_expression == 25)) {
      __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBHYD_MOVEABSOLUTE.BUSY,));
      __SET_VAR(data__->,BDONE,,__GET_VAR(data__->FBHYD_MOVEABSOLUTE.DONE,));
    }
    else if ((__case_expression == 30) ||
             (__case_expression == 35)) {
      __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBHYD_MOVEVELOCITY.BUSY,));
      __SET_VAR(data__->,BDONE,,__GET_VAR(data__->FBHYD_MOVEVELOCITY.INVELOCITY,));
    }
    else if ((__case_expression == 40) ||
             (__case_expression == 45)) {
      __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.BUSY,));
      __SET_VAR(data__->,BDONE,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.INPRESSURE,));
    }
    else if ((__case_expression == 50)) {
      __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBHYD_STOP.BUSY,));
      __SET_VAR(data__->,BDONE,,__GET_VAR(data__->FBHYD_STOP.DONE,));
    }
    else if ((__case_expression == 60)) {
      __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(FALSE));
    }
    else if ((__case_expression == 70)) {
      __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBHYD_RESET.BUSY,));
      __SET_VAR(data__->,BDONE,,__GET_VAR(data__->FBHYD_RESET.DONE,));
    }
    else if ((__case_expression == 90)) {
      __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(FALSE));
      __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(FALSE));
    }
  };

  goto __end;

__end:
  return;
} // FB_HYDAXIS1_body__() 





void FB_HYDAXIS2_init__(FB_HYDAXIS2 *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->BSTART,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BSTOP,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BESTOP,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BRESET,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->UIDIR,0,retain)
  __INIT_VAR(data__->UIMODE,0,retain)
  __INIT_VAR(data__->UIPRES,0,retain)
  __INIT_VAR(data__->UISPD,0,retain)
  __INIT_VAR(data__->UDIPOS,0,retain)
  __INIT_VAR(data__->UIFORCE,0,retain)
  __INIT_VAR(data__->UIACC,0,retain)
  __INIT_VAR(data__->UIDEC,0,retain)
  __INIT_VAR(data__->UIJERK,0,retain)
  __INIT_VAR(data__->BBUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BDONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BALARM,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DWALARMID,0,retain)
  __INIT_VAR(data__->ISTEP,0,retain)
  __INIT_VAR(data__->IDIR,0,retain)
  HYD_CREATEMOTION_init__(&data__->FBCREATE,retain);
  HYD_MOVEABSOLUTE_init__(&data__->FBMOVEABS,retain);
  HYD_MOVEVELOCITY_init__(&data__->FBMOVEVEL,retain);
  HYD_PRESSUREHANDLE_init__(&data__->FBPRESHDL,retain);
  HYD_STOP_init__(&data__->FBSTOP,retain);
  HYD_STOP_init__(&data__->FBESTOP,retain);
  HYD_RESET_init__(&data__->FBRESET,retain);
  HYD_READSIMFEEDBACK_init__(&data__->FBREADFB,retain);
  __INIT_VAR(data__->SIAXISID,0,retain)
  __INIT_VAR(data__->RPOSITION,0,retain)
  __INIT_VAR(data__->RVELOCTITY,0,retain)
  __INIT_VAR(data__->RFLOW,0,retain)
  __INIT_VAR(data__->RPERSSURE,0,retain)
  R_TRIG_init__(&data__->RTRIGSTART,retain);
  R_TRIG_init__(&data__->RTRIGSTOP,retain);
  R_TRIG_init__(&data__->RTRIGESTOP,retain);
  R_TRIG_init__(&data__->RTRIGRESET,retain);
  __INIT_VAR(data__->UIACTIVEMODE,0,retain)
  __INIT_VAR(data__->BCMDEXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->RPOS,0,retain)
  __INIT_VAR(data__->RVEL,0,retain)
  __INIT_VAR(data__->RPRES,0,retain)
  __INIT_VAR(data__->RACC,0,retain)
  __INIT_VAR(data__->RDEC,0,retain)
  __INIT_VAR(data__->RJERK,0,retain)
  __INIT_VAR(data__->BFBDONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BFBERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->WFBERRORID,0,retain)
}

// Code part
void FB_HYDAXIS2_body__(FB_HYDAXIS2 *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __SET_VAR(data__->RTRIGSTART.,CLK,,__GET_VAR(data__->BSTART,));
  R_TRIG_body__(&data__->RTRIGSTART);
  __SET_VAR(data__->RTRIGSTOP.,CLK,,__GET_VAR(data__->BSTOP,));
  R_TRIG_body__(&data__->RTRIGSTOP);
  __SET_VAR(data__->RTRIGESTOP.,CLK,,__GET_VAR(data__->BESTOP,));
  R_TRIG_body__(&data__->RTRIGESTOP);
  __SET_VAR(data__->RTRIGRESET.,CLK,,__GET_VAR(data__->BRESET,));
  R_TRIG_body__(&data__->RTRIGRESET);
  __SET_VAR(data__->,RVEL,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UISPD,)));
  __SET_VAR(data__->,RACC,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UIACC,)));
  __SET_VAR(data__->,RDEC,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UIDEC,)));
  __SET_VAR(data__->,RJERK,,UINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)__GET_VAR(data__->UIJERK,)));
  __SET_VAR(data__->,RPOS,,UDINT_TO_REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UDINT)__GET_VAR(data__->UDIPOS,)));
  __SET_VAR(data__->,RPRES,,MAX__REAL__REAL(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (UINT)2,
    (REAL)UINT_TO_REAL(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (UINT)__GET_VAR(data__->UIPRES,)),
    (REAL)UINT_TO_REAL(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (UINT)__GET_VAR(data__->UIFORCE,))));
  if ((__GET_VAR(data__->UIDIR,) == 2)) {
    __SET_VAR(data__->,IDIR,,-1);
  } else {
    __SET_VAR(data__->,IDIR,,UINT_TO_SINT(
      (BOOL)__BOOL_LITERAL(TRUE),
      NULL,
      (UINT)__GET_VAR(data__->UIDIR,)));
  };
  {
    INT __case_expression = __GET_VAR(data__->ISTEP,);
    if ((__case_expression == 0)) {
      if (__GET_VAR(data__->FBCREATE.DONE,)) {
        __SET_VAR(data__->,SIAXISID,,__GET_VAR(data__->FBCREATE.AXISID,));
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->FBCREATE.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,0x1);
        __SET_VAR(data__->,ISTEP,,900);
      };
    }
    else if ((__case_expression == 10)) {
      __SET_VAR(data__->,BCMDEXECUTE,,__BOOL_LITERAL(FALSE));
      if (__GET_VAR(data__->RTRIGESTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,300);
      } else if (__GET_VAR(data__->RTRIGSTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,200);
      } else if ((__GET_VAR(data__->RTRIGRESET.Q,) && __GET_VAR(data__->BALARM,))) {
        __SET_VAR(data__->,ISTEP,,400);
      } else if (((__GET_VAR(data__->RTRIGSTART.Q,) && (__GET_VAR(data__->UIMODE,) >= 1)) && (__GET_VAR(data__->UIMODE,) <= 3))) {
        __SET_VAR(data__->,UIACTIVEMODE,,__GET_VAR(data__->UIMODE,));
        __SET_VAR(data__->,ISTEP,,100);
      };
    }
    else if ((__case_expression == 100)) {
      __SET_VAR(data__->,BCMDEXECUTE,,__BOOL_LITERAL(TRUE));
      {
        UINT __case_expression = __GET_VAR(data__->UIACTIVEMODE,);
        if ((__case_expression == 1)) {
          __SET_VAR(data__->,BFBDONE,,__GET_VAR(data__->FBMOVEABS.DONE,));
          __SET_VAR(data__->,BFBERROR,,__GET_VAR(data__->FBMOVEABS.ERROR,));
          __SET_VAR(data__->,WFBERRORID,,__GET_VAR(data__->FBMOVEABS.ERRORID,));
        }
        else if ((__case_expression == 2)) {
          __SET_VAR(data__->,BFBDONE,,__GET_VAR(data__->FBMOVEVEL.INVELOCITY,));
          __SET_VAR(data__->,BFBERROR,,__GET_VAR(data__->FBMOVEVEL.ERROR,));
          __SET_VAR(data__->,WFBERRORID,,__GET_VAR(data__->FBMOVEVEL.ERRORID,));
        }
        else if ((__case_expression == 3)) {
          __SET_VAR(data__->,BFBDONE,,__GET_VAR(data__->FBPRESHDL.INPRESSURE,));
          __SET_VAR(data__->,BFBERROR,,__GET_VAR(data__->FBPRESHDL.ERROR,));
          __SET_VAR(data__->,WFBERRORID,,__GET_VAR(data__->FBPRESHDL.ERRORID,));
        }
      };
      if (__GET_VAR(data__->BFBERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->WFBERRORID,));
        __SET_VAR(data__->,ISTEP,,900);
      } else if (__GET_VAR(data__->BFBDONE,)) {
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->RTRIGESTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,300);
      } else if (__GET_VAR(data__->RTRIGSTOP.Q,)) {
        __SET_VAR(data__->,ISTEP,,200);
      } else if (__GET_VAR(data__->RTRIGSTART.Q,)) {
        if ((__GET_VAR(data__->UIMODE,) == __GET_VAR(data__->UIACTIVEMODE,))) {
          __SET_VAR(data__->,ISTEP,,105);
        } else if (((__GET_VAR(data__->UIMODE,) >= 1) && (__GET_VAR(data__->UIMODE,) <= 3))) {
          __SET_VAR(data__->,UIACTIVEMODE,,__GET_VAR(data__->UIMODE,));
        };
      };
    }
    else if ((__case_expression == 105)) {
      __SET_VAR(data__->,BCMDEXECUTE,,__BOOL_LITERAL(FALSE));
      __SET_VAR(data__->,ISTEP,,100);
    }
    else if ((__case_expression == 200)) {
      if (__GET_VAR(data__->FBSTOP.DONE,)) {
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->FBSTOP.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->FBSTOP.ERRORID,));
        __SET_VAR(data__->,ISTEP,,900);
      };
    }
    else if ((__case_expression == 300)) {
      if ((__GET_VAR(data__->FBESTOP.DONE,) || __GET_VAR(data__->FBESTOP.ERROR,))) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,0xFFFF);
        __SET_VAR(data__->,ISTEP,,900);
      };
    }
    else if ((__case_expression == 400)) {
      if (__GET_VAR(data__->FBRESET.DONE,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(FALSE));
        __SET_VAR(data__->,DWALARMID,,0);
        __SET_VAR(data__->,ISTEP,,10);
      } else if (__GET_VAR(data__->FBRESET.ERROR,)) {
        __SET_VAR(data__->,BALARM,,__BOOL_LITERAL(TRUE));
        __SET_VAR(data__->,DWALARMID,,__GET_VAR(data__->FBRESET.ERRORID,));
        __SET_VAR(data__->,ISTEP,,900);
      };
    }
    else if ((__case_expression == 900)) {
      if (__GET_VAR(data__->RTRIGRESET.Q,)) {
        __SET_VAR(data__->,ISTEP,,400);
      };
    }
  };
  __SET_VAR(data__->FBCREATE.,USE_SIMULATION,,__BOOL_LITERAL(TRUE));
  HYD_CREATEMOTION_body__(&data__->FBCREATE);
  __SET_VAR(data__->FBMOVEABS.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBMOVEABS.,EXECUTE,,((__GET_VAR(data__->UIACTIVEMODE,) == 1) && __GET_VAR(data__->BCMDEXECUTE,)));
  __SET_VAR(data__->FBMOVEABS.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
  __SET_VAR(data__->FBMOVEABS.,POSITION,,__GET_VAR(data__->RPOS,));
  __SET_VAR(data__->FBMOVEABS.,VELOCITY,,__GET_VAR(data__->RVEL,));
  __SET_VAR(data__->FBMOVEABS.,ACCELERATION,,__GET_VAR(data__->RACC,));
  __SET_VAR(data__->FBMOVEABS.,DECELERATION,,__GET_VAR(data__->RDEC,));
  __SET_VAR(data__->FBMOVEABS.,JERK,,__GET_VAR(data__->RJERK,));
  __SET_VAR(data__->FBMOVEABS.,DIRECTION,,__GET_VAR(data__->IDIR,));
  __SET_VAR(data__->FBMOVEABS.,BUFFERMODE,,0);
  HYD_MOVEABSOLUTE_body__(&data__->FBMOVEABS);
  __SET_VAR(data__->FBMOVEVEL.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBMOVEVEL.,EXECUTE,,((__GET_VAR(data__->UIACTIVEMODE,) == 2) && __GET_VAR(data__->BCMDEXECUTE,)));
  __SET_VAR(data__->FBMOVEVEL.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
  __SET_VAR(data__->FBMOVEVEL.,VELOCITY,,__GET_VAR(data__->RVEL,));
  __SET_VAR(data__->FBMOVEVEL.,ACCELERATION,,__GET_VAR(data__->RACC,));
  __SET_VAR(data__->FBMOVEVEL.,DECELERATION,,__GET_VAR(data__->RDEC,));
  __SET_VAR(data__->FBMOVEVEL.,JERK,,__GET_VAR(data__->RJERK,));
  __SET_VAR(data__->FBMOVEVEL.,DIRECTION,,__GET_VAR(data__->IDIR,));
  __SET_VAR(data__->FBMOVEVEL.,BUFFERMODE,,0);
  HYD_MOVEVELOCITY_body__(&data__->FBMOVEVEL);
  __SET_VAR(data__->FBPRESHDL.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBPRESHDL.,EXECUTE,,((__GET_VAR(data__->UIACTIVEMODE,) == 3) && __GET_VAR(data__->BCMDEXECUTE,)));
  __SET_VAR(data__->FBPRESHDL.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
  __SET_VAR(data__->FBPRESHDL.,PRESSURE,,__GET_VAR(data__->RPRES,));
  __SET_VAR(data__->FBPRESHDL.,PRESSURERAMPRATE,,__GET_VAR(data__->RACC,));
  __SET_VAR(data__->FBPRESHDL.,DURATION,,0.0);
  __SET_VAR(data__->FBPRESHDL.,BUFFERMODE,,0);
  HYD_PRESSUREHANDLE_body__(&data__->FBPRESHDL);
  __SET_VAR(data__->FBSTOP.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBSTOP.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 200));
  __SET_VAR(data__->FBSTOP.,DECELERATION,,__GET_VAR(data__->RDEC,));
  HYD_STOP_body__(&data__->FBSTOP);
  __SET_VAR(data__->FBESTOP.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBESTOP.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 300));
  __SET_VAR(data__->FBESTOP.,DECELERATION,,99999.0);
  HYD_STOP_body__(&data__->FBESTOP);
  __SET_VAR(data__->FBRESET.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBRESET.,EXECUTE,,(__GET_VAR(data__->ISTEP,) == 400));
  HYD_RESET_body__(&data__->FBRESET);
  __SET_VAR(data__->FBREADFB.,AXISID,,__GET_VAR(data__->SIAXISID,));
  __SET_VAR(data__->FBREADFB.,ENABLE,,__BOOL_LITERAL(TRUE));
  HYD_READSIMFEEDBACK_body__(&data__->FBREADFB);
  if (__GET_VAR(data__->FBREADFB.VALID,)) {
    __SET_VAR(data__->,RPOSITION,,__GET_VAR(data__->FBREADFB.POSITION,));
    __SET_VAR(data__->,RVELOCTITY,,__GET_VAR(data__->FBREADFB.VELOCITY,));
    __SET_VAR(data__->,RFLOW,,__GET_VAR(data__->FBREADFB.FLOW,));
    __SET_VAR(data__->,RPERSSURE,,__GET_VAR(data__->FBREADFB.PRESSURE,));
  };
  if ((__GET_VAR(data__->ISTEP,) == 10)) {
    __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(FALSE));
    __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(TRUE));
  } else if (((__GET_VAR(data__->ISTEP,) == 100) || (__GET_VAR(data__->ISTEP,) == 105))) {
    __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(TRUE));
    __SET_VAR(data__->,BDONE,,__GET_VAR(data__->BFBDONE,));
  } else if ((__GET_VAR(data__->ISTEP,) == 200)) {
    __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBSTOP.BUSY,));
    __SET_VAR(data__->,BDONE,,__GET_VAR(data__->FBSTOP.DONE,));
  } else if ((__GET_VAR(data__->ISTEP,) == 300)) {
    __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(TRUE));
    __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(FALSE));
  } else if ((__GET_VAR(data__->ISTEP,) == 400)) {
    __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBRESET.BUSY,));
    __SET_VAR(data__->,BDONE,,__GET_VAR(data__->FBRESET.DONE,));
  } else {
    __SET_VAR(data__->,BBUSY,,__BOOL_LITERAL(FALSE));
    __SET_VAR(data__->,BDONE,,__BOOL_LITERAL(FALSE));
  };

  goto __end;

__end:
  return;
} // FB_HYDAXIS2_body__() 





void FB_WORDBITRISING_init__(FB_WORDBITRISING *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->WORDIN,0,retain)
  __INIT_VAR(data__->BITPOS,0,retain)
  __INIT_VAR(data__->RISING,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->PREVBIT,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->CURRENTBIT,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->WTMP,0,retain)
}

// Code part
void FB_WORDBITRISING_body__(FB_WORDBITRISING *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __SET_VAR(data__->,WTMP,,(SHR__WORD__WORD__INT(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (WORD)__GET_VAR(data__->WORDIN,),
    (INT)__GET_VAR(data__->BITPOS,)) & 0x1));
  __SET_VAR(data__->,CURRENTBIT,,(__GET_VAR(data__->WTMP,) == 0x1));
  __SET_VAR(data__->,RISING,,(__GET_VAR(data__->CURRENTBIT,) && !(__GET_VAR(data__->PREVBIT,))));
  __SET_VAR(data__->,PREVBIT,,__GET_VAR(data__->CURRENTBIT,));

  goto __end;

__end:
  return;
} // FB_WORDBITRISING_body__() 





void TESTVECLOCITY_init__(TESTVECLOCITY *data__, BOOL retain) {
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
  HYD_MOVEVELOCITY_init__(&data__->FBHYD_MOVEVELOCITY,retain);
  HYD_READSIMFEEDBACK_init__(&data__->FBHYD_READSIMFEEDBACK,retain);
  HYD_READSTATUS_init__(&data__->FBHYD_READSTATUS,retain);
  HYD_SETAXISFEEDBACK_init__(&data__->FBHYD_SETAXISFEEDBACK,retain);
  __INIT_VAR(data__->ISTATE,0,retain)
  __INIT_VAR(data__->BACTIVE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BEXECUTE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->TCYCLE,0,retain)
  FB_WORDBITRISING_init__(&data__->CMD0_DETECT,retain);
  HYD_STOP_init__(&data__->FBHYD_STOP,retain);
  __INIT_VAR(data__->FSETVEL,20.0,retain)
  __INIT_EXTERNAL(WORD,GGRP_MBUS_ACTPOS,data__->GGRP_MBUS_ACTPOS,retain)
  __INIT_EXTERNAL(WORD,GGRP_MBUS_COMMAD,data__->GGRP_MBUS_COMMAD,retain)
  __INIT_EXTERNAL(WORD,GGRP_MBUS_ACTVEL,data__->GGRP_MBUS_ACTVEL,retain)
}

// Code part
void TESTVECLOCITY_body__(TESTVECLOCITY *data__) {
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
    __SET_VAR(data__->FBHYD_READSIMFEEDBACK.,AXISID,,__GET_VAR(data__->IAXIS0,));
    __SET_VAR(data__->FBHYD_READSIMFEEDBACK.,ENABLE,,1);
    HYD_READSIMFEEDBACK_body__(&data__->FBHYD_READSIMFEEDBACK);
    __SET_VAR(data__->,FPOS,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.POSITION));
    __SET_VAR(data__->,FVEL,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.VELOCITY));
    __SET_VAR(data__->,FFLOW,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.FLOW));
    __SET_VAR(data__->,FPRESSURE,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.PRESSURE));
    __SET_VAR(data__->FBHYD_READSTATUS.,AXISID,,__GET_VAR(data__->IAXIS0,));
    __SET_VAR(data__->FBHYD_READSTATUS.,ENABLE,,1);
    HYD_READSTATUS_body__(&data__->FBHYD_READSTATUS);
    __SET_VAR(data__->,ISTATE,,__GET_VAR(data__->FBHYD_READSTATUS.STATE));
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,AXISID,,0);
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ENABLE,,1);
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
        __SET_VAR(data__->FBHYD_MOVEVELOCITY.,EXECUTE,,__BOOL_LITERAL(FALSE));
        HYD_MOVEVELOCITY_body__(&data__->FBHYD_MOVEVELOCITY);
      };
    }
    else if ((__case_expression == 1)) {
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,EXECUTE,,__GET_VAR(data__->BEXECUTE,));
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,CONTINUOUSUPDATE,,1);
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,VELOCITY,,__GET_VAR(data__->FSETVEL,));
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,ACCELERATION,,200);
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,DECELERATION,,0);
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,DIRECTION,,1);
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,BUFFERMODE,,0);
      HYD_MOVEVELOCITY_body__(&data__->FBHYD_MOVEVELOCITY);
      __SET_VAR(data__->,BERROR,,__GET_VAR(data__->FBHYD_MOVEVELOCITY.ERROR,));
      __SET_VAR(data__->,WERRORID,,__GET_VAR(data__->FBHYD_MOVEVELOCITY.ERRORID,));
      __SET_VAR(data__->,BACTIVE,,__GET_VAR(data__->FBHYD_MOVEVELOCITY.ACTIVE,));
      if (__GET_VAR(data__->FBHYD_MOVEVELOCITY.INVELOCITY,)) {
        __SET_VAR(data__->TDELAY.,IN,,1);
        __SET_VAR(data__->TDELAY.,PT,,__time_to_timespec(1, 0, 15, 0, 0, 0));
        TON_body__(&data__->TDELAY);
      };
    }
    else if ((__case_expression == 2)) {
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,EXECUTE,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,VELOCITY,,20);
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,ACCELERATION,,200);
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,DECELERATION,,200);
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,DIRECTION,,-1);
      __SET_VAR(data__->FBHYD_MOVEVELOCITY.,BUFFERMODE,,0);
      HYD_MOVEVELOCITY_body__(&data__->FBHYD_MOVEVELOCITY);
      if (__GET_VAR(data__->FBHYD_MOVEVELOCITY.INVELOCITY,)) {
        __SET_VAR(data__->TDELAY.,IN,,1);
        __SET_VAR(data__->TDELAY.,PT,,__time_to_timespec(1, 0, 5, 0, 0, 0));
        TON_body__(&data__->TDELAY);
        if (__GET_VAR(data__->TDELAY.Q,)) {
          __SET_VAR(data__->,ISTEP,,1);
          __SET_VAR(data__->TDELAY.,IN,,0);
          TON_body__(&data__->TDELAY);
          __SET_VAR(data__->FBHYD_MOVEVELOCITY.,AXISID,,__GET_VAR(data__->IAXIS0,));
          __SET_VAR(data__->FBHYD_MOVEVELOCITY.,EXECUTE,,__BOOL_LITERAL(FALSE));
          HYD_MOVEVELOCITY_body__(&data__->FBHYD_MOVEVELOCITY);
        };
      };
    }
    else if ((__case_expression == 3)) {
      __SET_VAR(data__->FBHYD_STOP.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_STOP.,EXECUTE,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->FBHYD_STOP.,DECELERATION,,50.0);
      HYD_STOP_body__(&data__->FBHYD_STOP);
      __SET_VAR(data__->,BERROR,,__GET_VAR(data__->FBHYD_STOP.ERROR,));
      __SET_VAR(data__->,WERRORID,,__GET_VAR(data__->FBHYD_STOP.ERRORID,));
      if (__GET_VAR(data__->FBHYD_STOP.DONE,)) {
        __SET_VAR(data__->FBHYD_MOVEVELOCITY.,EXECUTE,,__BOOL_LITERAL(FALSE));
        HYD_MOVEVELOCITY_body__(&data__->FBHYD_MOVEVELOCITY);
        __SET_VAR(data__->,ISTEP,,33);
      };
    }
  };
  __SET_EXTERNAL(data__->,GGRP_MBUS_ACTPOS,,REAL_TO_INT(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (REAL)__GET_VAR(data__->FPOS,)));
  __SET_EXTERNAL(data__->,GGRP_MBUS_ACTVEL,,REAL_TO_INT(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (REAL)(__GET_VAR(data__->FVEL,) * 10)));

  goto __end;

__end:
  return;
} // TESTVECLOCITY_body__() 





void TESTMOTION_init__(TESTMOTION *data__, BOOL retain) {
  __INIT_VAR(data__->NEWLOCALVAR0,0,retain)
  __INIT_VAR(data__->NEWLOCALVAR2,0,retain)
  __INIT_VAR(data__->NEWLOCALVAR1,0,retain)
  __INIT_VAR(data__->FPOS,0,retain)
  __INIT_VAR(data__->FVEL,0,retain)
  __INIT_VAR(data__->FFLOW,0,retain)
  __INIT_VAR(data__->FPRESSURE,0,retain)
  __INIT_VAR(data__->ISTEP,-1,retain)
  __INIT_VAR(data__->BERROR,0,retain)
  __INIT_VAR(data__->BVALID,0,retain)
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
  HYD_PRESSUREHANDLE_init__(&data__->FBHYD_PRESSUREHANDLE,retain);
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
    __SET_VAR(data__->FBHYD_READSIMFEEDBACK.,AXISID,,__GET_VAR(data__->IAXIS0,));
    __SET_VAR(data__->FBHYD_READSIMFEEDBACK.,ENABLE,,1);
    HYD_READSIMFEEDBACK_body__(&data__->FBHYD_READSIMFEEDBACK);
    __SET_VAR(data__->,BVALID,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.VALID));
    __SET_VAR(data__->,BERROR,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.ERROR));
    __SET_VAR(data__->,WERRORID,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.ERRORID));
    __SET_VAR(data__->,FPOS,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.POSITION));
    __SET_VAR(data__->,FVEL,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.VELOCITY));
    __SET_VAR(data__->,FFLOW,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.FLOW));
    __SET_VAR(data__->,FPRESSURE,,__GET_VAR(data__->FBHYD_READSIMFEEDBACK.PRESSURE));
    __SET_VAR(data__->FBHYD_READSTATUS.,AXISID,,__GET_VAR(data__->IAXIS0,));
    __SET_VAR(data__->FBHYD_READSTATUS.,ENABLE,,1);
    HYD_READSTATUS_body__(&data__->FBHYD_READSTATUS);
    __SET_VAR(data__->,ISTATE,,__GET_VAR(data__->FBHYD_READSTATUS.STATE));
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,AXISID,,0);
    __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ENABLE,,1);
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
      __SET_VAR(data__->FBHYD_STOP.,EXECUTE,,__BOOL_LITERAL(FALSE));
      HYD_STOP_body__(&data__->FBHYD_STOP);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,CONTINUOUSUPDATE,,__BOOL_LITERAL(TRUE));
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
          __SET_VAR(data__->TDELAY.,IN,,0);
          TON_body__(&data__->TDELAY);
        };
      };
    }
    else if ((__case_expression == 2)) {
      __SET_VAR(data__->FBHYD_STOP.,EXECUTE,,__BOOL_LITERAL(FALSE));
      HYD_STOP_body__(&data__->FBHYD_STOP);
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(TRUE));
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
          __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,AXISID,,__GET_VAR(data__->IAXIS0,));
          __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(FALSE));
          HYD_MOVEABSOLUTE_body__(&data__->FBHYD_MOVEABSOLUTE);
        };
      };
    }
    else if ((__case_expression == 3)) {
      __SET_VAR(data__->FBHYD_STOP.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_STOP.,EXECUTE,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->FBHYD_STOP.,DECELERATION,,50.0);
      HYD_STOP_body__(&data__->FBHYD_STOP);
      __SET_VAR(data__->,BERROR,,__GET_VAR(data__->FBHYD_STOP.ERROR,));
      __SET_VAR(data__->,WERRORID,,__GET_VAR(data__->FBHYD_STOP.ERRORID,));
      if (__GET_VAR(data__->FBHYD_STOP.DONE,)) {
        __SET_VAR(data__->FBHYD_MOVEABSOLUTE.,EXECUTE,,__BOOL_LITERAL(FALSE));
        HYD_MOVEABSOLUTE_body__(&data__->FBHYD_MOVEABSOLUTE);
        __SET_VAR(data__->FBHYD_STOP.,EXECUTE,,__BOOL_LITERAL(FALSE));
        HYD_STOP_body__(&data__->FBHYD_STOP);
      };
    }
  };
  __SET_EXTERNAL(data__->,GGRP_MBUS_ACTPOS,,REAL_TO_INT(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (REAL)__GET_VAR(data__->FPOS,)));
  __SET_EXTERNAL(data__->,GGRP_MBUS_ACTVEL,,REAL_TO_INT(
    (BOOL)__BOOL_LITERAL(TRUE),
    NULL,
    (REAL)(__GET_VAR(data__->FVEL,) * 10)));

  goto __end;

__end:
  return;
} // TESTMOTION_body__() 





void AXISMOTIONDEMO_init__(AXISMOTIONDEMO *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  {
    static const HDY_AXISREF temp = {0,0,0,0,0};
    __SET_VAR(data__->,IHDYAXIS,,temp);
  }
  __INIT_VAR(data__->NEWLOCALVAR0,0,retain)
}

// Code part
void AXISMOTIONDEMO_body__(AXISMOTIONDEMO *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __SET_VAR(data__->,NEWLOCALVAR0,,0);
  __SET_VAR(data__->,IHDYAXIS,.POSITION,3);

  goto __end;

__end:
  return;
} // AXISMOTIONDEMO_body__() 





void AXISMOTIONCONTROL222_init__(AXISMOTIONCONTROL222 *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->EXECUTE,__BOOL_LITERAL(FALSE),retain)
  {
    static const HDY_AXISMOTION222 temp = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    __SET_VAR(data__->,MOTION,,temp);
  }
  __INIT_VAR(data__->ACTIVE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->DONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERROR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->ERRORID,0,retain)
  __INIT_VAR(data__->STATE,0,retain)
  __INIT_VAR(data__->PUMP_SPEED,0,retain)
}

// Code part
void AXISMOTIONCONTROL222_body__(AXISMOTIONCONTROL222 *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __SET_VAR(data__->,ACTIVE,,__BOOL_LITERAL(FALSE));

  goto __end;

__end:
  return;
} // AXISMOTIONCONTROL222_body__() 





void HDY_INJECTSIMULATOR222_init__(HDY_INJECTSIMULATOR222 *data__, BOOL retain) {
  __INIT_VAR(data__->EN,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENO,__BOOL_LITERAL(TRUE),retain)
  __INIT_VAR(data__->ENABLE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->CYCLE_TIME,0,retain)
  __INIT_VAR(data__->CMD_RPM,0,retain)
  __INIT_VAR(data__->PUMP_OWNER_AXIS,0,retain)
  __INIT_VAR(data__->DIRECTION,0,retain)
  __INIT_VAR(data__->PRESSURE_BIAS,0,retain)
  __INIT_VAR(data__->PRESSURE_SCALE,0,retain)
  __INIT_VAR(data__->ACTIVE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->POS_MM,0,retain)
  __INIT_VAR(data__->VEL_MM_S,0,retain)
  __INIT_VAR(data__->PRESSURE_BAR,0,retain)
}

// Code part
void HDY_INJECTSIMULATOR222_body__(HDY_INJECTSIMULATOR222 *data__) {
  // Control execution
  if (!__GET_VAR(data__->EN)) {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(FALSE));
    return;
  }
  else {
    __SET_VAR(data__->,ENO,,__BOOL_LITERAL(TRUE));
  }
  // Initialise TEMP variables

  __SET_VAR(data__->,ACTIVE,,1);

  goto __end;

__end:
  return;
} // HDY_INJECTSIMULATOR222_body__() 





void TESTFBHYDAXIS_init__(TESTFBHYDAXIS *data__, BOOL retain) {
  __INIT_VAR(data__->NEWLOCALVAR0,0,retain)
  FB_HYDAXIS3_init__(&data__->FBAXIS,retain);
  __INIT_VAR(data__->START,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->STOP,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->IMODE,1,retain)
  __INIT_VAR(data__->IDIR,1,retain)
  __INIT_VAR(data__->BUSY,0,retain)
  __INIT_VAR(data__->ERR,0,retain)
  __INIT_VAR(data__->ERRID,0,retain)
  __INIT_VAR(data__->FSETPOS,200,retain)
}

// Code part
void TESTFBHYDAXIS_body__(TESTFBHYDAXIS *data__) {
  // Initialise TEMP variables

  __SET_VAR(data__->FBAXIS.,BSTART,,__GET_VAR(data__->START,));
  __SET_VAR(data__->FBAXIS.,BSTOP,,__GET_VAR(data__->STOP,));
  __SET_VAR(data__->FBAXIS.,UIDIR,,__GET_VAR(data__->IDIR,));
  __SET_VAR(data__->FBAXIS.,UIMODE,,__GET_VAR(data__->IMODE,));
  __SET_VAR(data__->FBAXIS.,UIPRES,,100);
  __SET_VAR(data__->FBAXIS.,UISPD,,5);
  __SET_VAR(data__->FBAXIS.,UDIPOS,,__GET_VAR(data__->FSETPOS,));
  __SET_VAR(data__->FBAXIS.,UIFORCE,,100);
  __SET_VAR(data__->FBAXIS.,UIACC,,200);
  __SET_VAR(data__->FBAXIS.,UIDEC,,200);
  FB_HYDAXIS3_body__(&data__->FBAXIS);
  __SET_VAR(data__->,ERR,,__GET_VAR(data__->FBAXIS.BALARM,));
  __SET_VAR(data__->,ERRID,,__GET_VAR(data__->FBAXIS.DWALARMID,));

  goto __end;

__end:
  return;
} // TESTFBHYDAXIS_body__() 





void TESTSTANDFB_init__(TESTSTANDFB *data__, BOOL retain) {
  __INIT_VAR(data__->ISTEP,0,retain)
  __INIT_VAR(data__->BINVEL,0,retain)
  __INIT_VAR(data__->ICNT,0,retain)
  __INIT_VAR(data__->FSETVEL,0,retain)
  __INIT_VAR(data__->FACTVEL,0,retain)
  __INIT_VAR(data__->FACTPOS,0,retain)
  __INIT_VAR(data__->BBUSY,0,retain)
  __INIT_VAR(data__->BERR,0,retain)
  __INIT_VAR(data__->BABORT,0,retain)
  __INIT_VAR(data__->BACTIVE,0,retain)
  __INIT_VAR(data__->WERR,0,retain)
}

// Code part
void TESTSTANDFB_body__(TESTSTANDFB *data__) {
  // Initialise TEMP variables

  __SET_VAR(data__->,ICNT,,(__GET_VAR(data__->ICNT,) + 1));

  goto __end;

__end:
  return;
} // TESTSTANDFB_body__() 





void TESTPRESSURECTRL_init__(TESTPRESSURECTRL *data__, BOOL retain) {
  __INIT_VAR(data__->NEWLOCALVAR0,0,retain)
  FIRSTORDERSYSTEM_init__(&data__->HYDRAULIC_SYS,retain);
  HYD_GETPUMPREQUEST_init__(&data__->FBHYD_GETPUMPREQUEST,retain);
  HYD_CREATEMOTION_init__(&data__->FBHDY_CREATEMOTION,retain);
  HYD_PRESSUREHANDLE_init__(&data__->FBHYD_PRESSUREHANDLE,retain);
  HYD_READSIMFEEDBACK_init__(&data__->FBHYD_READSIMFEEDBACK,retain);
  HYD_READSTATUS_init__(&data__->FBHYD_READSTATUS,retain);
  HYD_SETAXISFEEDBACK_init__(&data__->FBHYD_SETAXISFEEDBACK,retain);
  __INIT_VAR(data__->FPIDOUTPUT,0,retain)
  __INIT_VAR(data__->FTARGETPRESSURE,100,retain)
  __INIT_VAR(data__->ISTEP,-1,retain)
  __INIT_VAR(data__->IAXIS0,0,retain)
  __INIT_VAR(data__->BINPRESSURE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BDONE,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BACT,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BABORTED,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BERR,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->BBUSY,__BOOL_LITERAL(FALSE),retain)
  __INIT_VAR(data__->WERRID,0,retain)
  __INIT_VAR(data__->TCYCLE,0.001,retain)
  __INIT_VAR(data__->TIMECYCLE,0,retain)
  __INIT_VAR(data__->FACTPRESSURE,0,retain)
  __INIT_VAR(data__->FSETPRESSURE,0,retain)
  HYD_WRITEPARAMETER_init__(&data__->FBHYD_WRITEPARAMETER,retain);
}

// Code part
void TESTPRESSURECTRL_body__(TESTPRESSURECTRL *data__) {
  // Initialise TEMP variables

  __SET_VAR(data__->,TIMECYCLE,,(__GET_VAR(data__->TIMECYCLE,) + __GET_VAR(data__->TCYCLE,)));
  __SET_VAR(data__->,FSETPRESSURE,,__GET_VAR(data__->FTARGETPRESSURE,));
  __SET_VAR(data__->,FACTPRESSURE,,__GET_VAR(data__->HYDRAULIC_SYS.PRESSURE,));
  {
    INT __case_expression = __GET_VAR(data__->ISTEP,);
    if ((__case_expression == 0)) {
      __SET_VAR(data__->FBHDY_CREATEMOTION.,USE_RECIPE,,0);
      __SET_VAR(data__->FBHDY_CREATEMOTION.,FLOW_TO_PUMPSPEED,,20.0);
      __SET_VAR(data__->FBHDY_CREATEMOTION.,USE_SIMULATION,,1);
      HYD_CREATEMOTION_body__(&data__->FBHDY_CREATEMOTION);
      if (__GET_VAR(data__->FBHDY_CREATEMOTION.DONE,)) {
        __SET_VAR(data__->,IAXIS0,,__GET_VAR(data__->FBHDY_CREATEMOTION.AXISID,));
        __SET_VAR(data__->,ISTEP,,(__GET_VAR(data__->ISTEP,) + 1));
        __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,EXECUTE,,__BOOL_LITERAL(FALSE));
        HYD_PRESSUREHANDLE_body__(&data__->FBHYD_PRESSUREHANDLE);
        __SET_VAR(data__->,HYDRAULIC_SYS.ENABLE,,__BOOL_LITERAL(TRUE));
      };
    }
    else if ((__case_expression == 1)) {
      __SET_VAR(data__->FBHYD_WRITEPARAMETER.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_WRITEPARAMETER.,EXECUTE,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->FBHYD_WRITEPARAMETER.,PARAMETERNUMBER,,25);
      __SET_VAR(data__->FBHYD_WRITEPARAMETER.,VALUE,,4);
      HYD_WRITEPARAMETER_body__(&data__->FBHYD_WRITEPARAMETER);
      if (__GET_VAR(data__->FBHYD_WRITEPARAMETER.DONE,)) {
        __SET_VAR(data__->,ISTEP,,(__GET_VAR(data__->ISTEP,) + 1));
      };
    }
    else if ((__case_expression == 2)) {
      __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ENABLE,,1);
      __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ACT_POSITION,,0);
      __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ACT_VELOCITY,,0);
      __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,ACT_PRESSURE,,__GET_VAR(data__->FACTPRESSURE,));
      __SET_VAR(data__->FBHYD_SETAXISFEEDBACK.,TIMESTAMP,,__GET_VAR(data__->TIMECYCLE,));
      HYD_SETAXISFEEDBACK_body__(&data__->FBHYD_SETAXISFEEDBACK);
      __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,AXISID,,__GET_VAR(data__->IAXIS0,));
      __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,EXECUTE,,__BOOL_LITERAL(TRUE));
      __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,CONTINUOUSUPDATE,,0);
      __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,PRESSURE,,__GET_VAR(data__->FSETPRESSURE,));
      __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,PRESSURERAMPRATE,,20.0);
      __SET_VAR(data__->FBHYD_PRESSUREHANDLE.,DURATION,,0);
      HYD_PRESSUREHANDLE_body__(&data__->FBHYD_PRESSUREHANDLE);
      __SET_VAR(data__->,BINPRESSURE,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.INPRESSURE));
      __SET_VAR(data__->,BDONE,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.DONE));
      __SET_VAR(data__->,BBUSY,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.BUSY));
      __SET_VAR(data__->,BACT,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.ACTIVE));
      __SET_VAR(data__->,BABORTED,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.COMMANDABORTED));
      __SET_VAR(data__->,BERR,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.ERROR));
      __SET_VAR(data__->,WERRID,,__GET_VAR(data__->FBHYD_PRESSUREHANDLE.ERRORID));
    }
  };
  __SET_VAR(data__->FBHYD_GETPUMPREQUEST.,ENABLE,,1);
  HYD_GETPUMPREQUEST_body__(&data__->FBHYD_GETPUMPREQUEST);
  __SET_VAR(data__->,FPIDOUTPUT,,__GET_VAR(data__->FBHYD_GETPUMPREQUEST.PUMPSPEED));
  __SET_VAR(data__->,HYDRAULIC_SYS.MOTORVELOCITY,,__GET_VAR(data__->FPIDOUTPUT,));
  __SET_VAR(data__->,HYDRAULIC_SYS.TCYCLE,,__GET_VAR(data__->TCYCLE,));
  __SET_VAR(data__->,HYDRAULIC_SYS.KNUM,,5.4);
  __SET_VAR(data__->,HYDRAULIC_SYS.TTAU,,1.0);
  __SET_VAR(data__->,HYDRAULIC_SYS.DELAYTIME,,0.0);
  FIRSTORDERSYSTEM_body__(&data__->HYDRAULIC_SYS);

  goto __end;

__end:
  return;
} // TESTPRESSURECTRL_body__() 




