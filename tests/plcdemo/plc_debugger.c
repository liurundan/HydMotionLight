/*
 * DEBUGGER code
 * 
 * On "publish", when buffer is free, debugger stores arbitrary variables 
 * content into, and mark this buffer as filled
 * 
 * 
 * Buffer content is read asynchronously, (from non real time part), 
 * and then buffer marked free again.
 *  
 * 
 * */
#ifdef TARGET_DEBUG_AND_RETAIN_DISABLE
#include <stdint.h>
void __init_debug    (void){}
void __cleanup_debug (void){}
void __retrieve_debug(void){}
void __publish_debug (void){}

void ResetDebugVariables(void){}
void RegisterDebugVariable(int idx, void* force, int32_t array_index_number){}
//
int GetDebugData(unsigned long *tick, unsigned long *size, void **buffer){ return 0; }
void FreeDebugData(void) {}

#else

#if defined(_WIN32) && defined(VOFA_ENABLED)
#include "vofa_server.h"
#endif

#include "iec_types_all.h"
#include "POUS.h"
/*for memcpy*/
#include <string.h>
#include <stdio.h>




typedef unsigned int dbgvardsc_index_t;
typedef unsigned short trace_buf_offset_t;

#define BUFFER_EMPTY 0
#define BUFFER_FULL 1

#ifndef TARGET_ONLINE_DEBUG_DISABLE

#define TRACE_BUFFER_SIZE 4096
#define TRACE_LIST_SIZE 1024

/* Atomically accessed variable for buffer state */
static long trace_buffer_state = BUFFER_EMPTY;

typedef struct trace_item_s {
    dbgvardsc_index_t dbgvardsc_index;
    int array_index_number;
} trace_item_t;

trace_item_t trace_list[TRACE_LIST_SIZE];
char trace_buffer[TRACE_BUFFER_SIZE];

/* Trace's cursor*/
static trace_item_t *trace_list_collect_cursor = trace_list;
static trace_item_t *trace_list_addvar_cursor = trace_list;
static const trace_item_t *trace_list_end = 
    &trace_list[TRACE_LIST_SIZE-1];
static char *trace_buffer_cursor = trace_buffer;
static const char *trace_buffer_end = trace_buffer + TRACE_BUFFER_SIZE;



#define FORCE_BUFFER_SIZE 1024
#define FORCE_LIST_SIZE 256

typedef struct force_item_s {
    dbgvardsc_index_t dbgvardsc_index;
    void *value_pointer_backup;
    int array_index_number;
} force_item_t;

force_item_t force_list[FORCE_LIST_SIZE];
char force_buffer[FORCE_BUFFER_SIZE];

/* Force's cursor*/
static force_item_t *force_list_apply_cursor = force_list;
static force_item_t *force_list_addvar_cursor = force_list;
static const force_item_t *force_list_end =  &force_list[FORCE_LIST_SIZE-1];
static char *force_buffer_cursor = force_buffer;
static const char *force_buffer_end = force_buffer + FORCE_BUFFER_SIZE;


#endif

/***
 * Declare programs 
 **/
extern TESTPRESSURECTRL RESOURCE1__INSTANCE0;

/***
 * Declare global variables from resources and conf 
 **/
extern __IEC_WORD_t CONFIG__GVLG0_CAN_AXIS_NUM;
extern __IEC_WORD_t CONFIG__GVLG0_LOCAL_AXIS_NUM;
extern __IEC_UINT_t CONFIG__GVLG0_BOARD_ID;
extern __IEC_UDINT_t CONFIG__GVLG0_KERNEL_VERSION;
extern __IEC_WORD_t CONFIG__GGRP_MBUS_COMMAD;
extern __IEC_WORD_t CONFIG__GGRP_MBUS_ACTPOS;
extern __IEC_WORD_t CONFIG__GGRP_MBUS_ACTVEL;
extern       TESTPRESSURECTRL   RESOURCE1__INSTANCE0;

#define MAX_ARRAY_DIMENSIONS 4  // Maximum supported dimension count
typedef struct {
    void* ptr;
    __IEC_types_enum type;
} dbgvardsc_t;

typedef const struct {
    int index;
    int size;
} dbgvardsc_struct_index_t;

static dbgvardsc_t dbgvardsc[] = {
{&(CONFIG__GVLG0_CAN_AXIS_NUM), WORD_ENUM},
{&(CONFIG__GVLG0_LOCAL_AXIS_NUM), WORD_ENUM},
{&(CONFIG__GVLG0_BOARD_ID), UINT_ENUM},
};

static const dbgvardsc_index_t retain_list[] = {

};
static const dbgvardsc_struct_index_t retain_address_index[] = {

};
static unsigned int retain_list_collect_cursor = 0;
static const unsigned int retain_list_size = sizeof(retain_list)/sizeof(dbgvardsc_index_t);

typedef void(*__for_each_variable_do_fp)(dbgvardsc_t*);
void __for_each_variable_do(__for_each_variable_do_fp fp)
{
    unsigned int i;
    for(i = 0; i < sizeof(dbgvardsc)/sizeof(dbgvardsc_t); i++){
        dbgvardsc_t *dsc = &dbgvardsc[i];
        if(dsc->type != UNKNOWN_ENUM) 
            (*fp)(dsc);
    }
}

#define __Unpack_desc_type dbgvardsc_t


void toLogMessage(const char* format, ...) {
    char mstr2[256] = { 0 };
    va_list args;
    va_start(args, format);
    vsnprintf(mstr2, sizeof(mstr2), format, args);
    va_end(args);
    LogMessage(LOG_WARNING, mstr2, strlen(mstr2));
}


#define __Unpack_Variable_case_t(TYPENAME)                                               \
         case TYPENAME##_ENUM:                                                           \
              if (array_index_number != -1) {                      \
                 TYPENAME* array = ((TYPENAME*)varp);            \
                 varp = &array[array_index_number];                                            \
                 if (value_p) *value_p = varp;          \
                 if (size) *size = sizeof(TYPENAME);                                     \
             } else {                                                                    \
                 if (flags) *flags = ((__IEC_##TYPENAME##_t*)varp)->flags;               \
                 if (value_p) *value_p = &((__IEC_##TYPENAME##_t*)varp)->value;         \
                 if (size) *size = sizeof(TYPENAME);                                    \
             }                                                                          \
             break;

#define __Unpack_Variable_case_p(TYPENAME)                                               \
         case TYPENAME##_O_ENUM:                                                           \
         case TYPENAME##_P_ENUM:                                                           \
             if (array_index_number != -1) {                         \
                 TYPENAME* array = ((__IEC_##TYPENAME##_p*)varp)->value;            \
                 varp = &array[array_index_number];                                            \
                 if (value_p) *value_p = varp;          \
                 if (size) *size = sizeof(TYPENAME);                                     \
             } else {                                                                    \
                 if(flags) *flags = ((__IEC_##TYPENAME##_p *)varp)->flags;               \
                 if(value_p) *value_p = ((__IEC_##TYPENAME##_p *)varp)->value;         \
                 if(size) *size = sizeof(TYPENAME);                                    \
             }                                                                          \
             break;


#define __Unpack_case_f(TYPENAME)                                           \
        case TYPENAME##_F_ENUM :                                            \
            if(value_p) *value_p = ((IEC_##TYPENAME *)varp);                \
		    if(size) *size = sizeof(TYPENAME);                              \
            break;

#define __Unpack_case_l(TYPENAME)                                           \
        case TYPENAME##_L_ENUM :                                            \
			if(flags) *flags = ((__IEC_##TYPENAME##_p *)varp)->flags;       \
			if(value_p) *value_p = ((__IEC_##TYPENAME##_p *)varp)->value;   \
			if(size) *size = sizeof(TYPENAME);                              \
			break;

#define __Is_a_string(dsc) (dsc->type == STRING_ENUM)   ||\
						   (dsc->type == STRING_F_ENUM) ||\
                           (dsc->type == STRING_P_ENUM) ||\
                           (dsc->type == STRING_O_ENUM)


static int UnpackVarWithIndex(__Unpack_desc_type* dsc, void** value_p, char* flags, size_t* size, int array_index_number){
    void* varp = dsc->ptr;
    switch (dsc->type) {
        __ANY(__Unpack_Variable_case_t)
        __ANY(__Unpack_Variable_case_p)
        __ANY(__Unpack_case_f)
        __ANY(__Unpack_case_l)
    default:
        return 0; /* should never happen */
    }
    return 1;
}







void Remind(unsigned int offset, unsigned int count, void * p);

extern int CheckRetainBuffer(void);
extern int InitRetain(void);
static int retain_max_size = 0;

#if defined(_WIN32) && defined(VOFA_ENABLED)
    static float vofa_data[VOFA_MAX_CHANNELS]; // Assuming all variables can be represented as float for VOFA
    static int vofa_index = 0;
#endif

void __init_debug(void)
{
    //init_debug_variables();
    /* init local static vars */
#if defined(_WIN32) && defined(VOFA_ENABLED)
    if( vofa_init(VOFA_DEFAULT_PORT) == 0) {
		char mstr[] = "Failed to initialize VOFA server on port 1346!";
		LogMessage(LOG_CRITICAL, mstr, sizeof(mstr));
	}
    for( int i=0; i < VOFA_MAX_CHANNELS; ++i) {
    	vofa_data[i] = 0.0f; // Initialize with default values
    }
#endif

#ifndef TARGET_ONLINE_DEBUG_DISABLE
    trace_buffer_cursor = trace_buffer;
    trace_list_addvar_cursor = trace_list;
    trace_list_collect_cursor = trace_list;
    trace_buffer_state = BUFFER_EMPTY;

    force_buffer_cursor = force_buffer;
    force_list_addvar_cursor = force_list;
    force_list_apply_cursor = force_list;
#endif

    retain_max_size = InitRetain();
    /* Iterate over all variables to fill debug buffer */
    if(CheckRetainBuffer()){
        int retain_offset = 0;
        retain_list_collect_cursor = 0;

        /* iterate over retain list */
        while(retain_list_collect_cursor < retain_list_size){
            void *value_p = NULL;
            size_t size;
            char* next_cursor;

            dbgvardsc_t *dsc = &dbgvardsc[
                retain_list[retain_list_collect_cursor]];

            UnpackVarWithIndex(dsc, &value_p, NULL, &size,-1);

#if defined(TARGET_TYPE_Gm2xx)
            dbgvardsc_struct_index_t* retain_addr =
				&retain_address_index[retain_list_collect_cursor];
            retain_offset = retain_addr->index;
            size = retain_addr->size;
            if (retain_offset + size >= retain_max_size) {
				char mstr[] = "RETAIN variable memory invalid!";
				LogMessage(LOG_CRITICAL, mstr, sizeof(mstr));
				retain_list_collect_cursor++;
				continue;
			}
            if (retain_offset < 0) {
				char mstr[] = "RETAIN variable address not specified!";
				LogMessage(LOG_WARNING, mstr, sizeof(mstr));
				retain_list_collect_cursor++;
				continue;
			}
            /* if buffer not full */
            Remind(retain_offset, size, value_p);
#else
            /* if buffer not full */
            Remind(retain_offset, size, value_p);
            /* increment cursor according size*/
            retain_offset += size;
#endif

            retain_list_collect_cursor++;
        }
    }else{
        char mstr[] = "RETAIN memory invalid - defaults used";
        LogMessage(LOG_WARNING, mstr, sizeof(mstr));
    }
}

extern void InitiateDebugTransfer(void);
extern void CleanupRetain(void);

extern unsigned int __tick;

void __cleanup_debug(void)
{
#if defined(_WIN32) && defined(VOFA_ENABLED)
    vofa_cleanup();
#endif

#ifndef TARGET_ONLINE_DEBUG_DISABLE
    trace_buffer_cursor = trace_buffer;
    InitiateDebugTransfer();
#endif    

#if !defined(TARGET_TYPE_Gm2xx)
    CleanupRetain();
#endif

}

void __retrieve_debug(void)
{
}

void Retain(unsigned int offset, unsigned int count, void * p);

/* Return size of all retain variables */
unsigned int GetRetainSize(void)
{
    unsigned int retain_size = 0;
    retain_list_collect_cursor = 0;

    /* iterate over retain list */
    while(retain_list_collect_cursor < retain_list_size){
        void *value_p = NULL;
        size_t size;
        char* next_cursor;

        dbgvardsc_t *dsc = &dbgvardsc[
            retain_list[retain_list_collect_cursor]];

        UnpackVarWithIndex(dsc, &value_p, NULL, &size, -1);

        retain_size += size;
        retain_list_collect_cursor++;
    }

    return retain_size;
}


extern void PLC_GetTime(IEC_TIME*);
extern int TryEnterDebugSection(void);
extern long AtomicCompareExchange(long*, long, long);
extern long long AtomicCompareExchange64(long long* , long long , long long);
extern void LeaveDebugSection(void);
extern void ValidateRetainBuffer(void);
extern void InValidateRetainBuffer(void);
//fixme: liurundan force variable map Modbus input register address jump to 0
#define __ReForceOutput_case_p(TYPENAME)                                                       \
        case TYPENAME##_P_ENUM:                                                                    \
        case TYPENAME##_O_ENUM:                                                                    \
        {                                                                                       \
            if (force_list_apply_cursor->array_index_number >= 0) {\
                /*xuweiren add - Processing the array part*/                                                             \
                char* next_cursor = force_buffer_cursor + sizeof(TYPENAME);                   \
                TYPENAME* array = ((__IEC_##TYPENAME##_p*)varp)->value;                       \
                array[force_list_apply_cursor->array_index_number] = *((TYPENAME*)force_buffer_cursor); \
                force_buffer_cursor = next_cursor;                                                             \
            }else {\
                char* next_cursor = force_buffer_cursor + sizeof(TYPENAME);                     \
                if (next_cursor <= force_buffer_end) {\
                    if (vartype == TYPENAME##_O_ENUM)                                           \
                        * ((TYPENAME*)force_list_apply_cursor->value_pointer_backup) = *((TYPENAME*)force_buffer_cursor);\
                    force_buffer_cursor = next_cursor;                                          \
                }else {\
                    stop = 1;                                                                   \
                }                                                                               \
            }                                                                                   \
        }                                                                                       \
            break;

#if defined(_WIN32) && defined(VOFA_ENABLED)
void convert_and_store(float *vofa_data, int index, const void *value_p,
		__IEC_types_enum type, size_t size) {
	if (vofa_data == NULL || value_p == NULL)
		return;
	switch (size) {
	case 1: // BOOL, SINT, USINT, BYTE
		vofa_data[index] = (float) (*((uint8_t*) value_p));
		break;
	case 2: // INT, UINT, WORD
		vofa_data[index] = (float) (*((uint16_t*) value_p));
		break;
	case 4: // DINT, UDINT, DWORD, REAL
		if (type == REAL_ENUM || type == REAL_O_ENUM || type == REAL_P_ENUM) {
			vofa_data[index] = *((float*) value_p);
		} else {
			vofa_data[index] = (float) (*((uint32_t*) value_p));
		}
		break;
	case 8: // LINT, ULINT, LWORD, TIME, LREAL
		if (type == LREAL_ENUM || type == LREAL_O_ENUM
				|| type == LREAL_P_ENUM) {
			vofa_data[index] = (float) (*((double*) value_p));
		} else {
			vofa_data[index] = (float) (*((uint64_t*) value_p));
		}
		break;
	default:
		break;
	}
}
#endif

void __publish_debug(void)
{
    InValidateRetainBuffer();
    
#ifndef TARGET_ONLINE_DEBUG_DISABLE 
    /* Check there is no running debugger re-configuration */
    if(TryEnterDebugSection()){
        /* Lock buffer */
        long latest_state = AtomicCompareExchange(
            &trace_buffer_state,
            BUFFER_EMPTY,
            BUFFER_FULL);
            
        /* If buffer was free */
        if(latest_state == BUFFER_EMPTY)
        {
            int stop = 0;
            /* Reset force list cursor */
            force_list_apply_cursor = force_list;

            /* iterate over force list */
            while(!stop && force_list_apply_cursor < force_list_addvar_cursor){
                dbgvardsc_t *dsc = &dbgvardsc[force_list_apply_cursor->dbgvardsc_index];
                void *varp = dsc->ptr;
                __IEC_types_enum vartype = dsc->type;
                switch(vartype){
                    __ANY(__ReForceOutput_case_p)
                default:
                    break;
                }
                force_list_apply_cursor++;
            }

            /* Reset buffer cursor */
            trace_buffer_cursor = trace_buffer;
            /* Reset trace list cursor */
            trace_list_collect_cursor = trace_list;

#if defined(_WIN32) && defined(VOFA_ENABLED)
           vofa_index = 0;
#endif

            /* iterate over trace list */
            while(trace_list_collect_cursor < trace_list_addvar_cursor){
                void *value_p = NULL;
                size_t size;
                char* next_cursor;
                dbgvardsc_t *dsc = &dbgvardsc[trace_list_collect_cursor->dbgvardsc_index];
                //
                int array_index_number = trace_list_collect_cursor->array_index_number;
                UnpackVarWithIndex(dsc, &value_p, NULL, &size, array_index_number);
                //
                /* copy visible variable to buffer */;
                if(__Is_a_string(dsc)){
                    /* optimization for strings */
                    /* assume NULL terminated strings */
                    size = ((STRING*)value_p)->len + 1;
                }

                /* compute next cursor positon.*/
                next_cursor = trace_buffer_cursor + size;
                /* check for buffer overflow */
                if(next_cursor < trace_buffer_end)
                    /* copy data to the buffer */
                    memcpy(trace_buffer_cursor, value_p, size);
                else
                    /* stop looping in case of overflow */
                    break;
                /* increment cursor according size*/
                trace_buffer_cursor = next_cursor;
                trace_list_collect_cursor++;

				#if defined(_WIN32) && defined(VOFA_ENABLED)
					if(vofa_index < VOFA_MAX_CHANNELS && !__Is_a_string(dsc) ) {
						convert_and_store(vofa_data, vofa_index, value_p, dsc->type, size);
						vofa_index++;
					}

				#endif

            }

#if defined(_WIN32) && defined(VOFA_ENABLED)
		vofa_push_data(vofa_data, VOFA_MAX_CHANNELS); // Push data to VOFA server
#endif
            /* Leave debug section,
             * Trigger asynchronous transmission 
             * (returns immediately) */
            InitiateDebugTransfer(); /* size */
        }
        force_buffer_cursor = force_buffer; // reset pointer by liurundan
        LeaveDebugSection();
    }
#endif
    int retain_offset = 0;
    /* when not debugging, do only retain */
    retain_list_collect_cursor = 0;

    /* iterate over retain list */
    while(retain_list_collect_cursor < retain_list_size){
        void *value_p = NULL;
        size_t size;
        char* next_cursor;
        dbgvardsc_t *dsc = &dbgvardsc[
            retain_list[retain_list_collect_cursor]];

        UnpackVarWithIndex(dsc, &value_p, NULL, &size, -1);
#if defined(TARGET_TYPE_Gm2xx)
        dbgvardsc_struct_index_t* retain_addr =
			&retain_address_index[retain_list_collect_cursor];
        retain_offset = retain_addr->index;
        size = retain_addr->size;
        if (retain_offset + size >= retain_max_size) {
        	retain_list_collect_cursor++;
			continue;
		}
        if (retain_offset < 0) {
        	retain_list_collect_cursor++;
			continue;
		}
        /* if buffer not full */
        Retain(retain_offset, size, value_p);
#else
        /* if buffer not full */
        Retain(retain_offset, size, value_p);
        /* increment cursor according size*/
        retain_offset += size;
#endif
        retain_list_collect_cursor++;
    }
    ValidateRetainBuffer();
}

#ifndef TARGET_ONLINE_DEBUG_DISABLE

#define TRACE_LIST_OVERFLOW    1
#define FORCE_LIST_OVERFLOW    2
#define FORCE_BUFFER_OVERFLOW  3
#define FORCE_INVALID  4

#define __ForceVariable_checksize(TYPENAME)                                             \
    if(sizeof(TYPENAME) != force_size) {                                                \
        error_code = FORCE_BUFFER_OVERFLOW;                                             \
        goto error_cleanup;                                                             \
    }

#define __ForceVariable_case_t(TYPENAME)                                                \
        case TYPENAME##_ENUM :                                                          \
            __ForceVariable_checksize(TYPENAME)                                         \
            /* add to force_list*/                                                      \
            force_list_addvar_cursor->dbgvardsc_index = idx;                             \
            force_list_addvar_cursor->array_index_number = array_index_number;  \
            if(array_index_number >= 0){                                                \
                /* xuweiren add - Processing the array part*/                           \
                TYPENAME* array = ((TYPENAME*)varp);           \
                TYPENAME* element = &array[array_index_number];             \
                force_list_addvar_cursor->value_pointer_backup = &(*element);\
                *element = *((TYPENAME*)force);                              \
                /*toLogMessage("__ForceVariable_case_t   idx=%d arr-idx=%d type=%d  element=%d force=%d",*/\
                /*idx,array_index_number, dsc->type, *((TYPENAME*)element), *((TYPENAME *)force));*/\
            }else{                                                                     \
                ((__IEC_##TYPENAME##_t*)varp)->flags |= __IEC_FORCE_FLAG;               \
                ((__IEC_##TYPENAME##_t*)varp)->value = *((TYPENAME*)force);             \
            }                                                                           \
            break;

#define __ForceVariable_case_p(TYPENAME)                                                \
        case TYPENAME##_P_ENUM :                                                        \
        case TYPENAME##_O_ENUM :                                                        \
		case TYPENAME##_L_ENUM :                                                        \
            __ForceVariable_checksize(TYPENAME)                                         \
            {                                                                           \
                char *next_cursor = force_buffer_cursor + sizeof(TYPENAME);             \
                if(next_cursor <= force_buffer_end ){                                   \
                    /* add to force_list*/                                              \
                    force_list_addvar_cursor->dbgvardsc_index = idx;                    \
                    force_list_addvar_cursor->array_index_number = array_index_number;  \
                    /* save pointer to backup */                                        \
                    force_list_addvar_cursor->value_pointer_backup =                    \
                        ((__IEC_##TYPENAME##_p *)varp)->value;                          \
                    /* store forced value in force_buffer */                            \
					*((TYPENAME *)force_buffer_cursor) = *((TYPENAME *)force);          \
                    /* replace pointer with pointer to force_buffer */                  \
                    ((__IEC_##TYPENAME##_p *)varp)->value =                             \
                        (TYPENAME *)force_buffer_cursor;                                \
                    /* mark variable as forced */                                       \
                    ((__IEC_##TYPENAME##_p *)varp)->flags |= __IEC_FORCE_FLAG;          \
                    /* inc force_buffer cursor */                                       \
                    force_buffer_cursor = next_cursor;                                  \
                    /* outputs real value must be systematically forced */              \
                    if(vartype == TYPENAME##_O_ENUM) {                                   \
                       *(((__IEC_##TYPENAME##_p *)varp)->value) = *((TYPENAME *)force);  \
                    }\
                } else {                                                                \
                    error_code = FORCE_BUFFER_OVERFLOW;                                 \
                    goto error_cleanup;                                                 \
                }                                                                       \
            }                                                                           \
            break;
#define __ForceVariable_case_f(TYPENAME)                                                \
        case TYPENAME##_F_ENUM :                                                        \
            __ForceVariable_checksize(TYPENAME)                                         \
            /* add to force_list*/                                                      \
            force_list_addvar_cursor->dbgvardsc_index = idx;                            \
            force_list_addvar_cursor->array_index_number = array_index_number;  \
            *((IEC_##TYPENAME *)varp) = *((TYPENAME *)force);                           \
            break;

#define __ForceVariable_case_p1(TYPENAME,IS_REAL)                                                \
        case TYPENAME##_P_ENUM :                                                        \
        case TYPENAME##_O_ENUM :                                                        \
		case TYPENAME##_L_ENUM :                                                        \
		__ForceVariable_checksize(TYPENAME)                                         \
		{                                                                           \
        	char* aligned_ptr = force_buffer_cursor; \
        	char* next_cursor; \
			if (next_cursor <= force_buffer_end) { \
                force_list_addvar_cursor->dbgvardsc_index = idx;\
                force_list_addvar_cursor->array_index_number = array_index_number;\
                if(array_index_number >= 0){                                            \
                    /* xuweiren add - Processing the array part*/                                        \
                    TYPENAME* array = ((__IEC_##TYPENAME##_p *)varp)->value;                                \
                    force_list_addvar_cursor->value_pointer_backup = &array[array_index_number];         \
                    if (IS_REAL) {\
                        size_t align_mask = sizeof(TYPENAME) - 1;\
                        aligned_ptr = (char*)(((uintptr_t)force_buffer_cursor + align_mask) & ~align_mask);\
                    }\
                    next_cursor = aligned_ptr + sizeof(TYPENAME);\
                    *((TYPENAME*)aligned_ptr) = *((TYPENAME*)force); \
                    array[array_index_number] = *((TYPENAME*)aligned_ptr); \
                     /*toLogMessage("__ForceVariable_case_p1   idx=%d arr-idx=%d type=%d  element=%d force=%d",*/ \
                     /*idx, array_index_number, dsc->type, *((TYPENAME*)aligned_ptr), *((TYPENAME*)force));*/\
                } else {                                                                \
                    force_list_addvar_cursor->value_pointer_backup = ((__IEC_##TYPENAME##_p*)varp)->value; \
                    if (IS_REAL) {\
                        size_t align_mask = sizeof(TYPENAME) - 1; \
                        aligned_ptr = (char*)(((uintptr_t)force_buffer_cursor + align_mask) & ~align_mask); \
                        next_cursor = aligned_ptr + sizeof(TYPENAME); \
                    }else {\
                        next_cursor = force_buffer_cursor + sizeof(TYPENAME); \
                    } \
				    *((TYPENAME *)aligned_ptr) = *((TYPENAME *)force); \
				    ((__IEC_##TYPENAME##_p *)varp)->value = (TYPENAME *)aligned_ptr; \
				    ((__IEC_##TYPENAME##_p *)varp)->flags |= 0x02; \
				    if (vartype == TYPENAME##_O_ENUM) { \
				    	*(((__IEC_##TYPENAME##_p *)varp)->value) = *((TYPENAME *)force); \
				    } \
                }                                                                   \
                force_buffer_cursor = next_cursor;                                  \
			} else {                                                                \
				error_code = FORCE_BUFFER_OVERFLOW;                                 \
				goto error_cleanup;                                                 \
			}                                                                       \
		}                                                                           \
		break;

void ResetDebugVariables(void);

int RegisterDebugVariable(dbgvardsc_index_t idx, void* force, size_t force_size, int array_index_number)
{
    int error_code = 0;
    if(idx < sizeof(dbgvardsc)/sizeof(dbgvardsc_t)){
        /* add to trace_list, inc trace_list_addvar_cursor*/
        if(trace_list_addvar_cursor <= trace_list_end)
        {   
            trace_list_addvar_cursor->array_index_number = array_index_number;
            trace_list_addvar_cursor->dbgvardsc_index = idx;
            trace_list_addvar_cursor++;
        } else {
            error_code = TRACE_LIST_OVERFLOW;
            goto error_cleanup;
        }
        if(force){
            if(force_list_addvar_cursor <= force_list_end){
                dbgvardsc_t *dsc = &dbgvardsc[idx];
                void *varp = dsc->ptr;
                __IEC_types_enum vartype = dsc->type;
                char is_real = 0;
                if(vartype == REAL_P_ENUM || vartype == REAL_O_ENUM || vartype == LREAL_P_ENUM || vartype == LREAL_O_ENUM){
					is_real = 1;
				}
                switch(vartype){
                    __ANY(__ForceVariable_case_t)
				   //__ANY(__ForceVariable_case_p)
					__ANY_1(__ForceVariable_case_p1, is_real)
					__ANY(__ForceVariable_case_f)
                default:
                    break;
                }
                /* inc force_list cursor */
                force_list_addvar_cursor++;
            } else {
                error_code = FORCE_LIST_OVERFLOW;
                goto error_cleanup;
            }
        }
    }
    return 0;

error_cleanup:
    ResetDebugVariables();
    trace_buffer_state = BUFFER_EMPTY;
    return error_code;
    
}

#define ResetForcedVariable_case_t(TYPENAME)                                            \
        case TYPENAME##_ENUM :                                                          \
            if(array_index_number >= 0){                                                  \
                /* xuweiren add - Processing the array part*/                           \
                TYPENAME* array = (TYPENAME*)varp;                            \
                TYPENAME* element = &array[array_index_number];                           \
                *element = *((TYPENAME*)force_list_apply_cursor->value_pointer_backup);\
            }                                                                             \
            else {                                                                        \
                ((__IEC_##TYPENAME##_t*)varp)->flags &= ~__IEC_FORCE_FLAG;                 \
            }                                                                           \
            /* for local variable we don't restore original value */                    \
            /* that can be added if needed, but it was like that since ever */          \
            break;

#define ResetForcedVariable_case_p(TYPENAME)                                            \
        case TYPENAME##_P_ENUM :                                                        \
        case TYPENAME##_O_ENUM :                                                        \
    	case TYPENAME##_L_ENUM :                                                        \
            if(array_index_number >= 0){                                                  \
                /* xuweiren add - Processing the array part*/                           \
                TYPENAME* array = ((__IEC_##TYPENAME##_p *)varp)->value;                  \
                TYPENAME* element = &array[array_index_number];                           \
                *element = *((TYPENAME*)force_list_apply_cursor->value_pointer_backup);    \
            }else {                                                                        \
                ((__IEC_##TYPENAME##_p *)varp)->flags &= ~__IEC_FORCE_FLAG;                 \
                /* restore backup to pointer */                                             \
               ((__IEC_##TYPENAME##_p *)varp)->value = force_list_apply_cursor->value_pointer_backup;\
            }                                                                           \
            break;

#define ResetForcedVariable_case_f(TYPENAME)                                            \
        case TYPENAME##_F_ENUM :                                                        \
            /* for local variable we don't restore original value */                    \
            /* that can be added if needed, but it was like that since ever */          \
            break;

void ResetDebugVariables(void)
{
    /* Reset trace list */
    trace_list_addvar_cursor = trace_list;

    force_list_apply_cursor = force_list;
    /* Restore forced variables */
    while(force_list_apply_cursor < force_list_addvar_cursor){
        dbgvardsc_t *dsc = &dbgvardsc[
            force_list_apply_cursor->dbgvardsc_index];
        void *varp = dsc->ptr;
        int array_index_number = force_list_apply_cursor->array_index_number;
        switch(dsc->type){
            __ANY(ResetForcedVariable_case_t)
            __ANY(ResetForcedVariable_case_p)
			__ANY(ResetForcedVariable_case_f)
        default:
            break;
        }
        /* inc force_list cursor */
        force_list_apply_cursor++;
    } /* else TODO: warn user about failure to force */ 

    /* Reset force list */
    force_list_addvar_cursor = force_list;
    /* Reset force buffer */
    force_buffer_cursor = force_buffer;
}

void FreeDebugData(void)
{
    /* atomically mark buffer as free */
    AtomicCompareExchange(
        &trace_buffer_state,
        BUFFER_FULL,
        BUFFER_EMPTY);
}
int WaitDebugData(unsigned int *tick);
/* Wait until debug data ready and return pointer to it */
int GetDebugData(unsigned int *tick, unsigned int *size, void **buffer){
    int wait_error = WaitDebugData(tick);
    if(!wait_error){
        *size = trace_buffer_cursor - trace_buffer;
        *buffer = trace_buffer;
    }
    return wait_error;
}
#endif
#endif

