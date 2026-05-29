#include "iec_types_all.h"
#include "POUS.h"




    extern int  __HydMotion_framework_Init();
    extern void __HydMotion_framework_Cleanup();
    extern void __HydMotion_framework_Retrieve();
    extern void __HydMotion_framework_Publish();
    


/**
 * 注意：以下四个函数名称不能修改，接口固定
 * 集成开发环境在启用注塑库时，会生成对应的C代码，以调用下面四个函数
 */
int __init_HydMotion()
{
    return __HydMotion_framework_Init();
}

void __cleanup_HydMotion()
{
    __HydMotion_framework_Cleanup();
}

void __retrieve_HydMotion()
{
    __HydMotion_framework_Retrieve();
}

void __publish_HydMotion()
{
    __HydMotion_framework_Publish();
}

