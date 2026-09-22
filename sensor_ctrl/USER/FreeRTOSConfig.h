#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ============================================================
 * 针对平台：STM32F407 (Cortex-M4F, 168MHz, Keil/ARMCC)
 * 配套文件：FreeRTOS/Source/{tasks,list,queue,timers,
 *           event_groups,stream_buffer}.c
 *           portable/RVDS/ARM_CM4F/port.c
 *           portable/MemMang/heap_4.c
 * ============================================================ */

/******************************************************************************/
/* 硬件相关（F4 实际值）                                                       */
/******************************************************************************/

#define configCPU_CLOCK_HZ              ( 168000000UL )   /* 主频168M，须与delay_init(168)一致 */
#define configTICK_RATE_HZ              ( ( TickType_t ) 1000 )  /* 1ms一拍，对接delay_ms语义 */

/* F4 的 SysTick 与内核同频，无需 configSYSTICK_CLOCK_HZ */

/******************************************************************************/
/* 调度行为                                                                    */
/******************************************************************************/

#define configUSE_PREEMPTION                    1        /* 抢占式调度 */
#define configUSE_TIME_SLICING                  1        /* 同优先级轮流跑 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1        /* M4支持CLZ指令，开优化 */
#define configUSE_TICKLESS_IDLE                 0        /* 不用低功耗模式 */
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 )  /* 字，idle栈=512B */
#define configMAX_TASK_NAME_LEN                 16
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS  /* M4必须32位 */
#define configIDLE_SHOULD_YIELD                 1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1

/******************************************************************************/
/* 内存管理                                                                    */
/******************************************************************************/

#define configSUPPORT_STATIC_ALLOCATION         0        /* 全用动态创建，简单可靠 */
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ( 32 * 1024 )  /* 20K；开了timers会多占一点，不够就加 */
#define configAPPLICATION_ALLOCATED_HEAP        0
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP 0
#define configHEAP_CLEAR_MEMORY_ON_FREE         1

/******************************************************************************/
/* 功能开关（全部打开，需要时直接用，不用回来改配置）                            */
/******************************************************************************/

/* ---- 软件定时器 ---- */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )  /* 定时器服务任务优先级=6 */
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 ) /* 256字=1KB，回调里别干重活 */
#define configTIMER_QUEUE_LENGTH                10

/* ---- 事件组 ---- */
#define configUSE_EVENT_GROUPS                  1        /* 多条件同步：如"串口收到+对准完成"同时满足 */

/* ---- 流缓冲（字节流单向传输，天生适合串口） ---- */
#define configUSE_STREAM_BUFFERS                1
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t   /* 消息缓冲是流缓冲的定长版，一并可用 */

/* ---- 其他 ---- */
#define configUSE_CO_ROUTINES                   0        /* 协程已废弃，不开 */
#define configMAX_CO_ROUTINE_PRIORITIES         2
#define configUSE_MUTEXES                       1        /* 多任务共用串口时上互斥锁 */
#define configUSE_RECURSIVE_MUTEXES             1        /* 递归锁，同任务可重复上锁 */
#define configUSE_COUNTING_SEMAPHORES           1        /* 中断→任务同步常用 */
#define configUSE_QUEUE_SETS                    0        /* 队列集，用得上再说 */
#define configUSE_TASK_NOTIFICATIONS            1        /* 最轻量的同步方式 */
#define configUSE_APPLICATION_TASK_TAG          0

/******************************************************************************/
/* 中断嵌套配置（F4命门，勿动）                                                 */
/* 规矩：要在中断里调 FromISR API 的，NVIC抢占优先级数值必须 >= 5                 */
/******************************************************************************/

#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS                     __NVIC_PRIO_BITS   /* F4=4 */
#else
    #define configPRIO_BITS                     4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15   /* 最低 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5    /* 分界线 */

#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )  /* 0xF0 */

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) ) /* 0x50 */

/******************************************************************************/
/* 中断向量接管（不用改启动文件，链接时自动覆盖弱定义）                           */
/******************************************************************************/

#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/******************************************************************************/
/* 钩子                                                                        */
/******************************************************************************/

#define configUSE_IDLE_HOOK                    0
#define configUSE_TICK_HOOK                    1   /* 必须开：delay.c的systick_ms靠它累加 */
#define configUSE_MALLOC_FAILED_HOOK           1   /* 堆耗尽时卡死可见，别静默 */
#define configUSE_DAEMON_TASK_STARTUP_HOOK     0

/* configCHECK_FOR_STACK_OVERFLOW=2 需要提供溢出回调，见文件末尾说明 */
#define configCHECK_FOR_STACK_OVERFLOW         2

/******************************************************************************/
/* 调试辅助                                                                    */
/******************************************************************************/

/* 断言：抓配置错误/非法API调用的救命绳，调试期绝不能关 */
#define configASSERT( x )         \
    if( ( x ) == 0 )              \
    {                             \
        taskDISABLE_INTERRUPTS(); \
        for( ; ; )                \
        ;                         \
    }

/******************************************************************************/
/* 统计功能（默认关，要用再开）                                                 */
/******************************************************************************/

#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configQUEUE_REGISTRY_SIZE               0

/******************************************************************************/
/* INCLUDE 开关：不用的API不编进去，省空间                                     */
/******************************************************************************/

#define INCLUDE_vTaskPrioritySet               0
#define INCLUDE_uxTaskPriorityGet              0
#define INCLUDE_vTaskDelete                    1
#define INCLUDE_vTaskSuspend                   1
#define INCLUDE_xResumeFromISR                 0
#define INCLUDE_vTaskDelayUntil                1   /* 周期性任务用这个比vTaskDelay准 */
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_xTaskGetSchedulerState         1
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_uxTaskGetStackHighWaterMark    1   /* 调栈大小用，稳定后可关 */
#define INCLUDE_xTaskGetIdleTaskHandle         0
#define INCLUDE_eTaskGetState                  0
#define INCLUDE_xEventGroupSetBitFromISR       1   /* 开了事件组，这个ISR版一起备好 */
#define INCLUDE_xTimerPendFunctionCall         1   /* 开了定时器，ISR里转调函数用得上 */
#define INCLUDE_xTaskAbortDelay                0
#define INCLUDE_xTaskGetHandle                 0
#define INCLUDE_xTaskResumeFromISR             0

#endif /* FREERTOS_CONFIG_H */

