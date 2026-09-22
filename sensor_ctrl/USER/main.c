/* ================ main.c 【整体替换】 ================
 * FreeRTOS 版入口：外设初始化 → 建任务 → 启动调度器
 * ==================================================== */
#include "sys.h"
#include "delay.h"
#include "hardware.h"
#include "app_config.h"
#include "app_state.h"
#include "app_comm.h"
#include "app_ctrl.h"     /* ★新增：三个任务的声明 */
#include "FreeRTOS.h"
#include "task.h"

/* ★【必须核对】mpu6050.h 的初始化函数名 */
extern void MPU6050_Init(void);

static void hw_init_all(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  /* FreeRTOS 要求全抢占分组 */
    delay_init(168);          /* 调度器启动前调用，双模式延时 */
    sys_time_init();          /* DWT 微秒时间戳 */
    debug_usart_init(115200); /* 蓝牙调试 printf */

    Tracking_Init();

    /* ★必须先于 TIM6_init()：TIM6 中断调用 MotorControl_Update()，
       其中用到 PID/电机句柄，未初始化会输出乱码 PWM */
    MotorControl_Init();

    /* ★新增：编码器初始化（encoder.h 确认存在，原文件漏调，
       TIM6 中断里 get_speed() 依赖它） */
    encoder_init();

    TIM6_init();              /* 速度环 100Hz */

    MPU6050_Init();           /* ★含300ms零漂补偿，上电保持静止 */
    HC_SR04_Init();
    servo_bus_init(115200);   /* USART3 → 机械臂 */
    PathPlanner_Init();       /* 加载静态图配置 */
    ADC_Battery_Init();       /* ★新增：ADC1_IN12 电量检测 */

    /* LED PB6 / 按键 PD4 GPIO（蜂鸣器改由 Buzzer_Init 统一初始化） */
    {
        GPIO_InitTypeDef g;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);
        g.GPIO_Mode  = GPIO_Mode_OUT;
        g.GPIO_OType = GPIO_OType_PP;
        g.GPIO_Speed = GPIO_Speed_50MHz;
        g.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        g.GPIO_Pin   = GPIO_Pin_6;
        GPIO_Init(GPIOB, &g);  /* LED */

        g.GPIO_Mode = GPIO_Mode_IN;
        g.GPIO_PuPd = GPIO_PuPd_UP;
        g.GPIO_Pin  = GPIO_Pin_4;
        GPIO_Init(GPIOD, &g);  /* 按键 */
    }

    Buzzer_Init();  /* ★替换：原手写 PC13 配置删除（当时GPIOC时钟未开，
                       配置无效）；Buzzer_Init 内部已含 GPIOC 时钟使能 */
}

int main(void)
{
    hw_init_all();
    app_state_init();
    app_comm_init();  /* 内部完成 usart2/usart6 初始化+回调注册 */

    xTaskCreate(app_ctrl_task,     "ctrl", 512, NULL, 4, NULL);
    xTaskCreate(app_comm_task,     "comm", 512, NULL, 3, NULL);
    xTaskCreate(app_obstacle_task, "obst", 256, NULL, 2, NULL);
    xTaskCreate(app_battery_task,  "batt", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    /* 之后的代码不该被执行到 */
    while (1);
}
