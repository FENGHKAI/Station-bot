/* ================ app_state.c ================ */
#include "app_state.h"
#include "app_config.h"
#include "FreeRTOS.h"
#include "task.h"

static AppState_t m_state = APP_STANDBY;
static TickType_t m_enter_tick = 0;

void app_state_init(void) { m_state = APP_STANDBY; m_enter_tick = xTaskGetTickCount(); }

void app_enter(AppState_t s) { m_state = s; m_enter_tick = xTaskGetTickCount(); }

AppState_t app_state(void) { return m_state; }

uint32_t app_state_elapsed_ms(void)
{
    return (uint32_t)((xTaskGetTickCount() - m_enter_tick) * portTICK_PERIOD_MS);
}

void app_force_standby(void) { app_enter(APP_STANDBY); }
