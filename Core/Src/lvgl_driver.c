#include "GT9147_driver.h"
#include "SEGGER_RTT.h"
#include "cmsis_os.h"
#include "ltdc.h"
#include "lvgl.h"
#include "portmacro.h"
#include "stm32_hal_legacy.h"
#include "stm32h750xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_ltdc.h"
#include <src/misc/lv_types.h>
#include <src/tick/lv_tick.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include <sysmem.h>
#include <timers.h>
#include <lvgl_app.h>


static lv_color32_t lcd_frame_buffer_1[272][480] AT_EXSDRAM_ALIGN(64);
static lv_color32_t lcd_frame_buffer_2[272][480] AT_EXSDRAM_ALIGN(64);
static TaskHandle_t lvgl_task_handle;
static void *lvgl_timer_id;
static lv_indev_t *lvgl_input_dev;

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    SCB_CleanDCache_by_Addr((uint32_t *)px_map, sizeof(lcd_frame_buffer_1));
    HAL_LTDC_SetAddress(&hltdc, (uint32_t)px_map, 0);
    // HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);
    lv_display_flush_ready(disp);
}

static void lvgl_log(lv_log_level_t level, const char * buf)
{
    UNUSED(level);
    SEGGER_RTT_WriteString(0, buf);
}

static void lvgl_input_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    GT9147_Point_t point;
    int point_num = gt9147_get_touch(&point, 1);
    if (point_num > 0)
    {
        // 触摸屏坐标系
        //      480 x
        //          ^
        //          |
        // 800 y<---*
        data->point.x = (800 - point.y) * 479.0f / 800.0f;
        data->point.y = (480 - point.x) * 271.0f / 480.0f;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void lvgl_task(void *args)
{
    UNUSED(args);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    lv_init();
    lv_tick_set_cb(xTaskGetTickCount);
    lv_display_t *display = lv_display_create(480, 272);
    lv_display_set_buffers(display, lcd_frame_buffer_1, lcd_frame_buffer_2, sizeof(lcd_frame_buffer_2),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    lvgl_input_dev = lv_indev_create();
    lv_indev_set_type(lvgl_input_dev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lvgl_input_dev, lvgl_input_read);
    lv_log_register_print_cb(lvgl_log);

    lvgl_app_init();

    while (1)
    {
        uint32_t delay_time = lv_timer_handler();
        vTaskDelay(delay_time);
    }
    vTaskDelete(NULL);
}

static void lvgl_timer(TimerHandle_t xTimer)
{
    UNUSED(xTimer);
    lv_tick_inc(10);
}

int lvgl_init()
{
    HAL_GPIO_WritePin(LCD_BK_GPIO_Port, LCD_BK_Pin, GPIO_PIN_SET);
    // __HAL_LTDC_ENABLE_IT(&hltdc, LTDC_IT_LI);
    HAL_LTDC_SetAddress(&hltdc, (uint32_t)lcd_frame_buffer_1, 0);
    HAL_LTDC_SetWindowPosition_NoReload(&hltdc, 0, 0, 0);
    HAL_LTDC_SetWindowSize(&hltdc, 480, 272, 0);

    xTaskCreate(lvgl_task, "lvgl", 8192, NULL, 30, &lvgl_task_handle);
    lvgl_timer_id = xTimerCreate("lvgl_tick", 10, pdTRUE, NULL, lvgl_timer);
    xTimerStart(lvgl_timer_id, 0);
    return 0;
}

void lv_mem_init()
{
}

void *lv_malloc_core(size_t size)
{
    return app_malloc(size);
}

void lv_free_core(void *p)
{
    app_free(p);
}

void *lv_realloc_core(void *p, size_t new_size)
{
    return app_realloc(p, new_size);
}

// #include "cmsis_os.h"
// #include "ltdc.h"
// #include "lvgl.h"
// #include "portmacro.h"
// #include "stm32_hal_legacy.h"
// #include "stm32h750xx.h"
// #include "stm32h7xx_hal.h"
// #include "stm32h7xx_hal_def.h"
// #include "stm32h7xx_hal_ltdc.h"
// #include <src/hal/lv_hal_disp.h>
// #include <stdint.h>
// #include <sys/cdefs.h>
// #include <sysmem.h>
// #include <demos/benchmark/lv_demo_benchmark.h>

// static lv_color32_t lcd_frame_buffer_1[272][480] AT_EXSDRAM_ALIGN(64);
// static lv_color32_t lcd_frame_buffer_2[272][480] AT_EXSDRAM_ALIGN(64);
// static TaskHandle_t lvgl_task_handle;
// static lv_disp_draw_buf_t draw_buf;
// static lv_disp_drv_t disp_drv;        /*Descriptor of a display driver*/

// static void lvgl_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *px_map)
// {
//     SCB_CleanDCache_by_Addr((uint32_t *)px_map, sizeof(lcd_frame_buffer_1));
//     HAL_LTDC_SetAddress(&hltdc, (uint32_t)px_map, 0);
//     // HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);
//     lv_disp_flush_ready(disp);
// }

// static void lvgl_task(void *args)
// {
//     UNUSED(args);
//     TickType_t pxPreviousWakeTime = xTaskGetTickCount();

//     lv_init();

//     lv_disp_draw_buf_init(&draw_buf, lcd_frame_buffer_1, lcd_frame_buffer_2, 272 * 480);  /*Initialize the display
//     buffer.*/

//     lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
//     disp_drv.flush_cb = lvgl_flush_cb;    /*Set your driver function*/
//     disp_drv.draw_buf = &draw_buf;        /*Assign the buffer to the display*/
//     disp_drv.hor_res = 480;   /*Set the horizontal resolution of the display*/
//     disp_drv.ver_res = 272;   /*Set the vertical resolution of the display*/
//     lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

//     lv_demo_benchmark();

//     while (1)
//     {
//         lv_timer_handler();
//         vTaskDelayUntil(&pxPreviousWakeTime, 20);
//     }
//     vTaskDelete(NULL);
// }

// int lvgl_init()
// {
//     HAL_GPIO_WritePin(LCD_BK_GPIO_Port, LCD_BK_Pin, GPIO_PIN_SET);
//     // __HAL_LTDC_ENABLE_IT(&hltdc, LTDC_IT_LI);
//     HAL_LTDC_SetAddress(&hltdc, (uint32_t)lcd_frame_buffer_1, 0);
//     HAL_LTDC_SetWindowPosition_NoReload(&hltdc, 0, 0, 0);
//     HAL_LTDC_SetWindowSize(&hltdc, 480, 272, 0);

//     xTaskCreate(lvgl_task, "lvgl", 8192, NULL, 30, &lvgl_task_handle);
//     return 0;
// }
