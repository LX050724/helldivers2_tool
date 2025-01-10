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
#include <demos/benchmark/lv_demo_benchmark.h>

static lv_color32_t lcd_frame_buffer_1[272][480] AT_EXSDRAM_ALIGN(64);
static lv_color32_t lcd_frame_buffer_2[272][480] AT_EXSDRAM_ALIGN(64);
static TaskHandle_t lvgl_task_handle;

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    SCB_CleanDCache_by_Addr((uint32_t *)px_map, sizeof(lcd_frame_buffer_1));
    HAL_LTDC_SetAddress(&hltdc, (uint32_t)px_map, 0);
    // HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);
    lv_display_flush_ready(disp);
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

    lv_demo_benchmark();

    while (1)
    {
        lv_tick_inc(20);
        lv_timer_handler();
        vTaskDelayUntil(&pxPreviousWakeTime, 20);
    }
    vTaskDelete(NULL);
}

int lvgl_init()
{
    HAL_GPIO_WritePin(LCD_BK_GPIO_Port, LCD_BK_Pin, GPIO_PIN_SET);
    // __HAL_LTDC_ENABLE_IT(&hltdc, LTDC_IT_LI);
    HAL_LTDC_SetAddress(&hltdc, (uint32_t)lcd_frame_buffer_1, 0);
    HAL_LTDC_SetWindowPosition_NoReload(&hltdc, 0, 0, 0);
    HAL_LTDC_SetWindowSize(&hltdc, 480, 272, 0);

    xTaskCreate(lvgl_task, "lvgl", 8192, NULL, 30, &lvgl_task_handle);
}

void lv_mem_init()
{

}

void * lv_malloc_core(size_t size)
{
    return app_malloc(size);
}

void lv_free_core(void * p)
{
    app_free(p);
}

void * lv_realloc_core(void * p, size_t new_size)
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

//     lv_disp_draw_buf_init(&draw_buf, lcd_frame_buffer_1, lcd_frame_buffer_2, 272 * 480);  /*Initialize the display buffer.*/

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
