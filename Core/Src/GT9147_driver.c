
#include "GT9147_driver.h"
#include "i2c.h"
#include "main.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_i2c.h"
#include <FreeRTOS.h>
#include <cmsis_os.h>
#include <semphr.h>
#include <stdint.h>
#include <string.h>
#include <sys/cdefs.h>

// #define GT_CMD_WR 0X28 // 写命令
// #define GT_CMD_RD 0X29 // 读命令
#define GT_CMD_WR 0xba // 写命令
#define GT_CMD_RD 0xbb // 读命令

#define GT_CTRL_REG 0X8040  // GT9147控制寄存器
#define GT_CFGS_REG 0X8047  // GT9147配置起始地址寄存器
#define GT_CHECK_REG 0X80FF // GT9147校验和寄存器
#define GT_PID_REG 0X8140   // GT9147产品ID寄存器

#define GT_GSTID_REG 0X814E // GT9147当前检测到的触摸情况
#define GT_TP1_REG 0X8150   // 第一个触摸点数据地址
#define GT_TP2_REG 0X8158   // 第二个触摸点数据地址
#define GT_TP3_REG 0X8160   // 第三个触摸点数据地址
#define GT_TP4_REG 0X8168   // 第四个触摸点数据地址
#define GT_TP5_REG 0X8170   // 第五个触摸点数据地址

static SemaphoreHandle_t iic_semaphore;
static SemaphoreHandle_t exit_semaphore;
static TaskHandle_t gt9147_task_handle;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t size;
    uint8_t track_id;
} GT9147_Point_t;

static uint8_t dma_buf[8] __attribute__((section(".noncacheable.bss")));
static GT9147_Point_t gt9147_points[5] __attribute__((section(".noncacheable.bss")));
static uint8_t gt9147_touch_num;


const uint8_t GT9147_CFG_TBL[] = {
    0X60, 0XE0, 0X01, 0X20, 0X03, 0X05, 0X35, 0X00, 0X02, 0X08, 0X1E, 0X08, 0X50, 0X3C, 0X0F, 0X05, 0X00, 0X00, 0XFF,
    0X67, 0X50, 0X00, 0X00, 0X18, 0X1A, 0X1E, 0X14, 0X89, 0X28, 0X0A, 0X30, 0X2E, 0XBB, 0X0A, 0X03, 0X00, 0X00, 0X02,
    0X33, 0X1D, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X32, 0X00, 0X00, 0X2A, 0X1C, 0X5A, 0X94, 0XC5, 0X02, 0X07,
    0X00, 0X00, 0X00, 0XB5, 0X1F, 0X00, 0X90, 0X28, 0X00, 0X77, 0X32, 0X00, 0X62, 0X3F, 0X00, 0X52, 0X50, 0X00, 0X52,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X0F, 0X0F, 0X03, 0X06, 0X10, 0X42, 0XF8, 0X0F, 0X14, 0X00, 0X00, 0X00, 0X00, 0X1A, 0X18,
    0X16, 0X14, 0X12, 0X10, 0X0E, 0X0C, 0X0A, 0X08, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X29, 0X28, 0X24, 0X22, 0X20, 0X1F, 0X1E, 0X1D, 0X0E, 0X0C,
    0X0A, 0X08, 0X06, 0X05, 0X04, 0X02, 0X00, 0XFF, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF,
};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LED_TS_INT_Pin)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(exit_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void gt9147_iic_complete_cb(I2C_HandleTypeDef *hi2c)
{
    UNUSED(hi2c);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(iic_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static int gt9147_read_reg(uint16_t regaddr, void *data, uint32_t len)
{
    HAL_I2C_Mem_Read_DMA(&hi2c1, GT_CMD_RD, regaddr, I2C_MEMADD_SIZE_16BIT, data, len);
    if (xSemaphoreTake(iic_semaphore, portMAX_DELAY) != pdPASS)
    {
        return -1;
    }
    return 0;
}

static int gt9147_write_reg(uint16_t regaddr, const void *data, uint32_t len)
{
    HAL_I2C_Mem_Write_DMA(&hi2c1, GT_CMD_WR, regaddr, I2C_MEMADD_SIZE_16BIT, (uint8_t *)data, len);
    if (xSemaphoreTake(iic_semaphore, portMAX_DELAY) != pdPASS)
    {
        return -1;
    }
    return 0;
}
uint8_t mode;

static void gt9147_task(void *args)
{
    UNUSED(args);
    HAL_GPIO_WritePin(LCD_TS_RST_GPIO_Port, LCD_TS_RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    HAL_GPIO_WritePin(LCD_TS_RST_GPIO_Port, LCD_TS_RST_Pin, GPIO_PIN_SET);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    gt9147_read_reg(GT_PID_REG, dma_buf, 4); // 读取产品ID

    dma_buf[0] = 2;
    gt9147_write_reg(GT_CTRL_REG, dma_buf, 1);
    gt9147_read_reg(GT_CFGS_REG, dma_buf, 1);

    if (dma_buf[0] < 0x60)
    {
        dma_buf[0] = 0;
        dma_buf[1] = 1; // 是否写入到GT9147 FLASH?  即是否掉电保存
        for (uint32_t i = 0; i < sizeof(GT9147_CFG_TBL); i++)
        {
            dma_buf[0] += GT9147_CFG_TBL[i]; // 计算校验和
        }
        dma_buf[0] = (~dma_buf[0]) + 1;

        gt9147_write_reg(GT_CFGS_REG, GT9147_CFG_TBL, sizeof(GT9147_CFG_TBL));
        gt9147_write_reg(GT_CHECK_REG, dma_buf, 2);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    dma_buf[0] = 0;
    gt9147_write_reg(GT_CTRL_REG, dma_buf, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    while (1)
    {
        if (xSemaphoreTake(exit_semaphore, portMAX_DELAY) != pdPASS)
            continue;
        
        gt9147_read_reg(GT_GSTID_REG, dma_buf, 1);
        mode = dma_buf[0];
        if ((mode & 0x80) == 0)
            continue;

        gt9147_touch_num = mode & 0x0f;
        dma_buf[0] = 0;
        gt9147_write_reg(GT_GSTID_REG, &dma_buf, 1); // 清标志
        if (gt9147_touch_num)
        {
            gt9147_read_reg(GT_TP1_REG, gt9147_points, sizeof(GT9147_Point_t) * gt9147_touch_num);
        }
    }
    vTaskDelete(NULL);
}

void gt9147_init()
{
    iic_semaphore = xSemaphoreCreateBinary();
    exit_semaphore = xSemaphoreCreateBinary();

    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MEM_RX_COMPLETE_CB_ID, gt9147_iic_complete_cb);
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MEM_TX_COMPLETE_CB_ID, gt9147_iic_complete_cb);

    xTaskCreate(gt9147_task, "gt9147", 256, NULL, 25, &gt9147_task_handle);
}
