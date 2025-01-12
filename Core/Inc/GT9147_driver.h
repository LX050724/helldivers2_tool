#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t size;
    uint8_t track_id;
} GT9147_Point_t;

void gt9147_init();
int gt9147_get_touch(GT9147_Point_t *points, int buf_num);

#ifdef __cplusplus
}
#endif
