#pragma once

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BALL_CLASS_NONE,
    BALL_CLASS_WEAPON,
    BALL_CLASS_ORBOTAL,
    BALL_CLASS_HANGAR,
    BALL_CLASS_BRIDGE,
    BALL_CLASS_ENGIEERING_BAY,
    BALL_CLASS_ROBOTICS_WORKSHOP,
    BALL_CLASS_CHEMICAL_AGENTS,
    BALL_CLASS_URBAN_LEGENDS,
    BALL_CLASS_OTHER,
} HellDiversBallClass_t;

typedef struct
{
    const char *text;
    const char *keys;
    const lv_image_dsc_t *image;
    HellDiversBallClass_t class;
} HellDiversBall_t;

extern const HellDiversBall_t hellDiversBalls[66];

#ifdef __cplusplus
}
#endif
