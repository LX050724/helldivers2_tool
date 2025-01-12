#include "SEGGER_RTT.h"
#include "helldivers.h"
#include "lvgl.h"
#include "stm32_hal_legacy.h"
#include <lvgl_app.h>
#include <src/widgets/label/lv_label.h>
#include <src/widgets/tabview/lv_tabview.h>
#include <stdint.h>
#include <src\core\lv_obj_private.h>

LV_FONT_DECLARE(lv_font_simhei_16)
LV_FONT_DECLARE(lv_font_simhei_12)

static lv_obj_t *select_boxs[sizeof(hellDiversBalls) / sizeof(hellDiversBalls[0])];

int usb_push_keyseq(const char * const*seq, TickType_t xTicksToWait);

void select_box_change_event(lv_event_t *e)
{
    const HellDiversBall_t *ball = lv_event_get_user_data(e);
    lv_obj_t *obj = lv_event_get_target(e);
    LV_LOG_USER("check %s %d\n", ball->text, lv_obj_has_state(obj, LV_STATE_CHECKED));
}

void send_keys_event(lv_event_t *e)
{
    const HellDiversBall_t *ball = lv_event_get_user_data(e);
    usb_push_keyseq(&ball->keys, 30);
    LV_LOG_USER("send %s %s\n", ball->text, ball->keys);
}

lv_obj_t *create_select_box(lv_obj_t *parent, const HellDiversBall_t *ball)
{
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_set_style_text_font(box, &lv_font_simhei_16, 0);
    lv_obj_remove_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(box, lv_obj_get_content_width(parent), LV_SIZE_CONTENT);
    lv_obj_set_user_data(box, (void *)ball);

    lv_obj_t *img = lv_image_create(box);
    lv_image_set_src(img, ball->image);
    lv_obj_align_to(img, box, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *text = lv_label_create(box);
    lv_label_set_text_static(text, ball->text);
    lv_label_set_long_mode(text, LV_LABEL_LONG_SCROLL);
    lv_obj_align_to(text, img, LV_ALIGN_OUT_RIGHT_TOP, 5, lv_obj_get_height(text));
    lv_obj_set_width(text, lv_obj_get_content_width(box) - lv_obj_get_width(img) - 6);

    lv_obj_t *sw = lv_switch_create(box);
    lv_obj_align_to(sw, img, LV_ALIGN_OUT_RIGHT_BOTTOM, 5, -lv_obj_get_height(sw));
    lv_obj_add_event_cb(sw, select_box_change_event, LV_EVENT_VALUE_CHANGED, (void *)ball);
    lv_obj_set_size(sw, lv_obj_get_width(sw) - 6, lv_obj_get_height(sw) - 3);

    lv_obj_t *button = lv_button_create(box);
    lv_obj_t *button_label = lv_label_create(button);
    lv_obj_set_height(button, lv_obj_get_height(sw));
    lv_label_set_text_static(button_label, "发送");
    lv_obj_align_to(button, sw, LV_ALIGN_OUT_RIGHT_TOP, 3, 0);
    lv_obj_add_event_cb(button, send_keys_event, LV_EVENT_CLICKED, (void *)ball);

    return box;
}

HellDiversBallClass_t get_ball_class(lv_obj_t *obj)
{
    const HellDiversBall_t *ball = lv_obj_get_user_data(obj);
    return ball ? ball->class : BALL_CLASS_NONE;
}

int lvgl_app_init()
{
    lv_obj_t *tab_view = lv_tabview_create(lv_screen_active());
    lv_tabview_set_tab_bar_position(tab_view, LV_DIR_LEFT);
    lv_tabview_set_tab_bar_size(tab_view, 80);
    lv_obj_set_style_text_font(tab_view, &lv_font_simhei_12, 0);

    const char *tabs_name[] = {"主页",   "武器",     "轨道加农炮", "机库",     "舰桥",
                               "工程港", "机械工坊", "化学专家",   "都市传奇", "其他"};
    lv_obj_t *tab_pages[sizeof(tabs_name) / sizeof(tabs_name[0])];

    for (uint32_t i = 0; i < sizeof(tabs_name) / sizeof(tabs_name[0]); i++)
    {
        tab_pages[i] = lv_tabview_add_tab(tab_view, tabs_name[i]);
        SEGGER_RTT_printf(0, "generate %d\n", i);
    }

    lv_obj_t *last_box = NULL;
    lv_obj_t *parent = NULL;
    for (const HellDiversBall_t *ball = hellDiversBalls; ball->text != NULL; ball++)
    {
        lv_obj_t *box = create_select_box(tab_pages[ball->class], ball);
        select_boxs[ball - hellDiversBalls] = box;
        if (parent != tab_pages[ball->class])
        {
            parent = tab_pages[ball->class];
            last_box = lv_label_create(parent);
            lv_label_set_text_static(last_box, tabs_name[ball->class]);
        }
        if (last_box)
        {
            lv_obj_align_to(select_boxs[ball - hellDiversBalls], last_box, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 3);
        }
        last_box = box;
    }

    return 0;
}
