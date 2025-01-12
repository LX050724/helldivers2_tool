import os, sys

file_head = '''#pragma once
#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#elif defined(LV_BUILD_TEST)
#include "../lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

'''

if __name__ == '__main__':
    with open(sys.argv[2], 'w') as hearder_file:
        hearder_file.write(file_head)
        for file in os.scandir(sys.argv[1]):
            if not file.name.endswith('.png'):
                continue
            var_name = os.path.splitext(file.name)[0]
            print(f"extern const lv_image_dsc_t {var_name};", file=hearder_file)
