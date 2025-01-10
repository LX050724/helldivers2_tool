#pragma once

#include <sys/cdefs.h>

#ifdef __cplusplus
extern "C" {
#endif

void *app_malloc(size_t xWantedSize);
void app_free(void *pv);
void *app_realloc(void *p, size_t s);


#ifdef __cplusplus
}
#endif
