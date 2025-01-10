#pragma once

#include <sys/cdefs.h>

#ifdef __cplusplus
extern "C" {
#endif

void *app_malloc(size_t xWantedSize);
void app_free(void *pv);
void *app_realloc(void *p, size_t s);

#define AT_NONCACHEABLE __attribute__((section(".noncacheable")))
#define AT_NONCACHEABLE_INIT __attribute__((section(".noncacheable.bss")))
#define AT_NONCACHEABLE_BSS __attribute__((section(".noncacheable.init")))

#define AT_NONCACHEABLE_ALIGN(N) __attribute__((section(".noncacheable"), aligned(N)))
#define AT_NONCACHEABLE_INIT_ALIGN(N) __attribute__((section(".noncacheable.bss"), aligned(N)))
#define AT_NONCACHEABLE_BSS_ALIGN(N) __attribute__((section(".noncacheable.init"), aligned(N)))

#define AT_EXSDRAM __attribute__((section(".sdram")))
#define AT_EXSDRAM_BSS __attribute__((section(".sdram.bss")))

#define AT_EXSDRAM_ALIGN(N) __attribute__((section(".sdram"), aligned(N)))
#define AT_EXSDRAM_BSS_ALIGN(N) __attribute__((section(".sdram.bss"), aligned(N)))

#define FAST_FUNC __attribute__((section(".fast_ram")))

#ifdef __cplusplus
}
#endif
