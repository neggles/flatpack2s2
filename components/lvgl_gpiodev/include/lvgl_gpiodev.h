/**
 * @file lvgl_gpiodev.h
 */

#ifndef _LVGL_GPIODEV_H
#define _LVGL_GPIODEV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

void lvgl_gpiodev_init(void);
bool lvgl_gpiodev_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LVGL_GPIODEV_H */