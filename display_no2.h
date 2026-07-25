/*
 * Compatibility support for LishuiFOC "No.2" UART2 display protocol
 */

#ifndef DISPLAY_NO2_H
#define DISPLAY_NO2_H

#include "config.h"

#ifdef DISPLAY_TYPE_NO2
void display_init(void);
void display_update(void);
#endif

#endif /* DISPLAY_NO2_H */
