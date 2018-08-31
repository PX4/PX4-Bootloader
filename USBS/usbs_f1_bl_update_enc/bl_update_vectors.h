/******************** (C) COPYRIGHT 2018 merafour ********************
* Author             : 冷月追风@merafour.blog.163.com
* Version            : V1.0.0
* Date               : 30/8/2018
* Description        : vectors.
********************************************************************************
* merafour.blog.163.com
* merafour@163.com
* github.com/Merafour
*******************************************************************************/

#ifndef _BL_UPDATE_VECTORS_H_
#define _BL_UPDATE_VECTORS_H_

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>

/* load optional platform dependent initialization routines */
#include "../libopencm3/lib/dispatch/vector_chipset.c"
/* load the weak symbols for IRQ_HANDLERS */
#include "../libopencm3/lib/dispatch/vector_nvic.c"

#endif  // _BL_UPDATE_VECTORS_H_
