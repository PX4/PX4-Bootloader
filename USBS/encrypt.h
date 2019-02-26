/******************** (C) COPYRIGHT 2018 merafour ********************
* Author             : 冷月追风@merafour.blog.163.com
* Version            : V2.0.0
* Date               : 25/2/2019
* Description        : TEA encryption algorithm.
********************************************************************************
* merafour.blog.163.com
* merafour@163.com
* github.com/Merafour
* gitee.com/merafour
*******************************************************************************/

#ifndef _ENCRYPT_H_
#define _ENCRYPT_H_
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

extern uint8_t get_flag(const uint8_t pos);
extern void encryption(void (*read_uid)(uint32_t uid[3]));

#ifdef __cplusplus
}
#endif
#endif // _ENCRYPT_H_
