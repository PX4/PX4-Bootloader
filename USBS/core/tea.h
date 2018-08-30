/******************** (C) COPYRIGHT 2018 merafour ********************
* Author             : 冷月追风@merafour.blog.163.com
* Version            : V2.0.0
* Date               : 29/8/2018
* Description        : TEA encryption algorithm.
********************************************************************************
* merafour.blog.163.com
* merafour@163.com
* github.com/Merafour
*******************************************************************************/

#ifndef _TEA_H_
#define _TEA_H_
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

extern unsigned char tea_key[16];
//extern void set_Iteration(const uint8_t Iter);
//extern void tea_encrypt(const uint32_t* v, const uint32_t* k, uint32_t* vout);
//extern void tea_decrypt(const uint32_t* v, const uint32_t* k, uint32_t* vout);
extern void tea_encrypt(uint32_t* v, const uint32_t* k);
extern void tea_decrypt(uint32_t* v, const uint32_t* k);
extern void _tea_encrypt(const uint32_t* v, const uint32_t* k, uint32_t* vout);
extern void _tea_decrypt(const uint32_t* v, const uint32_t* k, uint32_t* vout);
#ifdef __cplusplus
}
#endif
#endif // _TEA_H_
