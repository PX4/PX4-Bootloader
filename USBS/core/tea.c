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

#include "tea.h"
unsigned char tea_key[16]=
{
    0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
    0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10
};
static const uint8_t Iteration = 32;
//void set_Iteration(const uint8_t Iter)
//{
//    Iteration = Iter;
//}
#if 0
void tea_encrypt(uint32_t* v, const uint32_t* k)
{
    uint32_t v0 = v[0], v1 = v[1], sum = 0, i; /*setup*/
    uint32_t delta = 0x9e3779b9; /*akeyscheduleconstant*/
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3]; /*cachekey*/
    for(i=0;i<32;i++) {
        /*basiccyclestart*/
        sum += delta;
        v0 += ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        v1 += ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
    } /*endcycle*/
    v[0] = v0;
    v[1] = v1;
}

void tea_decrypt(uint32_t* v, const uint32_t* k)
{
    uint32_t v0 = v[0], v1 = v[1], sum = 0xC6EF3720, i; /*setup*/
    uint32_t delta = 0x9e3779b9; /*akeyscheduleconstant*/
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3]; /*cachekey*/
    for(i = 0; i < 32; i++) {
        /*basiccyclestart*/
        v1 -= ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
        v0 -= ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        sum -= delta;
    } /*endcycle*/
    v[0] = v0;
    v[1] = v1;
}
void _tea_encrypt(const uint32_t* v, const uint32_t* k, uint32_t* vout)
{
    uint32_t v0 = v[0], v1 = v[1], sum = 0, i; /*setup*/
    uint32_t delta = 0x9e3779b9; /*akeyscheduleconstant*/
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3]; /*cachekey*/
    for(i=0;i<32;i++) {
        /*basiccyclestart*/
        sum += delta;
        v0 += ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        v1 += ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
    } /*endcycle*/
    vout[0] = v0;
    vout[1] = v1;
}

void _tea_decrypt(const uint32_t* v, const uint32_t* k, uint32_t* vout)
{
    uint32_t v0 = v[0], v1 = v[1], sum = 0xC6EF3720, i; /*setup*/
    uint32_t delta = 0x9e3779b9; /*akeyscheduleconstant*/
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3]; /*cachekey*/
    for(i = 0; i < 32; i++) {
        /*basiccyclestart*/
        v1 -= ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
        v0 -= ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        sum -= delta;
    } /*endcycle*/
    vout[0] = v0;
    vout[1] = v1;
}
#else
#define _delta_mask  0xbdc3931aU
static const uint32_t _delta = 0x9e3779b9^_delta_mask;
void tea_encrypt(uint32_t* v, const uint32_t* k)
{
    uint32_t v0 = v[0], v1 = v[1], sum = 0, i; /*setup*/
    //uint32_t delta = 0x9e3779b9; /*akeyscheduleconstant*/
    uint32_t delta = _delta^_delta_mask; /*akeyscheduleconstant*/
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3]; /*cachekey*/
    for(i=0;i<Iteration;i++) {
        /*basiccyclestart*/
        sum += delta;
        v0 += ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        v1 += ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
    } /*endcycle*/
    v[0] = v0;
    v[1] = v1;
}

void tea_decrypt(uint32_t* v, const uint32_t* k)
{
    uint32_t v0 = v[0], v1 = v[1], sum = 0xC6EF3720, i; /*setup*/
    //uint32_t delta = 0x9e3779b9; /*akeyscheduleconstant*/
    uint32_t delta = _delta^_delta_mask; /*akeyscheduleconstant*/
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3]; /*cachekey*/
    for(i = 0; i < Iteration; i++) {
        /*basiccyclestart*/
        v1 -= ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
        v0 -= ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        sum -= delta;
    } /*endcycle*/
    v[0] = v0;
    v[1] = v1;
}
void _tea_encrypt(const uint32_t* v, const uint32_t* k, uint32_t* vout)
{
    uint32_t v0 = v[0], v1 = v[1], sum = 0, i; /*setup*/
    uint32_t delta = 0x9e3779b9; /*akeyscheduleconstant*/
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3]; /*cachekey*/
    for(i=0;i<Iteration;i++) {
        /*basiccyclestart*/
        sum += delta;
        v0 += ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        v1 += ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
    } /*endcycle*/
    vout[0] = v0;
    vout[1] = v1;
}

void _tea_decrypt(const uint32_t* v, const uint32_t* k, uint32_t* vout)
{
    uint32_t v0 = v[0], v1 = v[1], sum = 0xC6EF3720, i; /*setup*/
    uint32_t delta = 0x9e3779b9; /*akeyscheduleconstant*/
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3]; /*cachekey*/
    for(i = 0; i < Iteration; i++) {
        /*basiccyclestart*/
        v1 -= ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
        v0 -= ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        sum -= delta;
    } /*endcycle*/
    vout[0] = v0;
    vout[1] = v1;
}
#endif

