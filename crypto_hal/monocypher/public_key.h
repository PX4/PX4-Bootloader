/****************************************************************************
 *
 *   Copyright (c) 2020 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file public_key.h
 *
 * File holds public keys for signed firmware.
 *
 *
 */

#pragma once

#define XSTR(x) #x
#define STR(x) XSTR(x)

#ifndef PUBLIC_KEY0
#error "At least one key (PUBLIC_KEY0) must be defined"
#endif

/* This constant only exists to calculate size of the
   key. It will be removed by the linker */
static const uint8_t public_key0[] = {
#include STR(PUBLIC_KEY0)
};

static const uint8_t public_keys[][sizeof(public_key0)] = {

	{
#include STR(PUBLIC_KEY0)
	}

#ifdef PUBLIC_KEY1
	,
	{
#include STR(PUBLIC_KEY1)
	}
#endif

#ifdef PUBLIC_KEY2
	,
	{
#include STR(PUBLIC_KEY2)
	}
#endif

#ifdef PUBLIC_KEY3
	,
	{
#include STR(PUBLIC_KEY3)
	}
#endif

};

#define NUMBER_OF_KEYS (sizeof(public_keys) / sizeof(public_keys[0]))

static inline const uint8_t *get_pubkey_by_index(uint8_t idx)
{
	if (idx < NUMBER_OF_KEYS) {
		return public_keys[idx];

	} else {
		return NULL;
	}
}
