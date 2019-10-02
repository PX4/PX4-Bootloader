/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file crypto.c
 *
 * Wrapper for the crytpo stuff.
 *
 */

#include <inttypes.h>
#include <stdbool.h>

#include "hw_config.h"
#include "public_key.h"
#include "monocypher/src/monocypher.h"

#include "crypto.h"

//prevents compile errors with non crypto btl
#ifndef APP_MEM_AREA_START
 #define APP_MEM_AREA_START APP_LOAD_ADDRESS
#endif

//data structure representing the meta data header of crypto images
struct meta_data
{
	size_t app_len;
	uint8_t signature[64];
};

size_t get_app_len(void)
{
	const struct meta_data *meta_data_ptr= (const struct meta_data *)APP_MEM_AREA_START;
	return meta_data_ptr->app_len;
}

uint8_t* get_app_signature(void){

	struct meta_data *meta_data_ptr= (struct meta_data *)APP_MEM_AREA_START;
	return meta_data_ptr->signature;
}

bool verifyApp(const size_t max_appl_len)
{
	bool ret = false;
	volatile uint8_t *app_signature_ptr = NULL;

	volatile size_t len = get_app_len();

	if (len > max_appl_len ){
		return false;
	}

	app_signature_ptr = get_app_signature();

	if( crypto_check((const uint8_t *)app_signature_ptr, public_key, (const uint8_t *)APP_LOAD_ADDRESS, len) == 0 ){
		ret = true;
	}

	return ret;
}
