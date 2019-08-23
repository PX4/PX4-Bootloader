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
#include <libopencm3/cm3/systick.h>


#include "crypto.h"
#include "hw_config.h"

#include "monocypher/src/monocypher.h"

#define TEST_BUF_SIZE 1024


const char *oneKmsg="GSfSamANRZZNSixQVMwawIpy13J5gl2KcJikJjatDa\
					6n8ktpQluq1PBv0IrNV7OmwEXqkzm3aytZHAnT50A2dFLI1SDZR8nM\
					VsD5ashChyjnL3MikxdvBOzqvIjZhFKTPuwlKHPqrivDV34tkDD6Zi\
					UP0NV77yTuqLpWMZ2uAjDOemnGjkpTOpAKuSjJtsH7DFhBArndttMU\
					2WKXRArYTwwS45k0HthVV9iJs5PYd1P0QtHxyPxm4XjRvqB3aW3vTL\
					2rUn1oxn2kiIBls2lgXbxHuZll65vDjGftnzWY6pIP3xB59WQaImwX\
					p6z9HpsiyMh7HYk3IbudFlC9hbB8RrOnXh4N2nXyY0bxETVB4ZKoI4\
					uyhCVqYtHagS8o5vRuM3QmEey1KOyDjDGSOmRsH0o2rKeLABYYKzAW\
					ehTJLdFlWXGIQ3efwZ5ZJMH6ohwmmflNrJ8hbNxovb8LgnxEa6mKEP\
					F1XUU0JPRW2hOWDZLyVrSrwyH47sKQE2394iSSixfQAfBTI4sASsM6\
					IxeKeRsQxc2vH3xwMvZgWRrR3fmdBh2lWDbgbNYarG0vrECwCTt7Bj\
					87wcFO42X6qkIozNTVsm9j0LuX5Sq2WuGrAQiESdC8WCI3IhnDC9yC\
					CLSlB3Dw5NonC0rAKaXD9Xb4McjeH0cdM78XGYqlYJHi0NjJ2n19VV\
					2yKkkA4apZJovauBsuOsC8tybSqVrUQJgUQsJnUcxoMG6jUpiugpYM\
					Sm67muFOq9wclFcXelWHCJ8CPcp3OjCASXeHMbzwRj7qjxsX9MAubw\
					5joAohGg9hWWvwY8mcfL3bfncvciTxbzBDRfJHCiAjGlZJNgq578Gm\
					kaIy2qKLrJ0auorM3rUnOpRNO2WsrVnoeORzPw9l4p1zMA99kpyBRu\
					0Nmv2ranZPqFqnauSW0foe61zsiZCclXwZhZZdsKnyXPduIAw3oGtv\
					4kfSskKVluhCEGz5K5rE8kJiua1JMfiiCFKvMHe8eF0Y7N3r3Sj4KD\
					b74NlP8pJ2";


uint32_t sha512_bench(void)
{
    uint8_t hash[64];
    //const char  *msg="the quick brown fox jumps over the lazy dog";


    volatile uint32_t t1 = systick_get_value();

    //crypto_blake2b(hash,(const uint8_t *)oneKmsg, sizeof(oneKmsg));
    crypto_sha512(hash,(const uint8_t *)oneKmsg, TEST_BUF_SIZE);

    volatile uint32_t t2 = systick_get_value();
    volatile uint32_t d = t1-t2;
    volatile uint32_t u_sec = d/216;


	return u_sec;
}

uint32_t verify_signature(void)
{


	const uint8_t  signature[64]={0xaa, 0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
			0xaa, 0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
			0xaa, 0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
			0xaa, 0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
			0xaa, 0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
			0xaa, 0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
			0xaa, 0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
			0xaa, 0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};

	const uint8_t  public_key[32]={0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
			0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
			0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
			0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

	volatile uint32_t t1 = systick_get_value();
	int check = crypto_check( signature,
	                  public_key,
	                 (const uint8_t *)oneKmsg,
					 TEST_BUF_SIZE);

	volatile uint32_t t2 = systick_get_value();
	volatile uint32_t d = t1-t2;
	volatile uint32_t u_sec = d/216;
	if(check== 1)
	{

	}

	return u_sec;
}




void crypto_test_bench(void)
{
	volatile  uint32_t  results[7];

	results[0]= sha512_bench();
	results[1]=verify_signature();

	  if (results[0] == results[1]){
	    }
}

struct meta_data
	{
		size_t app_len;
		uint8_t hash[64];
	};

size_t get_app_len(void)
{
	const struct meta_data *meta_data_ptr= (const struct meta_data *)APP_MEM_AREA_START;

	return meta_data_ptr->app_len;

}

enum errno verifyApp(const uint32_t start_adr, const size_t application_len)
{
	enum errno ret = NO_ERROR;
	uint8_t hash[64];

	size_t len = get_app_len();

	const struct meta_data *meta_data_ptr= (const struct meta_data *)APP_MEM_AREA_START;

	crypto_sha512(hash,(const uint8_t *)APP_LOAD_ADDRESS, len);

	if (crypto_verify64((const uint8_t *)meta_data_ptr->hash, (const uint8_t *) hash ) <  0)
	{
		ret = INVALID_APPLICATION_HASH;
	}




	return ret;
}

