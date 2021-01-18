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
 * @file crypto.c
 *
 * Wrapper for the crytpo stuff.
 *
 */

#include <inttypes.h>
#include <stdbool.h>

#include "monocypher/src/optional/monocypher-ed25519.h"
#include "crypto_hal/monocypher/public_key.h"
#include "crypto_hal/image_toc.h"

bool verify_app(uint16_t idx, const image_toc_entry_t *toc_entries)
{
	volatile uint8_t *app_signature_ptr = NULL;
	volatile size_t len = 0;

	uint8_t sig_idx = toc_entries[idx].signature_idx;
	uint8_t sig_key = toc_entries[idx].signature_key;

	const uint8_t *public_key = get_pubkey_by_index(sig_key);

	if (public_key == NULL) {
		return false;
	}

	app_signature_ptr = (volatile uint8_t *)toc_entries[sig_idx].start;
	len = (size_t)toc_entries[idx].end - (size_t)toc_entries[idx].start;

	if (crypto_ed25519_check((const uint8_t *)app_signature_ptr, public_key, (const uint8_t *)toc_entries[idx].start,
				 len) == 0) {
		return true;
	}

	return false;
}

bool decrypt_app(uint16_t idx, const image_toc_entry_t *toc_entries)
{
	/* We don't support embedding private keys to the bootloader.
	 * This function exists only for implementing the complete api.
	 * With monocypher it could be possible to embed a symmetric key
	 * to the bootloader and implement RFC 8439 w. xchacha20
	 * decryption using crypto_unlock(...);
	 * but the security of a system where private keys are directly
	 * embedded into bootloader code would be questionable
	 */

	return false;
}
