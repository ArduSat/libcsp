/*
*   Byte-oriented AES-256 implementation.
*   All lookup tables replaced with 'on the fly' calculations. 
*
*   Copyright (c) 2007-2009 Ilya O. Levin, http://www.literatecode.com
*   Other contributors: Hal Finney
*
*   Permission to use, copy, modify, and distribute this software for any
*   purpose with or without fee is hereby granted, provided that the above
*   copyright notice and this permission notice appear in all copies.
*
*   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
*   WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
*   ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
*   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
*   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
*   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

/*
*   HACK: Due to licensing restrictions and my own tardiness, Lemur-1 requires
*   AES256 instead of XTEA encryption.  This is gross, but to minimize code
*   and protocol changes, we simply substitute AES256 for XTEA, keeping function
*   names and protocol flags the same.  You should not see this file for anything
*   past Lemur-2.
*/
#include <stdint.h>

#include <csp/csp.h>

#include "csp_xtea.h"
#include <csp/crypto/csp_aes256.h>

#ifdef CSP_USE_XTEA

int csp_xtea_set_key(char * key, uint32_t keylen) {
    return csp_aes256_set_key(key, keylen);
}

int csp_xtea_encrypt(uint8_t * plain, const uint32_t len, uint32_t iv[2]) {
    uint32_t aes_iv[4];

    aes_iv[0] = iv[0];
    aes_iv[1] = 0;
    aes_iv[2] = 0;
    aes_iv[3] = iv[1];

    return csp_aes256_encrypt(plain, len, aes_iv);
}

int csp_xtea_decrypt(uint8_t * cipher, const uint32_t len, uint32_t iv[2]) {
    uint32_t aes_iv[4];

    aes_iv[0] = iv[0];
    aes_iv[1] = 0;
    aes_iv[2] = 0;
    aes_iv[3] = iv[1];

    return csp_aes256_decrypt(cipher, len, aes_iv);
}

#endif //CSP_USE_XTEA