#ifndef _CSP_AES256_H_
#define _CSP_AES256_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * AES256 encrypt byte array
 * @param plain Pointer to plain text
 * @param len Length of plain text
 * @param iv Initialization vector
 */
int csp_aes256_encrypt(uint8_t * plain, const uint32_t len, uint32_t iv[4]);

/**
 * Decrypt XTEA encrypted byte array
 * @param cipher Pointer to cipher text
 * @param len Length of plain text
 * @param iv Initialization vector
 */
int csp_aes256_decrypt(uint8_t * plain, const uint32_t len, uint32_t iv[4]);

/** 
*  Set aes256 encryption key
*  @param key Pointer to encryption key
*  @param keylen Length of encryption key (must be AES256_KEYLENGTH i.e. 32)
*/
int csp_aes256_set_key(char * key, uint32_t keylen);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // _CSP_AES256_H_
