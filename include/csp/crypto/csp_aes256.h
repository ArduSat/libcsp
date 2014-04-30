#ifndef _CSP_AES256_H_
#define _CSP_AES256_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Simple wrapper around CSP packet for convenient encryption/decryption,transmission
typedef struct {
	uint32_t iv_nonce[2];
	uint8_t len;	//CSP packets can't be bigger than 255 bytes
	uint8_t data[0];
} aes256_packet;

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

void aes256_packet_encrypt(aes256_packet * packet);

void aes256_packet_decrypt(aes256_packet * packet);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // _CSP_AES256_H_
