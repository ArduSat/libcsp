/*
*/

#ifndef _CSP_IF_RADIO_H_
#define _CSP_IF_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <csp/csp.h>
#include <csp/csp_interface.h>

extern csp_iface_t csp_if_radio;

typedef int (*csp_radio_putstr_f)(uint8_t *buf, int len);

void csp_radio_rx (uint8_t *buf, int len);
int csp_radio_init (csp_radio_putstr_f radio_putstr_f);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _CSP_IF_RADIO_H_ */
