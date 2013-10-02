/**
 * CSP Radio interface using astrodev radios.
 *
 * @author Francisco Soto <francisco@nanosatisfi.com>
 *
 * Copyright (C) 2013 Nanosatisfi (http://www.nanosatisfi.com)
 */

#ifndef _CSP_IF_ASTRODEV_H_
#define _CSP_IF_ASTRODEV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <csp/csp.h>
#include <csp/csp_interface.h>

extern csp_iface_t csp_if_astrodev;

typedef int (*csp_astrodev_putstr_f)(uint8_t *buf, int len);

void csp_astrodev_rx (uint8_t *buf, int len);
int csp_astrodev_init (csp_astrodev_putstr_f astrodev_putstr_f);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _CSP_IF_RADIO_H_ */
