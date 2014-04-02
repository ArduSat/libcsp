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

typedef int (*csp_astrodev_putstr_f)(uint8_t *buf, int len);

#define ASTRODEV_PACKET_SIZE 255
#define NUM_ASTRODEV_MODULES 2

typedef struct __attribute__((__packed__)) {
    uint8_t dst_callsign[6];
    uint8_t dst_ssid;
    uint8_t src_callsign[6];
    uint8_t src_ssid;
    uint8_t digi_callsign[6];
    uint8_t digi_ssid;
    uint8_t control;
    uint8_t pid;
} ax25_header_t;

typedef struct csp_astrodev_handle_s {
    csp_astrodev_putstr_f radio_tx;
    bool use_ax25_header;
    uint8_t module;
} csp_astrodev_handle_t;

void csp_astrodev_rx (csp_iface_t *interface,
                      uint8_t *buf, int len, void *xTaskWoken);
void csp_astrodev_init (csp_iface_t *csp_iface, csp_astrodev_handle_t *csp_astrodev_handle,
                        csp_astrodev_putstr_f astrodev_putstr, const char *name);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _CSP_IF_RADIO_H_ */
