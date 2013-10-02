/**
 * CSP Radio interface using astrodev radios.
 *
 * @author Francisco Soto <francisco@nanosatisfi.com>
 *
 * Copyright (C) 2013 Nanosatisfi (http://www.nanosatisfi.com)
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <csp/csp.h>
#include <csp/csp_endian.h>
#include <csp/csp_platform.h>
#include <csp/csp_interface.h>
#include <csp/interfaces/csp_if_astrodev.h>
#include <csp/arch/csp_malloc.h>

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

static csp_astrodev_putstr_f radio_tx = NULL;

static int csp_astrodev_tx (csp_packet_t * packet, uint32_t timeout) {
    int ret = CSP_ERR_NONE;
    int txbufin = packet->length + CSP_HEADER_LENGTH;
    uint8_t *txbuf = csp_malloc(txbufin);

    if (txbuf == NULL)
        return CSP_ERR_NOMEM;

    /* Save the outgoing id in the buffer */
    packet->id.ext = csp_hton32(packet->id.ext);

    memcpy(txbuf, &packet->id.ext, txbufin);

    csp_buffer_free(packet);

    /* The packet goes straigth to the radio. */
    if (radio_tx(txbuf, txbufin) != 0) {
        ret = CSP_ERR_TIMEDOUT;
    }

    csp_free(txbuf);

    return ret;
}

void csp_astrodev_rx (uint8_t *buf, int len) {
    csp_packet_t *packet;
    ax25_header_t radio_header;

    if (len < (int)sizeof(ax25_header_t) + CSP_HEADER_LENGTH) {
        csp_log_warn("Weird radio frame received! Size %u\r\n", len);
    }

    memcpy(&radio_header, buf, sizeof(ax25_header_t));

    /* Strip off the AX.25 header. */
    buf += sizeof(ax25_header_t);
    len -= sizeof(ax25_header_t);

    packet = csp_buffer_get(csp_if_astrodev.mtu);

    if (packet != NULL) {
        memcpy(&packet->id.ext, buf, len);

        packet->length = len;

        csp_if_astrodev.frame++;

        if (packet->length >= CSP_HEADER_LENGTH &&
            packet->length <= csp_if_astrodev.mtu + CSP_HEADER_LENGTH) {

            /* Strip the CSP header off the length field before converting to CSP packet */
            packet->length -= CSP_HEADER_LENGTH;

            /* Convert the packet from network to host order */
            packet->id.ext = csp_ntoh32(packet->id.ext);

            /* Send back into CSP, notice calling from task so last argument must be NULL! */
            csp_new_packet(packet, &csp_if_astrodev, NULL);
        }
        else {
            csp_log_warn("Weird radio frame received! Size %u\r\n", packet->length);
            csp_buffer_free(packet);
        }
    }
}

int csp_astrodev_init (csp_astrodev_putstr_f astrodev_putstr_f) {
    radio_tx = astrodev_putstr_f;

    /* Register interface */
    csp_route_add_if(&csp_if_astrodev);

    return CSP_ERR_NONE;
}

/** Interface definition */
csp_iface_t csp_if_astrodev = {
    .name = "ASTRODEV",
    .nexthop = csp_astrodev_tx,
    .mtu = 255 - CSP_HEADER_LENGTH,
};
